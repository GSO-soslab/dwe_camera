import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Duration
import cv2

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from dwe_camera_interfaces.msg import CamParameters

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('dwe_camera_node')

        self.dwe_camera = None  # Initialize camera object to None
        self.latest_raw_frame = None # To store the latest frame for other callbacks
        self.latest_header = Header() # To store the latest header

        # Timers
        self.raw_image_timer = None
        self.compressed_image_timer = None
        self.param_update_timer = None

        try:
            self.setup_params()
            self.setup_cam()  # This can raise RuntimeError
            self.setup_ros_elements() # Publishers and messages
            self.setup_timers()       # Timers based on parameters
            self.get_logger().info("DWE Camera Node successfully initialized.")
        except Exception as e:
            self.get_logger().error(f"Error during node initialization: {e}")
            self.cleanup_resources() # Clean up any resources partially initialized
            raise  # Re-raise the exception to stop node creation / inform main

    def cleanup_resources(self):
        print(f"[{self.get_name()}] INFO: Executing resource cleanup...")
        if self.raw_image_timer and not self.raw_image_timer.is_canceled():
            self.raw_image_timer.cancel()
            print(f"[{self.get_name()}] INFO: Raw image timer cancelled.")
        if self.compressed_image_timer and not self.compressed_image_timer.is_canceled():
            self.compressed_image_timer.cancel()
            print(f"[{self.get_name()}] INFO: Compressed image timer cancelled.")
        if self.param_update_timer and not self.param_update_timer.is_canceled():
            self.param_update_timer.cancel()
            print(f"[{self.get_name()}] INFO: Parameter update timer cancelled.")

        if self.dwe_camera and self.dwe_camera.isOpened():
            self.dwe_camera.release()
            print(f"[{self.get_name()}] INFO: Camera released.")

    def setup_params(self):
        # ros related
        self.declare_parameter('ros.frame_id', '/dwe_camera')
        self.declare_parameter('ros.param_update_rate', 1.0) # Hz for camera param updates
        # video related
        self.declare_parameters(
            namespace='',
            parameters=[
                ('video.id', 0),
                ('video.width', 1600),
                ('video.height', 1200),
                ('video.framerate', 15),
                ('video.format', 'MJPG'),
                ('video.jpeg_quality_recompress', 95)],)
        # compression related
        self.declare_parameters(
            namespace='',
            parameters=[
                ('compression.width', 320),
                ('compression.height', 240),
                ('compression.target_fps', 5),
                ('compression.jpeg_quality', 75)],)
        # camera related
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera.brightness', 0),
                ('camera.contrast', 32),
                ('camera.saturation', 64),
                ('camera.hue', 0),
                ('camera.gamma', 100),
                ('camera.gain', 0),
                ('camera.sharpness', 3),
                ('camera.auto_exposure', True),
                ('camera.exposure_time', 156)],)

    def setup_cam(self):
        [CAM_IDX_param, WIDTH_param, HEIGHT_param, CAM_FPS_param, VIDEO_FORMAT_param] = self.get_parameters(
            ['video.id', 'video.width', 'video.height', 'video.framerate', 'video.format'])
        
        CAM_IDX = CAM_IDX_param.value
        WIDTH = WIDTH_param.value
        HEIGHT = HEIGHT_param.value
        CAM_FPS_REQUEST = CAM_FPS_param.value
        VIDEO_FORMAT = VIDEO_FORMAT_param.value

        self.get_logger().info(f"Attempting to open camera {CAM_IDX} with V4L2 backend.")
        self.dwe_camera = cv2.VideoCapture(CAM_IDX, cv2.CAP_V4L2)

        if not self.dwe_camera or not self.dwe_camera.isOpened():
            raise RuntimeError(f"Failed to open video device {CAM_IDX}")

        # Set video format
        if len(VIDEO_FORMAT) == 4:
            self.get_logger().info(f"Setting FOURCC format to: {VIDEO_FORMAT}")
            self.dwe_camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*VIDEO_FORMAT))
        else:
            self.get_logger().warn(f"Video format '{VIDEO_FORMAT}' is not 4 characters long. Skipping FOURCC setting.")

        self.dwe_camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.dwe_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.dwe_camera.set(cv2.CAP_PROP_FPS, CAM_FPS_REQUEST)

        # Initial exposure setting
        auto_exposure_param = self.get_parameter('camera.auto_exposure')
        exposure_time_param = self.get_parameter('camera.exposure_time')

        if auto_exposure_param.value:
            self.get_logger().info("Setting V4L2 auto exposure mode (3 - Aperture Priority)")
            self.dwe_camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        else:
            self.get_logger().info(f"Setting V4L2 manual exposure mode (1), exposure time: {exposure_time_param.value}")
            self.dwe_camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.dwe_camera.set(cv2.CAP_PROP_EXPOSURE, exposure_time_param.value)

        # Store actual camera FPS
        self.CAM_FPS = self.dwe_camera.get(cv2.CAP_PROP_FPS)
        if self.CAM_FPS == 0 : # If driver doesn't report FPS, use requested
            self.get_logger().warn(f"Camera driver did not report FPS, using requested FPS: {CAM_FPS_REQUEST}. Raw image publishing might be inaccurate.")
            self.CAM_FPS = float(CAM_FPS_REQUEST)
        if self.CAM_FPS <= 0:
            raise RuntimeError(f"Camera FPS is {self.CAM_FPS}, which is invalid. Cannot proceed.")

        actual_width = self.dwe_camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.dwe_camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fourcc_val = self.dwe_camera.get(cv2.CAP_PROP_FOURCC)
        try:
            fourcc_str = "".join([chr((int(fourcc_val) >> 8 * i) & 0xFF) for i in range(4)]).strip()
        except:
            fourcc_str = "N/A"
        expo_time = self.dwe_camera.get(cv2.CAP_PROP_EXPOSURE)
        auto_expo = self.dwe_camera.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        self.get_logger().info(f"Actual Resolution: {actual_width}x{actual_height}, FPS: {self.CAM_FPS}, Format: {fourcc_str}, "
                               f"Auto Exposure: {auto_expo}, Exposure Time: {expo_time}")
        
        # Call once to set initial non-exposure parameters
        self.apply_camera_parameters()

    def setup_ros_elements(self):
        # header and frame
        self.latest_header.frame_id = self.get_parameter('ros.frame_id').value

        # publishers
        self.image_pub = self.create_publisher(CompressedImage, "image_mjpg", 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, "image_lowbw/compressed", 10)
        self.cam_settings_pub = self.create_publisher(CamParameters, "camera_settings", 10)

        # message instances
        self.cam_settings_msg = CamParameters()

    def setup_timers(self):
        # Raw Image Timer
        raw_image_timer_period = 1.0 / self.CAM_FPS
        self.raw_image_timer = self.create_timer(raw_image_timer_period, self.raw_image_capture_callback)
        self.get_logger().info(f"Raw image timer set to {self.CAM_FPS:.2f} Hz.")

        # Compressed Image Timer
        compressed_target_fps = self.get_parameter('compression.target_fps').value
        if compressed_target_fps > 0:
            compressed_image_timer_period = 1.0 / compressed_target_fps
            self.compressed_image_timer = self.create_timer(compressed_image_timer_period, self.compressed_image_callback)
            self.get_logger().info(f"Compressed image timer set to {compressed_target_fps:.2f} Hz.")
        else:
            self.get_logger().warn("compression.target_fps <= 0, compressed image publisher disabled.")

        # Parameter Update Timer
        param_update_rate = self.get_parameter('ros.param_update_rate').value
        if param_update_rate > 0:
            param_update_timer_period = 1.0 / param_update_rate
            self.param_update_timer = self.create_timer(param_update_timer_period, self.parameter_update_callback)
            self.get_logger().info(f"Parameter update timer set to {param_update_rate:.2f} Hz.")
        else:
            self.get_logger().warn("ros.param_update_rate <= 0, dynamic parameter updates disabled.")

    def apply_camera_parameters(self):
        """Applies current ROS parameters to the camera. Separated for clarity."""
        params_to_set = {
            'camera.brightness': cv2.CAP_PROP_BRIGHTNESS,
            'camera.contrast': cv2.CAP_PROP_CONTRAST,
            'camera.saturation': cv2.CAP_PROP_SATURATION,
            'camera.hue': cv2.CAP_PROP_HUE,
            'camera.gamma': cv2.CAP_PROP_GAMMA,
            'camera.gain': cv2.CAP_PROP_GAIN,
            'camera.sharpness': cv2.CAP_PROP_SHARPNESS,
        }
        changed_params = []
        for param_name, cv_prop in params_to_set.items():
            param_value = self.get_parameter(param_name).value
            current_cv_val = self.dwe_camera.get(cv_prop)
            if current_cv_val != param_value:
                self.dwe_camera.set(cv_prop, param_value)
                changed_params.append(f"{param_name}={param_value}")

        auto_exposure = self.get_parameter('camera.auto_exposure').value
        exposure_time = self.get_parameter('camera.exposure_time').value
        current_auto_exposure_mode_cv = self.dwe_camera.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        target_auto_exposure_mode_cv = 3 if auto_exposure else 1

        if current_auto_exposure_mode_cv != target_auto_exposure_mode_cv:
            self.dwe_camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, target_auto_exposure_mode_cv)
            changed_params.append(f"camera.auto_exposure_mode_cv={target_auto_exposure_mode_cv}")

        if not auto_exposure:
            current_exposure_time_cv = self.dwe_camera.get(cv2.CAP_PROP_EXPOSURE)
            if current_exposure_time_cv != exposure_time :
                self.dwe_camera.set(cv2.CAP_PROP_EXPOSURE, exposure_time)
                changed_params.append(f"camera.exposure_time={exposure_time}")
        
        if changed_params:
            self.get_logger().debug(f"Applied camera parameters: {', '.join(changed_params)}")

    def parameter_update_callback(self):
        """Periodically applies ROS parameters to the camera and publishes current settings."""
        if self.dwe_camera and self.dwe_camera.isOpened():
            self.apply_camera_parameters()

            # Read current settings from camera and publish them
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.latest_header.frame_id # Use consistent frame_id
            self.cam_settings_msg.header = header
            self.cam_settings_msg.brightness = int(self.dwe_camera.get(cv2.CAP_PROP_BRIGHTNESS))
            self.cam_settings_msg.contrast = int(self.dwe_camera.get(cv2.CAP_PROP_CONTRAST))
            self.cam_settings_msg.saturation = int(self.dwe_camera.get(cv2.CAP_PROP_SATURATION))
            self.cam_settings_msg.hue = int(self.dwe_camera.get(cv2.CAP_PROP_HUE))
            self.cam_settings_msg.gamma = int(self.dwe_camera.get(cv2.CAP_PROP_GAMMA))
            self.cam_settings_msg.gain = int(self.dwe_camera.get(cv2.CAP_PROP_GAIN))
            self.cam_settings_msg.sharpness = int(self.dwe_camera.get(cv2.CAP_PROP_SHARPNESS))
            self.cam_settings_msg.exposure = int(self.dwe_camera.get(cv2.CAP_PROP_EXPOSURE))
            self.cam_settings_msg.auto_exposure = int(self.dwe_camera.get(cv2.CAP_PROP_AUTO_EXPOSURE)) # 1 (manual) or 3 (auto)
            
            # Get and decode FOURCC
            fourcc_val = self.dwe_camera.get(cv2.CAP_PROP_FOURCC)
            try:
                self.cam_settings_msg.video_format = "".join([chr((int(fourcc_val) >> 8 * i) & 0xFF) for i in range(4)]).strip()
            except:
                self.cam_settings_msg.video_format = "N/A"

            self.cam_settings_pub.publish(self.cam_settings_msg)
        else:
            self.get_logger().warn("Parameter update: Camera not available.")

    def raw_image_capture_callback(self):
        """High-frequency callback. Only does essential work: read, store, publish."""
        if not self.dwe_camera or not self.dwe_camera.isOpened():
            self.get_logger().error("Camera not available for raw image capture.")
            return

        self.latest_header.stamp = self.get_clock().now().to_msg()
        success, frame = self.dwe_camera.read()
        if not success or frame is None:
            self.get_logger().warn("Failed to read frame from camera.")
            return

        self.latest_raw_frame = frame # Store for other callbacks to use

        # Create and publish a compressed image
        jpeg_quality_recompress = self.get_parameter('video.jpeg_quality_recompress').value
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality_recompress]
        result, encimg = cv2.imencode('.jpg', self.latest_raw_frame, encode_param)

        if not result:
            self.get_logger().warn("Could not encode frame to JPEG")
            return
        
        compressed_msg = CompressedImage()
        compressed_msg.header = self.latest_header
        compressed_msg.format = "jpeg"
        compressed_msg.data = encimg.tobytes()

        self.image_pub.publish(compressed_msg)


    def compressed_image_callback(self):
        """Lower-frequency callback for processing and publishing a compressed image."""
        if self.latest_raw_frame is None:
            return

        current_frame = self.latest_raw_frame

        compressed_width = self.get_parameter('compression.width').value
        compressed_height = self.get_parameter('compression.height').value
        jpeg_quality = self.get_parameter('compression.jpeg_quality').value

        try:
            resized_image = cv2.resize(current_frame,
                                   (compressed_width, compressed_height),
                                   interpolation=cv2.INTER_LINEAR)
        except cv2.error as e:
            self.get_logger().error(f"Error during cv2.resize: {e}. Frame shape: {current_frame.shape}")
            return

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        result, encimg = cv2.imencode('.jpg', resized_image, encode_param)

        if not result:
            self.get_logger().warn("Could not encode resized frame to JPEG")
            return
            
        compressed_msg = CompressedImage()
        compressed_msg.header.frame_id = self.latest_header.frame_id
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.format = "jpeg"
        compressed_msg.data = encimg.tobytes()

        self.compressed_image_pub.publish(compressed_msg)

    def destroy_node_custom(self):
        """Custom cleanup method called explicitly in main's finally block."""
        print(f"[{self.get_name()}] INFO: Executing custom node destruction (destroy_node_custom)...")
        self.cleanup_resources()

def main():
    rclpy.init()
    node = None
    try:
        node = ImagePublisher()
        # Use a MultiThreadedExecutor to allow callbacks to run in parallel.
        # This prevents slow I/O in one callback (e.g., parameter updates)
        # from blocking the time-sensitive image capture callback.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            print("Shutdown requested.")

    except RuntimeError as e:
        if node:
             node.get_logger().fatal(f"Node critical failure: {e}")
        else:
            print(f"Critical error during node instantiation: {e}")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Unhandled exception in main: {e}", exc_info=True)
        else:
            print(f"Unhandled exception in main before node init: {e}")
    finally:
        if node:
            node.destroy_node_custom()
            if rclpy.ok():
                 node.destroy_node()
                 print("ROS 2 node destroyed.")
        if rclpy.ok():
            rclpy.try_shutdown()
            print("RCLPY shutdown complete.")

if __name__ == '__main__':
    main()