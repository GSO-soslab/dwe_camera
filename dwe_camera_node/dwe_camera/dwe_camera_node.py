import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from rcl_interfaces.msg import ParameterDescriptor # Not used

from rclpy.time import Duration
import cv2
from cv_bridge import CvBridge

from std_msgs.msg import Header # Float32 not used directly here
from sensor_msgs.msg import Image, CompressedImage
from dwe_camera_interfaces.msg import CamParameters

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('dwe_camera_node')

        self.bridge = CvBridge()
        self.dwe_camera = None  # Initialize camera object to None

        try:
            self.setup_params()
            self.setup_cam()  # This can raise RuntimeError
            self.setup_compression()
            self.setup_ros()
            self.get_logger().info("DWE Camera Node successfully initialized.")
        except Exception as e:
            self.get_logger().error(f"Error during node initialization: {e}")
            # Clean up any resources partially initialized
            if self.dwe_camera and self.dwe_camera.isOpened():
                self.dwe_camera.release()
                self.get_logger().info("Camera released due to initialization error.")
            raise  # Re-raise the exception to stop node creation / inform main

    def setup_params(self):
        # ros related
        self.declare_parameter('ros.frame_id', '/dwe_camera')
        # sensor related
        self.declare_parameter('video.id', 0)
        self.declare_parameter('video.width', 1600)
        self.declare_parameter('video.height', 1200)
        self.declare_parameter('video.framerate', 15)
        # values: 1 (manual), 3 (auto from CamParameters.msg comment)
        self.declare_parameter('video.auto_exposure', 3) 
        self.declare_parameter('video.exposure_time', 100)
        # compression related
        self.declare_parameter('compression.width', 320)
        self.declare_parameter('compression.height', 240)
        self.declare_parameter('compression.target_fps', 5)
        self.declare_parameter('compression.jpeg_quality', 75)

    def setup_cam(self):
        # Camera Index
        CAM_IDX = self.get_parameter('video.id').value
        # resolution
        WIDTH = self.get_parameter('video.width').value
        HEIGHT = self.get_parameter('video.height').value
        # frame rate
        CAM_FPS_PARAM = self.get_parameter('video.framerate').value
        # auto exposure: 1 for manual, 3 for auto (based on CamParameters.msg and V4L2)
        AUTO_EXPOSURE_PARAM = self.get_parameter('video.auto_exposure').value
        # exposure time
        EXPO_TIME_PARAM = self.get_parameter('video.exposure_time').value
        # pixel format
        MJPG = cv2.VideoWriter_fourcc(*'MJPG')

        # -- DEVICE SETUP --
        self.get_logger().info(f"Attempting to open camera {CAM_IDX} with V4L2 backend.")
        self.dwe_camera = cv2.VideoCapture(CAM_IDX, cv2.CAP_V4L2)

        if not self.dwe_camera or not self.dwe_camera.isOpened():
            self.get_logger().error(f"Error - could not open video device {CAM_IDX}.")
            raise RuntimeError(f"Failed to open video device {CAM_IDX}")

        self.get_logger().info(f"Setting pixel format to MJPG.")
        self.dwe_camera.set(cv2.CAP_PROP_FOURCC, MJPG)
        self.get_logger().info(f"Setting frame width to {WIDTH}.")
        self.dwe_camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.get_logger().info(f"Setting frame height to {HEIGHT}.")
        self.dwe_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.get_logger().info(f"Setting FPS to {CAM_FPS_PARAM}.")
        self.dwe_camera.set(cv2.CAP_PROP_FPS, CAM_FPS_PARAM)

        # Handle auto exposure
        # V4L2: 1 = Manual Mode, 3 = Aperture Priority Mode (effectively auto)
        if AUTO_EXPOSURE_PARAM == 1:  # Manual exposure
            self.get_logger().info(f"Setting V4L2 manual exposure mode (1), exposure time: {EXPO_TIME_PARAM}")
            self.dwe_camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
            self.dwe_camera.set(cv2.CAP_PROP_EXPOSURE, EXPO_TIME_PARAM)
        elif AUTO_EXPOSURE_PARAM == 3:  # Auto exposure
            self.get_logger().info("Setting V4L2 auto exposure mode (3 - Aperture Priority)")
            self.dwe_camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
        else:
            self.get_logger().warn(
                f"Invalid 'video.auto_exposure' parameter value: {AUTO_EXPOSURE_PARAM}. "
                "Expected 1 (manual) or 3 (auto). Defaulting to auto (3)."
            )
            self.dwe_camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # Default to auto

        # Read back actual values to confirm
        self.CAM_FPS = self.dwe_camera.get(cv2.CAP_PROP_FPS)
        self.expo_time = self.dwe_camera.get(cv2.CAP_PROP_EXPOSURE)
        actual_auto_expo = self.dwe_camera.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        actual_width = self.dwe_camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.dwe_camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.get_logger().info(f"Actual camera settings: Resolution: {actual_width}x{actual_height}, FPS: {self.CAM_FPS}, Auto Exposure Mode: {actual_auto_expo}, Exposure Time: {self.expo_time}")

        if self.CAM_FPS <= 0:
            self.get_logger().error(f"Camera reported invalid FPS: {self.CAM_FPS}. Cannot create timer.")
            # self.dwe_camera.release() # Will be handled by destroy_node_custom or __init__ except block
            raise RuntimeError(f"Camera reported invalid FPS: {self.CAM_FPS}")

    def setup_compression(self):
        self.compressed_width = self.get_parameter('compression.width').value
        self.compressed_height = self.get_parameter('compression.height').value
        self.compressed_quality = self.get_parameter('compression.jpeg_quality').value

        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.compressed_quality]

        self.last_pub_time = self.get_clock().now()
        self.compressed_CAM_FPS = self.get_parameter('compression.target_fps').value
        if self.compressed_CAM_FPS <= 0:
            self.get_logger().warn(f"Compression target_fps is {self.compressed_CAM_FPS}, disabling compressed image publishing.")
            self.compressed_image_timer_period_duration = None # Disable
        else:
            self.compressed_image_timer_period_float = 1.0 / self.compressed_CAM_FPS
            self.compressed_image_timer_period_duration = Duration(
                nanoseconds=int(self.compressed_image_timer_period_float * 1e9)
            )

    def setup_ros(self):
        # publisher
        self.image_pub = self.create_publisher(Image, "image", 10)
        self.image_msg = Image()

        self.compressed_image_pub = self.create_publisher(CompressedImage, "image_lowbw/compressed", 10)
        self.compressed_image_msg = CompressedImage()
        
        self.cam_settings_pub = self.create_publisher(CamParameters, "camera_settings", 10)
        self.cam_settings_msg = CamParameters()

        if self.CAM_FPS <= 0: # Should have been caught in setup_cam, but defensive check
            self.get_logger().error("Cannot create main image timer due to invalid CAM_FPS.")
            raise RuntimeError("Cannot create main image timer due to invalid CAM_FPS.")
            
        image_timer_period = 1.0 / self.CAM_FPS        
        self.timer = self.create_timer(image_timer_period, self.timer_callback)

    def compress_image(self):
        if self.compressed_image_timer_period_duration is None: # Check if disabled
            return None

        time_now = self.get_clock().now()
        if (time_now - self.last_pub_time) < self.compressed_image_timer_period_duration:
            return None
        self.last_pub_time = time_now

        # --- Resizing ---
        resized_image = cv2.resize(self.frame, (self.compressed_width, self.compressed_height), interpolation=cv2.INTER_LINEAR)
        # --- Compression ---
        result, encoded_jpeg = cv2.imencode('.jpg', resized_image, self.encode_param)
        
        if not result:
            self.get_logger().warn("JPEG encoding failed.")
            return None
        return encoded_jpeg

    def timer_callback(self):
        frame_id = self.get_parameter('ros.frame_id').value

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        
        if not self.dwe_camera or not self.dwe_camera.isOpened():
            self.get_logger().error("Camera is not open in timer_callback. Attempting to stop node.")
            # This is a severe error, node should probably stop.
            # For now, just log and skip. Consider a more robust shutdown.
            rclpy.shutdown() # Initiate shutdown
            return
        read_start_time = self.get_clock().now()
        self.success, self.frame = self.dwe_camera.read()
        read_end_time = self.get_clock().now()
        read_duration_ms = (read_end_time - read_start_time).nanoseconds / 1e6
        self.get_logger().info(f"cv2 read image time: {read_duration_ms}")
        if not self.success or self.frame is None:
            self.get_logger().warn("Failed to retrieve frame from camera or frame is None. Skipping frame.")
            return

        # image message
        try:
            self.image_msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion error: {e}")
            return
            
        self.image_msg.header = header
        self.image_pub.publish(self.image_msg)

        # compressed image message
        encoded_jpeg_data = self.compress_image()
        if encoded_jpeg_data is not None:
            self.compressed_image_msg.header = header
            self.compressed_image_msg.format = "jpeg"
            self.compressed_image_msg.data = encoded_jpeg_data.tobytes()
            self.compressed_image_pub.publish(self.compressed_image_msg)
        
        # camera setting message
        self.cam_settings_msg.header = header
        self.cam_settings_msg.brightness = int(self.dwe_camera.get(cv2.CAP_PROP_BRIGHTNESS))
        self.cam_settings_msg.contrast = int(self.dwe_camera.get(cv2.CAP_PROP_CONTRAST))
        self.cam_settings_msg.saturation = int(self.dwe_camera.get(cv2.CAP_PROP_SATURATION))
        self.cam_settings_msg.hue = int(self.dwe_camera.get(cv2.CAP_PROP_HUE))
        self.cam_settings_msg.gamma = int(self.dwe_camera.get(cv2.CAP_PROP_GAMMA))
        self.cam_settings_msg.gain = int(self.dwe_camera.get(cv2.CAP_PROP_GAIN))
        self.cam_settings_msg.sharpness = int(self.dwe_camera.get(cv2.CAP_PROP_SHARPNESS))
        self.cam_settings_msg.exposure = int(self.dwe_camera.get(cv2.CAP_PROP_EXPOSURE))
        self.cam_settings_msg.auto_exposure = int(self.dwe_camera.get(cv2.CAP_PROP_AUTO_EXPOSURE))
        self.cam_settings_pub.publish(self.cam_settings_msg)

    def destroy_node_custom(self):
        """Custom cleanup method."""
        self.get_logger().info("Executing custom node destruction...")
        if self.timer:
            self.timer.cancel()
            self.get_logger().info("Image timer cancelled.")
        if self.dwe_camera and self.dwe_camera.isOpened():
            self.dwe_camera.release()
            self.get_logger().info("Camera released.")


def main():
    rclpy.init()
    node = None
    try:
        node = ImagePublisher()
        rclpy.spin(node)
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
        if rclpy.ok():
            rclpy.try_shutdown()
            print("RCLPY shutdown complete.")

if __name__ == '__main__':
    main()
