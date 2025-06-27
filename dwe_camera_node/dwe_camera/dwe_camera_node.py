import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
import cv2
import numpy as np
import traceback # Import the traceback module

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from dwe_camera_interfaces.msg import CamParameters
# NEW: Imports for parameter handling
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult, FloatingPointRange

# Import the new GStreamer camera class
from .gstreamer_camera import GStreamerCamera

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('dwe_camera_node')

        self.dwe_camera_cv = None
        self.gstreamer_camera = None
        self.latest_raw_frame = None
        self.latest_header = Header()
        self.raw_image_timer = None
        self.compressed_image_timer = None
        self.settings_publish_timer = None # RENAMED from param_update_timer

        self.param_to_gst_map = {
            'camera.brightness': 'brightness', 'camera.contrast': 'contrast',
            'camera.saturation': 'saturation', 'camera.hue': 'hue',
            'camera.gamma': 'gamma', 'camera.gain': 'gain', 'camera.sharpness': 'sharpness',
            'camera.auto_exposure': 'auto_exposure',
            'camera.exposure_time': 'exposure_absolute'
        }
        self.V4L2_EXPOSURE_MANUAL = 1
        self.V4L2_EXPOSURE_AUTO = 3

        try:
            self.setup_params()
            self.setup_cam()
            self.setup_ros_elements()
            self.publish_current_settings() # Publish initial state
            self.setup_timers()
            self.get_logger().info("DWE Camera Node successfully initialized with dynamic controls.")
        except Exception as e:
            self.get_logger().error(f"Error during node initialization: {e}", exc_info=True)
            self.cleanup_resources()
            raise

    def cleanup_resources(self):
        self.get_logger().info(f"Executing resource cleanup...")
        if self.raw_image_timer: self.raw_image_timer.cancel()
        if self.compressed_image_timer: self.compressed_image_timer.cancel()
        if self.settings_publish_timer: self.settings_publish_timer.cancel() # RENAMED
        if self.gstreamer_camera: self.gstreamer_camera.release()
        if self.dwe_camera_cv and self.dwe_camera_cv.isOpened(): self.dwe_camera_cv.release()

    def setup_params(self):
        """
        Declares and configures ROS parameters for the node.
        This now includes ParameterDescriptors to enable rich dynamic reconfiguration.
        """
        # THE FIX IS HERE: Provide explicit descriptors for ALL parameters.
        # This prevents type ambiguity that can confuse GUI tools like rqt_reconfigure.
        
        # Descriptors for camera controls
        brightness_descriptor = ParameterDescriptor(description='Image brightness [-64, 64]', integer_range=[IntegerRange(from_value=-64, to_value=64, step=1)])
        contrast_descriptor = ParameterDescriptor(description='Image contrast [0, 64]', integer_range=[IntegerRange(from_value=0, to_value=64, step=1)])
        saturation_descriptor = ParameterDescriptor(description='Image saturation [0, 128]', integer_range=[IntegerRange(from_value=0, to_value=128, step=1)])
        hue_descriptor = ParameterDescriptor(description='Image hue [-40, 40]', integer_range=[IntegerRange(from_value=-40, to_value=40, step=1)])
        gamma_descriptor = ParameterDescriptor(description='Image gamma [72, 500]', integer_range=[IntegerRange(from_value=72, to_value=500, step=1)])
        gain_descriptor = ParameterDescriptor(description='Image gain [0, 100]', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)])
        sharpness_descriptor = ParameterDescriptor(description='Image sharpness [0, 6]', integer_range=[IntegerRange(from_value=0, to_value=6, step=1)])
        exposure_descriptor = ParameterDescriptor(description='Exposure time [1, 5000]. Used when auto_exposure is False.', integer_range=[IntegerRange(from_value=1, to_value=5000, step=1)])
        auto_exposure_descriptor = ParameterDescriptor(description='Enable/disable auto exposure')

        # Descriptors for ROS-specific and other parameters
        frame_id_descriptor = ParameterDescriptor(description='The TF frame ID for the camera images.')
        settings_publish_rate_descriptor = ParameterDescriptor(description='Rate (Hz) to publish the camera_settings topic.', floating_point_range=[FloatingPointRange(from_value=0.1, to_value=30.0, step=0.1)])
        video_id_descriptor = ParameterDescriptor(description='Camera device ID (e.g., /dev/videoX). Read-only after startup.', read_only=True)
        video_width_descriptor = ParameterDescriptor(description='Capture width in pixels. Read-only after startup.', read_only=True)
        video_height_descriptor = ParameterDescriptor(description='Capture height in pixels. Read-only after startup.', read_only=True)
        video_framerate_descriptor = ParameterDescriptor(description='Requested capture framerate (Hz). Read-only after startup.', read_only=True)
        video_format_descriptor = ParameterDescriptor(description='Capture format (e.g., MJPG). Read-only after startup.', read_only=True)
        compression_width_descriptor = ParameterDescriptor(description='Width for the low-bandwidth compressed stream.', integer_range=[IntegerRange(from_value=80, to_value=1920, step=1)])
        compression_height_descriptor = ParameterDescriptor(description='Height for the low-bandwidth compressed stream.', integer_range=[IntegerRange(from_value=60, to_value=1080, step=1)])
        compression_fps_descriptor = ParameterDescriptor(description='Target FPS for the low-bandwidth compressed stream.', floating_point_range=[FloatingPointRange(from_value=0.0, to_value=30.0, step=0.5)])
        jpeg_quality_descriptor = ParameterDescriptor(description='JPEG quality for low-bandwidth stream [0, 100]', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)])

        # ROS-specific parameters
        self.declare_parameter('ros.frame_id', 'dwe_camera_frame', frame_id_descriptor)
        self.declare_parameter('ros.settings_publish_rate', 1.0, settings_publish_rate_descriptor)

        # Video stream parameters (read-only after startup)
        self.declare_parameter('video.id', 2, video_id_descriptor)
        self.declare_parameter('video.width', 1920, video_width_descriptor)
        self.declare_parameter('video.height', 1080, video_height_descriptor)
        self.declare_parameter('video.framerate', 15, video_framerate_descriptor)
        self.declare_parameter('video.format', 'MJPG', video_format_descriptor)

        # Compression parameters (dynamically configurable)
        self.declare_parameter('compression.width', 320, compression_width_descriptor)
        self.declare_parameter('compression.height', 240, compression_height_descriptor)
        self.declare_parameter('compression.target_fps', 5.0, compression_fps_descriptor)
        self.declare_parameter('compression.jpeg_quality', 75, jpeg_quality_descriptor)

        # Camera control parameters (dynamically configurable)
        self.declare_parameter('camera.brightness', 0, brightness_descriptor)
        self.declare_parameter('camera.contrast', 32, contrast_descriptor)
        self.declare_parameter('camera.saturation', 64, saturation_descriptor)
        self.declare_parameter('camera.hue', 0, hue_descriptor)
        self.declare_parameter('camera.gamma', 100, gamma_descriptor)
        self.declare_parameter('camera.gain', 0, gain_descriptor)
        self.declare_parameter('camera.sharpness', 3, sharpness_descriptor)
        self.declare_parameter('camera.auto_exposure', True, auto_exposure_descriptor)
        self.declare_parameter('camera.exposure_time', 156, exposure_descriptor)
        
        # Register the callback for parameter changes
        self.add_on_set_parameters_callback(self.parameters_callback)

    def setup_cam(self):
        cam_id = self.get_parameter('video.id').value
        width = self.get_parameter('video.width').value
        height = self.get_parameter('video.height').value
        fps_req = self.get_parameter('video.framerate').value
        video_format = self.get_parameter('video.format').value

        self.get_logger().info(f"Temporarily opening OpenCV camera {cam_id} for initial setup.")
        self.dwe_camera_cv = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not self.dwe_camera_cv or not self.dwe_camera_cv.isOpened():
            raise RuntimeError(f"Failed to open video device {cam_id} with OpenCV for setup.")
        
        if len(video_format) == 4:
            self.dwe_camera_cv.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*video_format))
        self.dwe_camera_cv.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.dwe_camera_cv.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.dwe_camera_cv.set(cv2.CAP_PROP_FPS, fps_req)
        
        self.apply_initial_parameters_opencv()

        self.CAM_FPS = self.dwe_camera_cv.get(cv2.CAP_PROP_FPS)
        if self.CAM_FPS <= 0:
            self.CAM_FPS = float(fps_req)

        self.dwe_camera_cv.release()
        self.dwe_camera_cv = None

        self.gstreamer_camera = GStreamerCamera(
            device_id=cam_id, width=width, height=height,
            framerate=int(self.CAM_FPS), logger=self.get_logger()
        )
        if not self.gstreamer_camera.start():
            raise RuntimeError("GStreamer camera failed to start.")
        
        self.get_logger().info("GStreamer camera initialized successfully.")

    def apply_initial_parameters_opencv(self):
        """Sets initial camera parameters using the temporary OpenCV handle."""
        if not self.dwe_camera_cv or not self.dwe_camera_cv.isOpened(): return
        auto_exposure = self.get_parameter('camera.auto_exposure').value
        exposure_time = self.get_parameter('camera.exposure_time').value
        target_auto_exposure_cv = self.V4L2_EXPOSURE_AUTO if auto_exposure else self.V4L2_EXPOSURE_MANUAL
        self.dwe_camera_cv.set(cv2.CAP_PROP_AUTO_EXPOSURE, target_auto_exposure_cv)
        if not auto_exposure:
            self.dwe_camera_cv.set(cv2.CAP_PROP_EXPOSURE, exposure_time)
        params_to_set = { 'brightness': cv2.CAP_PROP_BRIGHTNESS, 'contrast': cv2.CAP_PROP_CONTRAST,
            'saturation': cv2.CAP_PROP_SATURATION, 'hue': cv2.CAP_PROP_HUE, 'gamma': cv2.CAP_PROP_GAMMA,
            'gain': cv2.CAP_PROP_GAIN, 'sharpness': cv2.CAP_PROP_SHARPNESS }
        for name, cv_prop in params_to_set.items():
            self.dwe_camera_cv.set(cv_prop, self.get_parameter(f'camera.{name}').value)
        self.get_logger().info("Initial camera parameters applied via OpenCV.")

    def setup_ros_elements(self):
        self.latest_header.frame_id = self.get_parameter('ros.frame_id').value
        self.image_pub = self.create_publisher(CompressedImage, "image/compressed", 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, "image_lowbw/compressed", 10)
        self.cam_settings_pub = self.create_publisher(CamParameters, "camera_settings", 10)
        self.cam_settings_msg = CamParameters()

    def setup_timers(self):
        self.raw_image_timer = self.create_timer(1.0 / self.CAM_FPS, self.raw_image_capture_callback)
        compressed_fps = self.get_parameter('compression.target_fps').value
        if compressed_fps > 0:
            self.compressed_image_timer = self.create_timer(1.0 / compressed_fps, self.compressed_image_callback)
        
        settings_rate = self.get_parameter('ros.settings_publish_rate').value
        if settings_rate > 0:
            self.settings_publish_timer = self.create_timer(1.0 / settings_rate, self.publish_current_settings)

    def parameters_callback(self, params):
        """
        This callback is triggered by the ROS framework whenever parameters are changed.
        It rebuilds the entire camera control structure and applies it atomically.
        """
        # Create a dictionary of the new proposed values from this request
        new_values = {p.name: p.value for p in params}

        # Get a dictionary of all current camera control parameter values
        current_params = {name: self.get_parameter(name).value for name in self.param_to_gst_map.keys()}
        
        # Update the current values with the new ones that are being set
        current_params.update(new_values)

        controls_to_set = {}
        
        # Determine auto exposure setting (using the new value if it was changed)
        auto_exposure_on = current_params['camera.auto_exposure']
        gst_auto_prop = self.param_to_gst_map['camera.auto_exposure']
        controls_to_set[gst_auto_prop] = self.V4L2_EXPOSURE_AUTO if auto_exposure_on else self.V4L2_EXPOSURE_MANUAL

        # Build the GStreamer control structure
        for ros_param, gst_prop in self.param_to_gst_map.items():
            if ros_param == 'camera.auto_exposure':
                continue
            # Only apply exposure_time if auto exposure is disabled
            if ros_param == 'camera.exposure_time' and auto_exposure_on:
                continue
            
            controls_to_set[gst_prop] = current_params[ros_param]

        self.get_logger().info(f"Applying new parameter set: {controls_to_set}")
        self.gstreamer_camera.set_all_controls(controls_to_set)
        
        # The parameter values within the node are updated automatically by the framework
        # before this callback runs. We can now publish the new state.
        self.publish_current_settings()

        return SetParametersResult(successful=True)

    def publish_current_settings(self):
        """Publishes the camera's state based on current ROS parameters."""
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.latest_header.frame_id)
        self.cam_settings_msg.header = header
        
        # Populate message from ROS parameters, handling type conversions
        auto_exposure_on = self.get_parameter('camera.auto_exposure').value
        self.cam_settings_msg.auto_exposure = self.V4L2_EXPOSURE_AUTO if auto_exposure_on else self.V4L2_EXPOSURE_MANUAL
        
        self.cam_settings_msg.brightness = int(self.get_parameter('camera.brightness').value)
        self.cam_settings_msg.contrast = int(self.get_parameter('camera.contrast').value)
        self.cam_settings_msg.saturation = int(self.get_parameter('camera.saturation').value)
        self.cam_settings_msg.hue = int(self.get_parameter('camera.hue').value)
        self.cam_settings_msg.gamma = int(self.get_parameter('camera.gamma').value)
        self.cam_settings_msg.gain = int(self.get_parameter('camera.gain').value)
        self.cam_settings_msg.sharpness = int(self.get_parameter('camera.sharpness').value)
        self.cam_settings_msg.exposure = int(self.get_parameter('camera.exposure_time').value)

        self.cam_settings_msg.video_format = self.get_parameter('video.format').value
        self.cam_settings_pub.publish(self.cam_settings_msg)

    def raw_image_capture_callback(self):
        if not (self.gstreamer_camera and self.gstreamer_camera.is_opened()): return
        self.latest_header.stamp = self.get_clock().now().to_msg()
        jpeg_data = self.gstreamer_camera.read_jpeg()
        if jpeg_data:
            compressed_msg = CompressedImage(header=self.latest_header, format="jpeg", data=jpeg_data)
            self.image_pub.publish(compressed_msg)
            if self.compressed_image_timer and not self.compressed_image_timer.is_canceled():
                try:
                    self.latest_raw_frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
                except cv2.error: self.latest_raw_frame = None

    def compressed_image_callback(self):
        if self.latest_raw_frame is None: return
        w = self.get_parameter('compression.width').value
        h = self.get_parameter('compression.height').value
        q = self.get_parameter('compression.jpeg_quality').value
        try:
            resized_image = cv2.resize(self.latest_raw_frame, (w, h), cv2.INTER_LINEAR)
            result, encimg = cv2.imencode('.jpg', resized_image, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if result:
                header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.latest_header.frame_id)
                msg = CompressedImage(header=header, format="jpeg", data=encimg.tobytes())
                self.compressed_image_pub.publish(msg)
        except cv2.error as e: self.get_logger().error(f"Error in low-bw compression: {e}")

    def destroy_node_custom(self):
        self.get_logger().info(f"Executing custom node destruction...")
        self.cleanup_resources()

def main():
    rclpy.init()
    node = None
    try:
        node = ImagePublisher()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (RuntimeError, ExternalShutdownException, KeyboardInterrupt) as e:
        if node and isinstance(e, RuntimeError):
            node.get_logger().fatal(f"Node critical failure: {e}")
        elif not node:
             print(f"Critical error during node instantiation: {e}")
    except Exception as e:
        if node:
            node.get_logger().exception(f"Unhandled exception in main: {e}")
        else:
            print(f"Unhandled exception in main before node init: {e}")
            traceback.print_exc()
    finally:
        if node:
            node.destroy_node_custom()
            if rclpy.ok(): node.destroy_node()
        if rclpy.ok(): rclpy.try_shutdown()

if __name__ == '__main__':
    main()