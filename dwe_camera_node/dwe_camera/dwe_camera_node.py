import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup # Import CallbackGroup
import cv2
import numpy as np
import traceback
import time
import threading
import copy

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from dwe_camera_interfaces.msg import CamParameters
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult, FloatingPointRange

# Import the new OpenCV-based camera class from the (conceptually renamed) file
from .cv2_v4l2 import V4L2Camera

# Node Design Philosophy:
# This node uses two primary mechanisms for interacting with camera settings,
# each serving a distinct purpose:
#
# 1. ROS 2 Parameters (The "Control" Interface):
#    - All camera settings (brightness, exposure, etc.) are declared as ROS 2
#      parameters in the `setup_params` method.
#    - This is the **INPUT** to the node. Users and other systems should change
#      camera settings by modifying these parameters (e.g., via a YAML file,
#      `ros2 param set`, or `rqt_reconfigure`).
#    - The `parameters_callback` function automatically applies these changes
#      to the camera hardware.
#
# 2. The `camera_settings` Topic (The "Status" Interface):
#    - This node publishes the current state of all settings on the
#      `camera_settings` topic, using the `dwe_camera_interfaces/CamParameters`
#      message.
#    - This is the **OUTPUT** from the node. It's a broadcast of the camera's
#      current, active configuration. It is published on startup and whenever a
#      setting is changed.
#    - This allows other nodes to easily monitor the camera's state without
#      needing to query each parameter individually. It's useful for diagnostics,
#      logging, and state-dependent logic in other parts of the system.


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('dwe_camera_node')

        # --- FIX: Create separate callback groups ---
        # This prevents the high-frequency timers from blocking the parameter service server,
        # which runs in the node's default callback group. This is the key to fixing the
        # "asynchronous service call failed" error in rqt_reconfigure.
        self.get_logger().info("Using separate callback groups for camera timers to ensure parameter services remain responsive.")
        self.image_capture_cb_group = MutuallyExclusiveCallbackGroup()
        self.compression_cb_group = MutuallyExclusiveCallbackGroup()
        self.cam_setting_cb_group = MutuallyExclusiveCallbackGroup()

        # The parameter callback will automatically use the default group.

        # --- Threading and State Management for Settings ---
        self.settings_lock = threading.Lock()
        self.pending_settings_publication = False
        self.cam_settings_msg = CamParameters() # Cached message for periodic publishing

        self.v4l2_camera = None
        self.latest_raw_frame = None
        self.latest_header = Header()
        self.raw_image_timer = None
        self.compressed_image_timer = None
        self.cam_settings_timer = None

        # List of camera control parameter names. The V4L2Camera class handles the mapping.
        self.camera_control_params = [
            'brightness', 'contrast', 'saturation', 'hue', 'gamma', 'gain', 'sharpness',
            'auto_exposure', 'exposure_time'
        ]
        
        # V4L2 standard values for auto exposure control
        self.V4L2_EXPOSURE_MANUAL = 1
        self.V4L2_EXPOSURE_AUTO = 3

        try:
            self.setup_params()
            self.setup_ros_elements()
            self.setup_cam()
            self.setup_timers() # Timers now handle initial publication
            self.get_logger().info("DWE Camera Node successfully initialized using OpenCV for direct V4L2 access.")
        except Exception as e:
            self.get_logger().error(f"Error during node initialization: {e}", exc_info=True)
            self.cleanup_resources()
            raise

    def cleanup_resources(self):
        self.get_logger().info("Executing resource cleanup...")
        if self.raw_image_timer: self.raw_image_timer.cancel()
        if self.compressed_image_timer: self.compressed_image_timer.cancel()
        if self.cam_settings_timer: self.cam_settings_timer.cancel()
        if self.v4l2_camera: self.v4l2_camera.release()

    def setup_params(self):
        """
        Declares and configures ROS parameters for the node.
        Only 'camera.*' parameters are dynamically configurable. All others are read-only after startup.
        """
        # === Dynamically-Configurable Camera Control Parameters ===
        brightness_descriptor = ParameterDescriptor(description='Image brightness [-64, 64]', integer_range=[IntegerRange(from_value=-64, to_value=64, step=1)])
        contrast_descriptor = ParameterDescriptor(description='Image contrast [0, 64]', integer_range=[IntegerRange(from_value=0, to_value=64, step=1)])
        saturation_descriptor = ParameterDescriptor(description='Image saturation [0, 128]', integer_range=[IntegerRange(from_value=0, to_value=128, step=1)])
        hue_descriptor = ParameterDescriptor(description='Image hue [-40, 40]', integer_range=[IntegerRange(from_value=-40, to_value=40, step=1)])
        gamma_descriptor = ParameterDescriptor(description='Image gamma [72, 500]', integer_range=[IntegerRange(from_value=72, to_value=500, step=1)])
        gain_descriptor = ParameterDescriptor(description='Image gain [0, 100]', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)])
        sharpness_descriptor = ParameterDescriptor(description='Image sharpness [0, 6]', integer_range=[IntegerRange(from_value=0, to_value=6, step=1)])
        exposure_descriptor = ParameterDescriptor(description='Exposure time [1, 5000]. Used when auto_exposure is False.', integer_range=[IntegerRange(from_value=1, to_value=5000, step=1)])
        auto_exposure_descriptor = ParameterDescriptor(description='Enable/disable auto exposure')

        # === Read-Only ROS-Specific Parameters ===
        frame_id_descriptor = ParameterDescriptor(description='The TF frame ID for the camera images. Read-only after startup.', read_only=True)
        
        # === Read-Only Video Stream Parameters ===
        video_id_descriptor = ParameterDescriptor(description='Camera device ID (e.g., /dev/videoX). Read-only after startup.', read_only=True)
        video_width_descriptor = ParameterDescriptor(description='Capture width in pixels. Read-only after startup.', read_only=True)
        video_height_descriptor = ParameterDescriptor(description='Capture height in pixels. Read-only after startup.', read_only=True)
        video_framerate_descriptor = ParameterDescriptor(description='Requested capture framerate (Hz). Read-only after startup.', read_only=True)
        video_format_descriptor = ParameterDescriptor(description='Capture format (e.g., MJPG). Read-only after startup.', read_only=True)
        
        # === Read-Only Compression Parameters ===
        compression_width_descriptor = ParameterDescriptor(description='Width for the low-bandwidth compressed stream. Read-only after startup.', integer_range=[IntegerRange(from_value=80, to_value=1920, step=1)], read_only=True)
        compression_height_descriptor = ParameterDescriptor(description='Height for the low-bandwidth compressed stream. Read-only after startup.', integer_range=[IntegerRange(from_value=60, to_value=1080, step=1)], read_only=True)
        compression_fps_descriptor = ParameterDescriptor(description='Target FPS for the low-bandwidth compressed stream. Read-only after startup.', floating_point_range=[FloatingPointRange(from_value=0.0, to_value=30.0, step=0.5)], read_only=True)
        jpeg_quality_descriptor = ParameterDescriptor(description='JPEG quality for low-bandwidth stream [0, 100]. Read-only after startup.', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)], read_only=True)

        # Declare all parameters
        self.declare_parameter('ros.frame_id', 'dwe_camera_frame', frame_id_descriptor)

        self.declare_parameter('video.id', 2, video_id_descriptor)
        self.declare_parameter('video.width', 1920, video_width_descriptor)
        self.declare_parameter('video.height', 1080, video_height_descriptor)
        self.declare_parameter('video.framerate', 15, video_framerate_descriptor)
        self.declare_parameter('video.format', 'MJPG', video_format_descriptor)

        self.declare_parameter('compression.width', 320, compression_width_descriptor)
        self.declare_parameter('compression.height', 240, compression_height_descriptor)
        self.declare_parameter('compression.target_fps', 5, compression_fps_descriptor)
        self.declare_parameter('compression.jpeg_quality', 75, jpeg_quality_descriptor)

        self.declare_parameter('camera.brightness', 0, brightness_descriptor)
        self.declare_parameter('camera.contrast', 32, contrast_descriptor)
        self.declare_parameter('camera.saturation', 64, saturation_descriptor)
        self.declare_parameter('camera.hue', 0, hue_descriptor)
        self.declare_parameter('camera.gamma', 100, gamma_descriptor)
        self.declare_parameter('camera.gain', 0, gain_descriptor)
        self.declare_parameter('camera.sharpness', 3, sharpness_descriptor)
        self.declare_parameter('camera.auto_exposure', True, auto_exposure_descriptor)
        self.declare_parameter('camera.exposure_time', 156, exposure_descriptor)
        
        self.add_on_set_parameters_callback(self.parameters_callback)

    def setup_cam(self):
        """Initializes the V4L2Camera with settings from ROS parameters."""
        cam_id = self.get_parameter('video.id').value
        width = self.get_parameter('video.width').value
        height = self.get_parameter('video.height').value
        fps_req = self.get_parameter('video.framerate').value

        # Build a dictionary of initial camera controls from ROS params
        initial_controls = self.get_current_controls_from_params()
        self.get_logger().info(f"Initial controls from parameters: {initial_controls}")
        
        self.v4l2_camera = V4L2Camera(
            device_id=cam_id, width=width, height=height,
            framerate=fps_req, logger=self.get_logger(),
            initial_controls=initial_controls
        )

        self.CAM_FPS = self.v4l2_camera.fps
        self.get_logger().info("V4L2 camera initialized successfully via OpenCV.")

        # Publish initial camera settings and cache them for periodic updates.
        self.update_and_publish_settings()
        self.get_logger().info("Published initial camera settings and cached for periodic updates.")

    def get_current_controls_from_params(self, new_params=[]):
        """
        Constructs a dictionary of camera control values based on current
        node parameters, optionally updated with a list of new parameters.
        """
        current_values = {p: self.get_parameter(f'camera.{p}').value for p in self.camera_control_params}
        
        for p in new_params:
            # The parameter server ensures only 'camera.*' params trigger the callback,
            # so we only need to handle those.
            if p.name.startswith('camera.'):
                control_name = p.name.split('.')[-1]
                if control_name in current_values:
                    current_values[control_name] = p.value
        
        controls_to_set = {}
        for name, value in current_values.items():
            if name == 'auto_exposure':
                controls_to_set['auto_exposure'] = self.V4L2_EXPOSURE_AUTO if value else self.V4L2_EXPOSURE_MANUAL
            elif name == 'exposure_time':
                controls_to_set['exposure_absolute'] = value
            else:
                controls_to_set[name] = value
        
        if current_values['auto_exposure']:
            if 'exposure_absolute' in controls_to_set:
                del controls_to_set['exposure_absolute']

        return controls_to_set

    def setup_ros_elements(self):
        self.latest_header.frame_id = self.get_parameter('ros.frame_id').value
        self.image_pub = self.create_publisher(CompressedImage, "image/compressed", 10)
        self.compressed_image_pub = self.create_publisher(CompressedImage, "image_lowbw/compressed", 10)
        self.cam_settings_pub = self.create_publisher(CamParameters, "camera_settings", 10)

    def setup_timers(self):
        if self.CAM_FPS > 0:
            # --- FIX: Assign timer to its specific callback group ---
            self.raw_image_timer = self.create_timer(
                1.0 / self.CAM_FPS, 
                self.raw_image_capture_callback,
                callback_group=self.image_capture_cb_group)
        else:
            self.get_logger().warn("Camera FPS is 0. Raw image timer will not be started.")

        compressed_fps = float(self.get_parameter('compression.target_fps').value)
        if compressed_fps > 0:
            # --- FIX: Assign timer to its specific callback group ---
            self.compressed_image_timer = self.create_timer(
                1.0 / compressed_fps, 
                self.compressed_image_callback,
                callback_group=self.compression_cb_group)
        
        self.cam_settings_timer = self.create_timer(
            1.0,
            self.camera_settings_callback,
            callback_group=self.cam_setting_cb_group
        )
        
    def parameters_callback(self, params):
        """Applies changed ROS parameters to the camera."""
        self.get_logger().info("Parameter callback triggered!")
        controls_to_set = self.get_current_controls_from_params(params)
        
        self.get_logger().info(f"Applying new camera parameter set: {controls_to_set}")
        if self.v4l2_camera:
            self.v4l2_camera.set_controls(controls_to_set)
        
        # Instead of publishing immediately, set a flag to sync with the next frame.
        with self.settings_lock:
            self.pending_settings_publication = True
        self.get_logger().info("Settings changed. Awaiting next frame to publish synchronized status.")
        
        return SetParametersResult(successful=True)

    def update_and_publish_settings(self, header_to_use=None):
        """
        Queries camera for actual settings, publishes them, and updates the node's
        internal cached message. This is the sole method for querying hardware.
        If a header is provided, it's used for timestamp synchronization.
        """
        if not self.v4l2_camera:
            self.get_logger().warn("Cannot update settings, camera not initialized.")
            return

        try:
            current_controls = self.v4l2_camera.get_all_controls()
        except Exception as e:
            self.get_logger().error(f"Failed to get camera controls: {e}", exc_info=True)
            return

        # Create a new message instance for this publication.
        settings_msg = CamParameters()
        
        if header_to_use:
            settings_msg.header = header_to_use
        else:
            settings_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.latest_header.frame_id)
        
        # Populate the message from hardware
        settings_msg.auto_exposure = current_controls.get('auto_exposure', 0)
        settings_msg.exposure_time = current_controls.get('exposure_time', 0)
        settings_msg.brightness = current_controls.get('brightness', 0)
        settings_msg.contrast = current_controls.get('contrast', 0)
        settings_msg.saturation = current_controls.get('saturation', 0)
        settings_msg.hue = current_controls.get('hue', 0)
        settings_msg.gamma = current_controls.get('gamma', 0)
        settings_msg.gain = current_controls.get('gain', 0)
        settings_msg.sharpness = current_controls.get('sharpness', 0)
        settings_msg.video_format = self.get_parameter('video.format').value
        
        # Publish the freshly queried settings.
        self.cam_settings_pub.publish(settings_msg)

        # Atomically update the shared cache for the periodic timer.
        with self.settings_lock:
            self.cam_settings_msg = settings_msg

    def camera_settings_callback(self):
        """
        Periodically publishes the last known camera settings at 1Hz.
        It does NOT query the camera hardware, making it very lightweight.
        It reuses the cached self.cam_settings_msg.
        """
        with self.settings_lock:
            # If the cache hasn't been populated yet, do nothing.
            if not hasattr(self, 'cam_settings_msg') or not self.cam_settings_msg.video_format:
                return
            # Make a deep copy to ensure thread safety. We will modify the header
            # of the copy before publishing, leaving the cached original untouched.
            msg_to_publish = copy.deepcopy(self.cam_settings_msg)

        # Update the header with a new timestamp for this specific publication.
        msg_to_publish.header.stamp = self.get_clock().now().to_msg()
        self.cam_settings_pub.publish(msg_to_publish)

    def raw_image_capture_callback(self):
        if not (self.v4l2_camera and self.v4l2_camera.is_opened()): return
        
        # Create a header for this frame capture event. It will be used for both
        # the image and, if necessary, the synchronized settings message.
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.latest_header.frame_id)
        
        jpeg_data = self.v4l2_camera.read_jpeg()
        
        if jpeg_data:
            # Check if a settings update is pending and publish it with this frame's header.
            is_pending = False
            with self.settings_lock:
                if self.pending_settings_publication:
                    is_pending = True
                    self.pending_settings_publication = False
            
            if is_pending:
                self.get_logger().info(f"Publishing synchronized settings with timestamp: {header.stamp.sec}.{header.stamp.nanosec}")
                # This queries HW, publishes, and updates the cache.
                self.update_and_publish_settings(header_to_use=header)

            # Publish the image itself.
            compressed_msg = CompressedImage(header=header, format="jpeg", data=jpeg_data)
            self.image_pub.publish(compressed_msg)
            
            # If the low-bandwidth stream is active, decode the frame for it.
            if self.compressed_image_timer and not self.compressed_image_timer.is_canceled():
                try:
                    self.latest_raw_frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
                except cv2.error: 
                    self.latest_raw_frame = None

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
        # A MultiThreadedExecutor allows timer callbacks to run in parallel, which is
        # useful for separating high-rate capture from slower processing.
        # With the callback groups defined in the node, this executor can now
        # run the image capture, compression, and parameter services concurrently
        # without them blocking each other.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        # executor.spin() is a blocking call that processes callbacks until shutdown.
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        # This is the expected path for a clean shutdown (e.g., Ctrl+C).
        # No error message needed as this is a normal exit.
        pass
    except Exception:
        # This will catch any other exception that causes the node to crash,
        # including errors during initialization.
        if node:
            # If the node was created, use its logger.
            node.get_logger().fatal("Unhandled exception in node execution:", exc_info=True)
        else:
            # If node creation failed, print to console.
            print("Unhandled exception during node setup:")
            traceback.print_exc()
    finally:
        # This block ensures that cleanup happens regardless of how the try block exits.
        if node:
            node.get_logger().info("Shutting down node and cleaning up resources.")
            # Custom cleanup must be called before the node is destroyed.
            node.destroy_node_custom()
            if rclpy.ok():
                node.destroy_node()
        
        # Finally, shutdown the rclpy context.
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()