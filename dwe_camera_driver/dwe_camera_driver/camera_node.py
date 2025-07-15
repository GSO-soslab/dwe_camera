import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import cv2
import numpy as np
import traceback
import threading
import copy
import time # Added for sleep calls
import subprocess # For auto-detection
import re # For auto-detection

from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage
from dwe_camera_interfaces.msg import CameraSettings
from rcl_interfaces.msg import SetParametersResult

# Core hardware interface
from .camera_device import CameraDevice
# Parameter setup module
from .parameter_setup import declare_camera_parameters, get_camera_control_descriptors

class CameraNode(Node):
    """
    The core camera node. It handles the direct interface with the camera hardware,
    publishes the main compressed image stream, and manages camera settings.
    It conditionally loads and manages auxiliary processors for features like
    low-bandwidth streams, raw image publishing, and AprilTag detection based on
    the provided ROS parameters.
    The capture logic runs in a dedicated thread to ensure self-pacing, which is
    critical for performance on resource-constrained devices like Raspberry Pi.
    """
    def __init__(self):
        super().__init__('camera_node')

        # Use separate callback groups for timers and services to ensure responsiveness.
        self.cam_setting_cb_group = MutuallyExclusiveCallbackGroup()
        self.compression_cb_group = MutuallyExclusiveCallbackGroup()
        self.raw_image_cb_group = MutuallyExclusiveCallbackGroup()
        self.apriltag_cb_group = MutuallyExclusiveCallbackGroup()
        self.calibrated_image_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.get_logger().info("Initializing DWE Camera Node...")

        # --- State and Resource Management ---
        self.settings_lock = threading.Lock()
        self.pending_settings_publication = False
        self.cam_settings_msg = CameraSettings()
        self.supported_controls = {}
        
        self.camera_device = None
        self.cam_settings_timer = None
        
        # Dedicated thread for image capture to ensure self-pacing
        self.capture_thread = None
        self.shutdown_event = threading.Event()

        # --- Auxiliary Processors (initialized to None) ---
        self.low_bw_compressor = None
        self.apriltag_processor = None
        self.raw_image_publisher = None
        self.calibrated_image_publisher = None
        
        # List of camera control parameter names for easier management.
        self.camera_control_params = [
            'brightness', 'contrast', 'saturation', 'hue', 'gamma', 'gain', 'sharpness',
            'auto_exposure', 'exposure_time', 'white_balance_automatic', 'white_balance_temperature',
            'power_line_frequency', 'backlight_compensation'
        ]
        
        # V4L2 standard values for auto exposure control
        self.V4L2_EXPOSURE_MANUAL = 1
        self.V4L2_EXPOSURE_AUTO = 3

        try:
            # Initialization sequence
            self.setup_parameters()
            self.setup_camera_device()
            self.setup_core_ros_publishers()
            self.setup_auxiliary_processors()
            self.publish_initial_settings()
            self.set_unsupported_params_to_readonly()
            
            # Register parameter callback AFTER all parameters are finalized
            self.add_on_set_parameters_callback(self.parameters_callback)
            
            self.setup_timers_and_thread() # New combined setup
            self.get_logger().info("Camera node successfully initialized.")
            
        except Exception as e:
            self.get_logger().fatal(f"Fatal error during node initialization: {e}", exc_info=True)
            self.cleanup_resources()
            # Re-raise the exception to make the launch system aware of the failure
            raise

    def _find_camera_id_by_name(self, name_substring):
        """
        Finds a V4L2 camera device index by searching for a substring in its name.
        Uses the `v4l2-ctl --list-devices` command.
        """
        if not name_substring:
            return None
        
        self.get_logger().info(f"Searching for camera with name containing: '{name_substring}'")
        try:
            # Execute the command
            output = subprocess.check_output(['v4l2-ctl', '--list-devices'], text=True, stderr=subprocess.STDOUT)
            
            # The output groups a device name with its /dev/videoX paths.
            # An empty line separates device entries.
            devices = output.strip().split('\n\n')
            
            for device_info in devices:
                # Check if the desired name is in this device block
                if name_substring.lower() in device_info.lower():
                    # Find the first /dev/videoX path associated with it
                    match = re.search(r'/dev/video(\d+)', device_info)
                    if match:
                        device_id = int(match.group(1))
                        self.get_logger().info(f"Found camera '{name_substring}' at /dev/video{device_id}")
                        return device_id
            
            self.get_logger().warn(f"Could not find a camera with name containing '{name_substring}'.")
            return None

        except FileNotFoundError:
            self.get_logger().error("'v4l2-ctl' command not found. Please install 'v4l-utils'. Cannot search for camera by name.")
            return None
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error executing 'v4l2-ctl --list-devices': {e.output}")
            return None
        
    def setup_parameters(self):
        """Delegates parameter declaration to the parameter_setup module."""
        declare_camera_parameters(self)
        
    def setup_camera_device(self):
        """Initializes the CameraDevice with settings from ROS parameters."""
        self.get_logger().info("Setting up camera device...")
        # --- Camera Device Discovery ---
        product_name = self.get_parameter('video.product_name').value
        cam_id = None
        if product_name:
            cam_id = self._find_camera_id_by_name(product_name)
        if cam_id is None:
            cam_id = self.get_parameter('video.id').value
            self.get_logger().info(f"Using camera ID from 'video.id' parameter: {cam_id}")
        width = self.get_parameter('video.width').value
        height = self.get_parameter('video.height').value
        fps_req = self.get_parameter('video.framerate').value

        initial_controls = self.get_current_controls_from_params()
        
        self.camera_device = CameraDevice(
            device_id=cam_id, width=width, height=height,
            framerate=fps_req, logger=self.get_logger(),
            initial_controls=initial_controls
        )
        self.CAM_FPS = self.camera_device.fps
        
        self.supported_controls = self.camera_device.get_supported_controls()
        self.get_logger().info(f"Hardware control support map: {self.supported_controls}")

    def setup_core_ros_publishers(self):
        """Initializes the publishers that are essential to the core node's function."""
        self.get_logger().info("Setting up core ROS publishers...")
        self.image_pub = self.create_publisher(CompressedImage, "image/compressed", 10)
        self.cam_settings_pub = self.create_publisher(CameraSettings, "camera_settings", 10)

    def setup_auxiliary_processors(self):
        """Conditionally initializes auxiliary processors based on configuration."""
        self.get_logger().info("Checking for auxiliary processors to enable...")
        
        # 1. Low-Bandwidth Compressor
        if self.get_parameter('compression.target_fps').value > 0:
            self.get_logger().info("Enabling low-bandwidth compressor module.")
            from .stream_processors import LowBandwidthCompressor
            self.low_bw_compressor = LowBandwidthCompressor(self, self.compression_cb_group)
        else:
            self.get_logger().info("Low-bandwidth stream is disabled (target_fps is 0).")

        # 2. Raw Image Publisher
        if self.get_parameter('aux_process.img_raw').value:
            self.get_logger().info("Enabling raw image publisher module.")
            raw_img_mono = self.get_parameter('aux_process.img_raw_mono').value
            raw_img_fps = self.get_parameter('aux_process.img_raw_framerate').value
            from .stream_processors import RawImagePublisher
            self.raw_image_publisher = RawImagePublisher(self, raw_img_mono, raw_img_fps, self.raw_image_cb_group)
        else:
            self.get_logger().info("Raw image stream is disabled (img_raw is false).")
            
        # 3. AprilTag Processor
        if self.get_parameter('apriltag.enable').value:
            self.get_logger().info("Enabling AprilTag processor module.")
            from .apriltag_processor import AprilTagProcessor
            self.apriltag_processor = AprilTagProcessor(self, self.apriltag_cb_group)
        else:
            self.get_logger().info("AprilTag detection is disabled (apriltag.enable is false).")
        
        # 4. Calibrated Image Publisher
        if self.get_parameter('aux_process.img_calibrated').value:
            self.get_logger().info("Enabling calibrated image publisher module.")
            from .stream_processors import CalibratedImagePublisher
            calibrated_publish_rate = self.get_parameter('aux_process.img_calibrated_framerate').value
            self.calibrated_image_publisher = CalibratedImagePublisher(self, calibrated_publish_rate, self.calibrated_image_cb_group)
        else:
            self.get_logger().info("Calibrated image stream is disabled (img_calibrated is false).")

    def publish_initial_settings(self):
        """Publishes the initial camera settings after querying the hardware."""
        self.get_logger().info("Publishing initial camera settings...")
        self.update_and_publish_settings()

    def set_unsupported_params_to_readonly(self):
        """Iterates through hardware controls and marks ROS parameters for unsupported controls as read-only."""
        self.get_logger().info("Setting unsupported parameters to read-only...")
        
        # Get the authoritative descriptor map from the parameter setup module
        descriptor_map = get_camera_control_descriptors()

        for control_name in self.camera_control_params:
            if not self.supported_controls.get(control_name, False):
                param_full_name = f'camera.{control_name}'
                self.get_logger().warn(f"Control '{control_name}' not supported. Setting parameter '{param_full_name}' to read-only.")
                try:
                    current_value = self.get_parameter(param_full_name).value
                    # Use the descriptor from the centralized map
                    new_descriptor = descriptor_map[control_name]
                    new_descriptor.read_only = True
                    self.undeclare_parameter(param_full_name)
                    self.declare_parameter(param_full_name, current_value, new_descriptor)
                except Exception as e:
                    self.get_logger().error(f"Failed to set unsupported parameter '{param_full_name}' to read-only: {e}")

    def setup_timers_and_thread(self):
        """Sets up the capture thread and the periodic settings timer."""
        self.get_logger().info("Setting up timers and capture thread...")

        # Timer for periodically publishing camera settings
        self.cam_settings_timer = self.create_timer(1.0, self.periodic_settings_callback, callback_group=self.cam_setting_cb_group)

        # Start the main capture thread for self-paced image processing
        if self.camera_device and self.camera_device.is_opened():
            self.get_logger().info("Starting main capture thread.")
            self.shutdown_event.clear()
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
        else:
            self.get_logger().error("Camera device not available. Cannot start capture thread.")

    def parameters_callback(self, params):
        """Applies changed ROS parameters to the camera hardware."""
        self.get_logger().info("Parameter callback triggered for parameter change.")
        controls_to_set = self.get_current_controls_from_params(params)
        
        self.get_logger().info(f"Applying new camera control set: {controls_to_set}")
        if self.camera_device:
            self.camera_device.set_controls(controls_to_set)
        
        # Flag that settings have changed, so the next capture callback can publish the updated state.
        with self.settings_lock:
            self.pending_settings_publication = True
        
        return SetParametersResult(successful=True)
    
    def _capture_loop(self):
        """
        The main capture loop, running in its own thread.
        This loop is "self-pacing," timed by the camera's hardware itself. It
        relies on the blocking `read_jpeg()` call, which waits for a new frame.
        This is more robust than a software timer (like `rclpy.Timer`), as it
        avoids clock drift and ensures every frame is processed as soon as it's
        available without constantly polling and wasting CPU.
        """
        self.get_logger().info("Capture thread started.")
        
        # On read failure, we sleep for one frame's duration before retrying.
        failure_sleep_duration = 1.0 / self.CAM_FPS if self.CAM_FPS > 0 else 0.05

        while not self.shutdown_event.is_set():
            if not (self.camera_device and self.camera_device.is_opened()):
                self.get_logger().warn("Camera device not open, sleeping for 1s before retry.", throttle_duration_sec=10)
                time.sleep(1.0)
                continue

            try:
                # 1. Read the raw, hardware-encoded JPEG data. This call blocks until a
                #    frame is available, naturally timing the loop to the camera's FPS.
                jpeg_data = self.camera_device.read_jpeg()
                
                if not jpeg_data:
                    # If read fails, sleep to prevent a tight busy-loop that consumes CPU.
                    time.sleep(failure_sleep_duration)
                    continue

                header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.get_parameter('ros.frame_id').value)

                # 2. Check if settings were changed and need to be re-published
                is_pending = False
                with self.settings_lock:
                    if self.pending_settings_publication:
                        is_pending = True
                        self.pending_settings_publication = False
                if is_pending:
                    self.update_and_publish_settings(header_to_use=header)

                # 3. Publish the main compressed image topic
                compressed_msg = CompressedImage(header=header, format="jpeg", data=jpeg_data)
                self.image_pub.publish(compressed_msg)
                
                # 4. Check if any auxiliary processors need the decoded frame
                needs_decode = self.low_bw_compressor or self.apriltag_processor or self.raw_image_publisher or self.calibrated_image_publisher
                
                if needs_decode:
                    decoded_frame = None
                    try:
                        # Decode the JPEG into a CV2 image matrix (BGR)
                        decoded_frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
                    except cv2.error as e:
                        self.get_logger().warn(f"Failed to decode JPEG frame: {e}. Skipping auxiliary processing.", throttle_duration_sec=5)
                        continue

                    if decoded_frame is None:
                        self.get_logger().warn("Decoded frame is None, possibly due to corruption. Skipping auxiliary processing.", throttle_duration_sec=5)
                        continue

                    # 5. Pass the single decoded frame to any active processors
                    if self.low_bw_compressor:
                        self.low_bw_compressor.update_frame(decoded_frame)
                    if self.apriltag_processor:
                        self.apriltag_processor.update_frame(decoded_frame)
                    if self.raw_image_publisher:
                        self.raw_image_publisher.update_frame(decoded_frame)
                    if self.calibrated_image_publisher:
                        self.calibrated_image_publisher.update_frame(decoded_frame)

            except Exception as e:
                self.get_logger().error(f"Exception in capture loop: {e}", exc_info=True)
                # Sleep to avoid spamming logs on repeated errors
                time.sleep(1.0)
        
        self.get_logger().info("Capture thread has been shut down.")

    def update_and_publish_settings(self, header_to_use=None):
        """Queries camera for its actual settings, creates a CameraSettings message, publishes it, and caches it."""
        if not self.camera_device: return

        try:
            current_controls = self.camera_device.get_all_controls()
        except Exception as e:
            self.get_logger().error(f"Failed to get camera controls: {e}", exc_info=True)
            return

        settings_msg = CameraSettings()
        settings_msg.header = header_to_use if header_to_use else Header(stamp=self.get_clock().now().to_msg(), frame_id=self.get_parameter('ros.frame_id').value)
        
        # Populate message from hardware values
        # Correctly interpret V4L2 auto exposure value (1=manual, 3=auto)
        settings_msg.auto_exposure = (current_controls.get('auto_exposure', 0) == self.V4L2_EXPOSURE_AUTO)
        settings_msg.exposure_time = current_controls.get('exposure_time', 0)
        settings_msg.brightness = current_controls.get('brightness', 0)
        settings_msg.contrast = current_controls.get('contrast', 0)
        settings_msg.saturation = current_controls.get('saturation', 0)
        settings_msg.hue = current_controls.get('hue', 0)
        settings_msg.gamma = current_controls.get('gamma', 0)
        settings_msg.gain = current_controls.get('gain', 0)
        settings_msg.sharpness = current_controls.get('sharpness', 0)
        settings_msg.white_balance_automatic = bool(current_controls.get('white_balance_automatic', 0))
        settings_msg.white_balance_temperature = current_controls.get('white_balance_temperature', 0)
        settings_msg.power_line_frequency = current_controls.get('power_line_frequency', 0)
        settings_msg.backlight_compensation = current_controls.get('backlight_compensation', 0)
        settings_msg.video_format = self.get_parameter('video.format').value
        
        self.cam_settings_pub.publish(settings_msg)

        # Cache the message for the periodic publisher
        with self.settings_lock:
            self.cam_settings_msg = settings_msg

    def periodic_settings_callback(self):
        """Periodically publishes the last known camera settings at 1Hz."""
        with self.settings_lock:
            # Check if the cached message has been initialized
            if not hasattr(self, 'cam_settings_msg') or not self.cam_settings_msg.video_format:
                return
            msg_to_publish = copy.deepcopy(self.cam_settings_msg)

        msg_to_publish.header.stamp = self.get_clock().now().to_msg()
        self.cam_settings_pub.publish(msg_to_publish)

    def get_current_controls_from_params(self, new_params=[]):
        """Constructs a dictionary of V4L2-compatible control values from ROS parameters."""
        current_values = {p: self.get_parameter(f'camera.{p}').value for p in self.camera_control_params}
        
        # Overlay any newly changed parameters
        for p in new_params:
            if p.name.startswith('camera.'):
                control_name = p.name.split('.')[-1]
                if control_name in current_values:
                    current_values[control_name] = p.value
        
        # Convert to the format expected by CameraDevice
        controls_to_set = {}
        for name, value in current_values.items():
            if name == 'auto_exposure':
                controls_to_set['auto_exposure'] = self.V4L2_EXPOSURE_AUTO if value else self.V4L2_EXPOSURE_MANUAL
            elif name == 'exposure_time':
                controls_to_set['exposure_absolute'] = value
            else:
                controls_to_set[name] = value
        
        # If auto exposure is on, the manual exposure setting should not be sent.
        if current_values.get('auto_exposure', False):
            if 'exposure_absolute' in controls_to_set:
                del controls_to_set['exposure_absolute']

        # If auto white balance is on, the manual temperature setting should not be sent.
        if current_values.get('white_balance_automatic', False):
            if 'white_balance_temperature' in controls_to_set:
                del controls_to_set['white_balance_temperature']

        return controls_to_set

    def cleanup_resources(self):
        """A centralized place to release all resources."""
        self.get_logger().info("Executing resource cleanup...")

        # Signal the capture thread to stop and wait for it to exit
        if self.capture_thread and self.capture_thread.is_alive():
            self.get_logger().info("Shutting down capture thread...")
            self.shutdown_event.set()
            self.capture_thread.join(timeout=2.0)
            if self.capture_thread.is_alive():
                self.get_logger().warn("Capture thread did not exit cleanly.")
        
        if self.cam_settings_timer: self.cam_settings_timer.cancel()
        
        if self.low_bw_compressor: self.low_bw_compressor.shutdown()
        if self.apriltag_processor: self.apriltag_processor.shutdown()
        if self.raw_image_publisher: self.raw_image_publisher.shutdown()
        
        if self.camera_device: self.camera_device.release()
        self.get_logger().info("Cleanup complete.")

def main():
    rclpy.init()
    node = None
    try:
        node = CameraNode()
        # Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass # Normal shutdown
    except Exception:
        # Log any other exceptions that might occur
        if node:
            node.get_logger().fatal("Unhandled exception in node execution:", exc_info=True)
        else:
            print("Unhandled exception during node setup:")
            traceback.print_exc()
    finally:
        if node:
            node.get_logger().info("Shutting down node and cleaning up resources.")
            node.cleanup_resources()
            # Explicitly destroy the node is good practice
            if rclpy.ok():
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()