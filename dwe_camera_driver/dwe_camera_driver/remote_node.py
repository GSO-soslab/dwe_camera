import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import cv2
import numpy as np
import traceback

from sensor_msgs.msg import CompressedImage
from rcl_interfaces.msg import ParameterDescriptor

class RemoteNode(Node):
    """
    A ROS 2 node for remote processing of camera streams.
    This node subscribes to a compressed image topic, decodes the images,
    and performs auxiliary processing tasks like AprilTag detection or
    image rectification based on its configuration. It does not interface
    with any camera hardware directly.
    """
    def __init__(self):
        super().__init__('remote_node')

        # Use separate callback groups for timers to ensure responsiveness.
        self.apriltag_cb_group = MutuallyExclusiveCallbackGroup()
        self.raw_image_cb_group = MutuallyExclusiveCallbackGroup()
        self.calibrated_image_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.get_logger().info("Initializing DWE Remote Processing Node...")

        # --- State Management for Throttling ---
        # For throttling auxiliary processing to save CPU on decoding by not
        # processing every single incoming frame.
        self.last_update_times = {}
        self.processor_periods_ns = {}

        # --- Auxiliary Processors (initialized to None) ---
        self.apriltag_processor = None
        self.raw_image_publisher = None
        self.calibrated_image_publisher = None
        
        try:
            # Initialization sequence
            self.setup_parameters()
            self.get_parameters_from_ros()  # Explicitly get parameters
            self.setup_auxiliary_processors()
            self.setup_subscriber()
            
            self.get_logger().info("Remote processing node successfully initialized.")
            
        except Exception as e:
            self.get_logger().fatal(f"Fatal error during node initialization: {e}", exc_info=True)
            self.cleanup_resources()
            # Re-raise to notify the launch system
            raise

    def setup_parameters(self):
        """Declares and configures all ROS parameters for the remote node."""
        self.get_logger().info("Declaring parameters for remote node...")

        # All parameters are read-only after startup because they define the
        # processing pipeline, which is not meant to be changed dynamically.
        readonly_descriptor = ParameterDescriptor(read_only=True)

        # Subscription
        self.declare_parameter('subscribe_topic', '/dwe_camera/image/compressed', readonly_descriptor)

        # General
        self.declare_parameter('ros.frame_id', 'dwe_camera_frame', readonly_descriptor)
        self.declare_parameter('compression.jpeg_quality', 90, readonly_descriptor)

        # Aux Processing Flags and Rates
        self.declare_parameter('aux_process.img_raw', False, readonly_descriptor)
        self.declare_parameter('aux_process.img_raw_mono', False, readonly_descriptor)
        self.declare_parameter('aux_process.img_raw_framerate', 5, readonly_descriptor)
        self.declare_parameter('aux_process.img_calibrated', False, readonly_descriptor)
        # FIX: Corrected typo 'framrate' to 'framerate'
        self.declare_parameter('aux_process.img_calibrated_framerate', 15, readonly_descriptor)

        # AprilTag parameters
        self.declare_parameter('apriltag.enable', False, readonly_descriptor)
        self.declare_parameter('apriltag.family', 'tag36h11', readonly_descriptor)
        self.declare_parameter('apriltag.size', 0.16, readonly_descriptor)
        self.declare_parameter('apriltag.publish_rate', 2, readonly_descriptor)
        self.declare_parameter('apriltag.detector.nthreads', 1, readonly_descriptor)
        self.declare_parameter('apriltag.detector.quad_decimate', 2.0, readonly_descriptor)
        self.declare_parameter('apriltag.detector.quad_sigma', 0.0, readonly_descriptor)
        self.declare_parameter('apriltag.detector.refine_edges', True, readonly_descriptor)
        self.declare_parameter('apriltag.detector.decode_sharpening', 0.25, readonly_descriptor)

        # Camera calibration and dimension parameters (required for processing)
        self.declare_parameter('video.width', 1920, readonly_descriptor)
        self.declare_parameter('video.height', 1080, readonly_descriptor)
        self.declare_parameter('camera.intrinsics.fx', 1000.0, readonly_descriptor)
        self.declare_parameter('camera.intrinsics.fy', 1000.0, readonly_descriptor)
        self.declare_parameter('camera.intrinsics.cx', 960.0, readonly_descriptor)
        self.declare_parameter('camera.intrinsics.cy', 540.0, readonly_descriptor)
        self.declare_parameter('camera.distortion', [0.0, 0.0, 0.0, 0.0, 0.0], readonly_descriptor)
        self.declare_parameter('camera.fisheye', False, readonly_descriptor)
        self.declare_parameter('camera.undistort_crop', False, readonly_descriptor)

    def get_parameters_from_ros(self):
        """
        Retrieves the ROS parameters after declaration and stores them in
        member variables for easy access throughout the node. This makes it
        explicit that the parameters are being read and used.
        """
        self.get_logger().info("Getting and logging parameters...")
        self.subscribe_topic = self.get_parameter('subscribe_topic').value
        self.enable_img_raw = self.get_parameter('aux_process.img_raw').value
        self.enable_img_raw_mono = self.get_parameter('aux_process.img_raw_mono').value
        self.img_raw_framerate = self.get_parameter('aux_process.img_raw_framerate').value
        self.enable_img_calibrated = self.get_parameter('aux_process.img_calibrated').value
        self.img_calibrated_framerate = self.get_parameter('aux_process.img_calibrated_framerate').value
        self.enable_apriltag = self.get_parameter('apriltag.enable').value
        self.apriltag_rate = self.get_parameter('apriltag.publish_rate').value

        self.get_logger().info(f"Subscribe Topic: {self.subscribe_topic}")
        self.get_logger().info(f"Raw Image Publisher: {'Enabled' if self.enable_img_raw else 'Disabled'}")
        if self.enable_img_raw:
            self.get_logger().info(f"Grayscale: {self.enable_img_raw_mono}")
            self.get_logger().info(f"Framerate: {self.img_raw_framerate} Hz")
        self.get_logger().info(f"Calibrated Image Publisher: {'Enabled' if self.enable_img_calibrated else 'Disabled'}")
        if self.enable_img_calibrated:
            self.get_logger().info(f"Framerate: {self.img_calibrated_framerate} Hz")
        self.get_logger().info(f"AprilTag Detection: {'Enabled' if self.enable_apriltag else 'Disabled'}")
        if self.enable_apriltag:
            self.get_logger().info(f"Publish Rate: {self.apriltag_rate} Hz")

    def setup_auxiliary_processors(self):
        """
        Conditionally initializes auxiliary processors based on configuration
        and sets up throttling parameters for each.
        """
        self.get_logger().info("Checking for auxiliary processors to enable...")

        # 1. Raw Image Publisher
        if self.enable_img_raw:
            self.get_logger().info("Enabling raw image publisher module.")
            from .aux_processors.stream_processors import RawImagePublisher
            self.raw_image_publisher = RawImagePublisher(self, self.enable_img_raw_mono, self.img_raw_framerate, self.raw_image_cb_group)
            
            # Store the processing period in nanoseconds for throttling
            if self.img_raw_framerate > 0:
                self.processor_periods_ns['raw'] = 1e9 / self.img_raw_framerate
            else:
                self.processor_periods_ns['raw'] = 0 # Process every frame if rate is <= 0
            self.last_update_times['raw'] = self.get_clock().now()
        else:
            self.get_logger().info("Raw image stream is disabled.")

        # 2. AprilTag Processor
        if self.enable_apriltag:
            self.get_logger().info("Enabling AprilTag processor module.")
            from .aux_processors.apriltag_processor import AprilTagProcessor
            self.apriltag_processor = AprilTagProcessor(self, self.apriltag_cb_group)

            if self.apriltag_rate > 0:
                self.processor_periods_ns['apriltag'] = 1e9 / self.apriltag_rate
            else:
                self.processor_periods_ns['apriltag'] = 0
            self.last_update_times['apriltag'] = self.get_clock().now()
        else:
            self.get_logger().info("AprilTag detection is disabled.")
        
        # 3. Calibrated Image Publisher
        if self.enable_img_calibrated:
            self.get_logger().info("Enabling calibrated image publisher module.")
            from .aux_processors.stream_processors import CalibratedImagePublisher
            self.calibrated_image_publisher = CalibratedImagePublisher(self, self.img_calibrated_framerate, self.calibrated_image_cb_group)

            if self.img_calibrated_framerate > 0:
                self.processor_periods_ns['calibrated'] = 1e9 / self.img_calibrated_framerate
            else:
                self.processor_periods_ns['calibrated'] = 0
            self.last_update_times['calibrated'] = self.get_clock().now()
        else:
            self.get_logger().info("Calibrated image stream is disabled.")
            
    def setup_subscriber(self):
        """Initializes the subscriber to the input image topic."""
        self.get_logger().info(f"Subscribing to topic: {self.subscribe_topic}")
        
        # Use a ReentrantCallbackGroup to allow the subscription callback to be interrupted
        # by timer callbacks if processing is slow, preventing timer starvation.
        subscription_cb_group = ReentrantCallbackGroup()
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.subscribe_topic,
            self.image_callback,
            100, # QoS profile depth
            callback_group=subscription_cb_group)

    def image_callback(self, msg: CompressedImage):
        """
        Callback for incoming compressed image messages.
        This function checks if any auxiliary processor is due for processing
        based on its configured rate. If so, it decodes the image and passes
        it to the relevant processors. This avoids decoding every single frame
        if the processing rates are lower than the incoming stream rate.
        """
        now = self.get_clock().now()
        
        # Determine which processors are ready for a new frame based on their rate
        processors_ready_for_update = []
        if self.raw_image_publisher:
            period_ns = self.processor_periods_ns.get('raw', 0)
            if (now - self.last_update_times['raw']).nanoseconds >= period_ns:
                processors_ready_for_update.append('raw')
        
        if self.apriltag_processor:
            period_ns = self.processor_periods_ns.get('apriltag', 0)
            if (now - self.last_update_times['apriltag']).nanoseconds >= period_ns:
                processors_ready_for_update.append('apriltag')
                
        if self.calibrated_image_publisher:
            period_ns = self.processor_periods_ns.get('calibrated', 0)
            if (now - self.last_update_times['calibrated']).nanoseconds >= period_ns:
                processors_ready_for_update.append('calibrated')
        
        # If no processors are ready, we can skip decoding this frame entirely
        if not processors_ready_for_update:
            return

        self.get_logger().debug(f"Processors ready for update: {processors_ready_for_update}")

        try:
            # Decode the JPEG into a CV2 image matrix (BGR)
            decoded_frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        except cv2.error as e:
            self.get_logger().warn(f"Failed to decode JPEG frame: {e}. Skipping.", throttle_duration_sec=5)
            return

        if decoded_frame is None:
            self.get_logger().warn("Decoded frame is None, possibly due to corruption. Skipping.", throttle_duration_sec=5)
            return

        # Pass the single decoded frame to the processors that are ready
        # and update their last update time.
        if 'raw' in processors_ready_for_update:
            self.raw_image_publisher.update_frame(decoded_frame)
            self.last_update_times['raw'] = now
            
        if 'apriltag' in processors_ready_for_update:
            self.apriltag_processor.update_frame(decoded_frame)
            self.last_update_times['apriltag'] = now

        if 'calibrated' in processors_ready_for_update:
            self.calibrated_image_publisher.update_frame(decoded_frame)
            self.last_update_times['calibrated'] = now

    def cleanup_resources(self):
        """A centralized place to shut down all auxiliary processors."""
        self.get_logger().info("Executing resource cleanup for remote node...")
        if self.apriltag_processor: self.apriltag_processor.shutdown()
        if self.raw_image_publisher: self.raw_image_publisher.shutdown()
        if self.calibrated_image_publisher: self.calibrated_image_publisher.shutdown()
        self.get_logger().info("Cleanup complete.")

def main():
    rclpy.init()
    node = None
    try:
        node = RemoteNode()
        # Use a MultiThreadedExecutor to allow callbacks in different groups to run concurrently.
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass # Normal shutdown
    except Exception:
        if node:
            node.get_logger().fatal("Unhandled exception in node execution:", exc_info=True)
        else:
            print("Unhandled exception during node setup:")
            traceback.print_exc()
    finally:
        if node:
            node.get_logger().info("Shutting down node and cleaning up resources.")
            node.cleanup_resources()
            if rclpy.ok():
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()