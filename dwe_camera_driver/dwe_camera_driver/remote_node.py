import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import traceback
import sys
import os
from cv_bridge import CvBridge, CvBridgeError
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
        # Configure QoS profile for low-latency video streams.
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Use separate callback groups for the main subscription and for each
        # auxiliary processor. This allows the MultiThreadedExecutor to run them in parallel.
        self.subscription_cb_group = MutuallyExclusiveCallbackGroup()
        self.apriltag_cb_group = MutuallyExclusiveCallbackGroup()
        self.raw_image_cb_group = MutuallyExclusiveCallbackGroup()
        self.calibrated_image_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.get_logger().info("Initializing DWE Remote Processing Node...")

        # --- image message from camera ---
        self.cv_bridge = CvBridge()
        self._latest_msg = None
        

        # --- Auxiliary Processors (initialized to None) ---
        self.apriltag_processor = None
        self.raw_image_publisher = None
        self.calibrated_image_publisher = None
        
        try:
            # Initialization sequence
            self.setup_parameters()
            self.get_parameters_from_ros()
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
        self.declare_parameter('aux_process.img_raw_framerate', 15, readonly_descriptor)
        self.declare_parameter('aux_process.img_calibrated', False, readonly_descriptor)
        self.declare_parameter('aux_process.img_calibrated_framerate', 15, readonly_descriptor)

        # AprilTag parameters
        self.declare_parameter('apriltag.enable', False, readonly_descriptor)
        self.declare_parameter('apriltag.family', 'tag36h11', readonly_descriptor)
        self.declare_parameter('apriltag.size', 0.21, readonly_descriptor)
        self.declare_parameter('apriltag.publish_rate', 1, readonly_descriptor)
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
        member variables for easy access throughout the node.
        """
        self.get_logger().info("Getting and logging parameters...")
        self.subscribe_topic = self.get_parameter('subscribe_topic').value
        self.enable_img_raw = self.get_parameter('aux_process.img_raw').value
        self.img_raw_framerate = self.get_parameter('aux_process.img_raw_framerate').value
        self.enable_img_calibrated = self.get_parameter('aux_process.img_calibrated').value
        self.img_calibrated_framerate = self.get_parameter('aux_process.img_calibrated_framerate').value
        self.enable_apriltag = self.get_parameter('apriltag.enable').value
        self.apriltag_rate = self.get_parameter('apriltag.publish_rate').value

        self.get_logger().info(f"Subscribe Topic: {self.subscribe_topic}")
        self.get_logger().info(f"Raw Image Publisher: {'Enabled' if self.enable_img_raw else 'Disabled'}")
        if self.enable_img_raw:
            self.get_logger().info(f"  - Framerate: {self.img_raw_framerate} Hz")
        self.get_logger().info(f"Calibrated Image Publisher: {'Enabled' if self.enable_img_calibrated else 'Disabled'}")
        if self.enable_img_calibrated:
            self.get_logger().info(f"  - Framerate: {self.img_calibrated_framerate} Hz")
        self.get_logger().info(f"AprilTag Detection: {'Enabled' if self.enable_apriltag else 'Disabled'}")
        if self.enable_apriltag:
            self.get_logger().info(f"  - Publish Rate: {self.apriltag_rate} Hz")

    def setup_auxiliary_processors(self):
        """
        Conditionally initializes auxiliary processors based on configuration.
        """
        self.get_logger().info("Checking for auxiliary processors to enable...")

        # 1. Raw Image Publisher
        if self.enable_img_raw:
            self.get_logger().info("Enabling raw image publisher module.")
            from .stream_processors import RawImagePublisher
            self.raw_image_publisher = RawImagePublisher(self, self.img_raw_framerate, self.raw_image_cb_group, self.qos_profile)
        else:
            self.get_logger().info("Raw image stream is disabled.")

        # 2. AprilTag Processor
        if self.enable_apriltag:
            self.get_logger().info("Enabling AprilTag processor module.")
            from .apriltag_processor import AprilTagProcessor
            self.apriltag_processor = AprilTagProcessor(self, self.apriltag_cb_group, self.qos_profile)
        else:
            self.get_logger().info("AprilTag detection is disabled.")
        
        # 3. Calibrated Image Publisher
        if self.enable_img_calibrated:
            self.get_logger().info("Enabling calibrated image publisher module.")
            from .stream_processors import CalibratedImagePublisher
            self.calibrated_image_publisher = CalibratedImagePublisher(self, self.img_calibrated_framerate, self.calibrated_image_cb_group, self.qos_profile)
        else:
            self.get_logger().info("Calibrated image stream is disabled.")
            
    def setup_subscriber(self):
        """Initializes the subscriber to the input image topic."""
        self.get_logger().info(f"Subscribing to topic: {self.subscribe_topic}")
        
        self.subscription = self.create_subscription(
            CompressedImage,
            self.subscribe_topic,
            self.image_callback,
            qos_profile=self.qos_profile,
            callback_group=self.subscription_cb_group,
            )

    def image_callback(self, msg: CompressedImage):
        """
        Update _latest_msg with the newest image from the camera topic and then converts to cv2 image
        """
        self._latest_msg = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

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
        # Use a MultiThreadedExecutor to allow callbacks in different groups to run in parallel.
        executor = SingleThreadedExecutor()
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