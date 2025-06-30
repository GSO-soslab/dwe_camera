import rclpy
import cv2
import numpy as np
import threading
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage
from rclpy.parameter import ParameterDescriptor

class LowBandwidthCompressor:
    """
    An auxiliary processor that handles the creation of a low-bandwidth,
    resized, and re-compressed video stream. It operates on its own timer
    and only runs if enabled in the configuration.
    """
    def __init__(self, parent_node: rclpy.node.Node, callback_group):
        """
        Initializes the low-bandwidth compressor.
        
        :param parent_node: The main camera node to which this processor is attached.
        :param callback_group: The ROS 2 callback group for the timer.
        """
        self._node = parent_node
        self._logger = self._node.get_logger().get_child('low_bandwidth_compressor')
        
        # Get compression parameters from the parent node's parameters
        self._target_fps = self._node.get_parameter('compression.target_fps').value
        self._jpeg_quality = int(self._node.get_parameter('compression.jpeg_quality').value)
        
        # Declare/get compression dimensions. This makes the node robust even if they aren't in all YAML files.
        try:
            self._node.declare_parameter('compression.width', 320, ParameterDescriptor(read_only=True))
            self._node.declare_parameter('compression.height', 240, ParameterDescriptor(read_only=True))
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass # Parameters were already declared, which is fine.
        
        self._width = self._node.get_parameter('compression.width').value
        self._height = self._node.get_parameter('compression.height').value

        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_id = self._node.get_parameter('ros.frame_id').value
        
        # Create ROS publisher and timer
        self._publisher = self._node.create_publisher(CompressedImage, "image_lowbw/compressed", 10)
        self._timer = self._node.create_timer(
            1.0 / self._target_fps,
            self._timer_callback,
            callback_group=callback_group
        )
        self._logger.info(f"Initialized. Publishing at {self._target_fps} FPS with resolution {self._width}x{self._height} and quality {self._jpeg_quality}.")

    def update_frame(self, frame: np.ndarray):
        """Receives a new, decoded frame from the main capture loop."""
        with self._frame_lock:
            self._latest_frame = frame

    def _timer_callback(self):
        """Periodically processes and publishes the latest frame."""
        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame_copy = self._latest_frame.copy()

        try:
            # Resize the image to the target dimensions for the low-bandwidth stream
            resized_image = cv2.resize(frame_copy, (self._width, self._height), interpolation=cv2.INTER_LINEAR)
            
            # Re-encode the resized image as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
            result, encimg = cv2.imencode('.jpg', resized_image, encode_param)
            
            if result:
                header = Header(stamp=self._node.get_clock().now().to_msg(), frame_id=self._frame_id)
                msg = CompressedImage(header=header, format="jpeg", data=encimg.tobytes())
                self._publisher.publish(msg)
        except cv2.error as e:
            self._logger.error(f"OpenCV error during low-bandwidth compression: {e}")

    def shutdown(self):
        """Cancels the timer to cleanly shut down the processor."""
        self._logger.info("Shutting down.")
        if self._timer: self._timer.cancel()


class RawImagePublisher:
    """
    An auxiliary processor that publishes the raw, uncompressed video stream.
    It operates on its own timer and is only active if enabled.
    """
    def __init__(self, parent_node: rclpy.node.Node, publish_rate: float, callback_group):
        """
        Initializes the raw image publisher.
        
        :param parent_node: The main camera node.
        :param publish_rate: The rate (Hz) at which to publish raw images.
        :param callback_group: The ROS 2 callback group for the timer.
        """
        self._node = parent_node
        self._logger = self._node.get_logger().get_child('raw_image_publisher')
        self._bridge = CvBridge()
        
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_id = self._node.get_parameter('ros.frame_id').value

        # Create ROS publisher and timer
        self._publisher = self._node.create_publisher(Image, "image_raw", 10)
        self._timer = self._node.create_timer(
            1.0 / publish_rate,
            self._timer_callback,
            callback_group=callback_group
        )
        self._logger.info(f"Initialized. Publishing raw images at {publish_rate} Hz.")
        
    def update_frame(self, frame: np.ndarray):
        """Receives a new, decoded frame from the main capture loop."""
        with self._frame_lock:
            self._latest_frame = frame

    def _timer_callback(self):
        """Periodically converts and publishes the latest frame as a raw Image message."""
        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame_copy = self._latest_frame.copy()
            
        try:
            # Convert the OpenCV image to a ROS Image message
            msg = self._bridge.cv2_to_imgmsg(frame_copy, encoding='bgr8')
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = self._frame_id
            self._publisher.publish(msg)
        except CvBridgeError as e:
            self._logger.error(f"Error converting frame to Image message: {e}")

    def shutdown(self):
        """Cancels the timer to cleanly shut down the processor."""
        self._logger.info("Shutting down.")
        if self._timer: self._timer.cancel()