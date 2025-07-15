import rclpy
import cv2
import numpy as np
import threading
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from rcl_interfaces.msg import ParameterDescriptor

from .image_processing import ImageRectifier

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
    def __init__(self, parent_node: rclpy.node.Node, mono: bool, publish_rate: float, callback_group):
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

        self.mono = mono

        # The image size is now determined dynamically from each frame in the callback
        # to ensure it matches the actual output of the camera hardware.

        # Create ROS publisher and timer
        self._publisher = self._node.create_publisher(Image, "image_raw", 10)
        self._info_pub = self._node.create_publisher(CameraInfo, 'camera_info', 10)
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
            time_now = self._node.get_clock().now().to_msg()
            
            # Convert the OpenCV image to a ROS Image message
            if self.mono:
                gray_image = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
                msg = self._bridge.cv2_to_imgmsg(gray_image, encoding='mono8')
            else:
                msg = self._bridge.cv2_to_imgmsg(frame_copy, encoding='bgr8')
            
            msg.header.stamp = time_now
            msg.header.frame_id = self._frame_id

            # Create and prepare the CameraInfo message
            info_msg = CameraInfo()
            # Use the dimensions from the created Image message to ensure they match.
            # This fixes the bug where CameraInfo reported the requested size
            # instead of the actual size from the camera hardware.
            info_msg.width = msg.width
            info_msg.height = msg.height
            info_msg.header.stamp = time_now
            info_msg.header.frame_id = self._frame_id
            
            # Publish both the image and its matching info message
            self._publisher.publish(msg)
            self._info_pub.publish(info_msg)

        except CvBridgeError as e:
            self._logger.error(f"Error converting frame to Image message: {e}")

    def shutdown(self):
        """Cancels the timer to cleanly shut down the processor."""
        self._logger.info("Shutting down.")
        if self._timer: self._timer.cancel()

class CalibratedImagePublisher:
    """
    Publishes a rectified (undistorted) and compressed image stream.
    """
    def __init__(self, parent_node: rclpy.node.Node, publish_rate: float, callback_group):
        """
        Initializes the calibrated image publisher.
        
        :param parent_node: The main camera node.
        :param publish_rate: The rate (Hz) at which to publish images.
        :param callback_group: The ROS 2 callback group for the timer.
        """
        self._node = parent_node
        self._logger = self._node.get_logger().get_child('calibrated_image_publisher')
        
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._frame_id = self._node.get_parameter('ros.frame_id').value
        self.publish_rate = publish_rate
        self._jpeg_quality = int(self._node.get_parameter('compression.jpeg_quality').value)

        # Get original camera parameters for rectification
        camera_intrinsics = {
            'fx': self._node.get_parameter('camera.intrinsics.fx').value,
            'fy': self._node.get_parameter('camera.intrinsics.fy').value,
            'cx': self._node.get_parameter('camera.intrinsics.cx').value,
            'cy': self._node.get_parameter('camera.intrinsics.cy').value
        }
        camera_distortion = self._node.get_parameter('camera.distortion').value
        is_fisheye = self._node.get_parameter('camera.fisheye').value
        crop_image = self._node.get_parameter('camera.undistort_crop').value
        image_size_tuple = (
            self._node.get_parameter('video.width').value,
            self._node.get_parameter('video.height').value
        )

        camera_matrix = np.array([
            [camera_intrinsics['fx'], 0, camera_intrinsics['cx']],
            [0, camera_intrinsics['fy'], camera_intrinsics['cy']],
            [0, 0, 1]
        ], dtype=np.float32)
        dist_coeffs = np.array(camera_distortion, dtype=np.float32)

        # Initialize the centralized image rectifier
        self._rectifier = ImageRectifier(
            logger=self._logger,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_size=image_size_tuple,
            is_fisheye=is_fisheye,
            crop_to_valid_pixels=crop_image
        )
        
        # Get the final image size for the logging message
        new_size = self._rectifier.get_new_image_size()
        new_w, new_h = new_size['img_width'], new_size['img_height']

        # Create ROS publisher and timer
        self._publisher = self._node.create_publisher(CompressedImage, "image_calibrated/compressed", 10)
        self._timer = self._node.create_timer(
            1.0 / self.publish_rate,
            self._timer_callback,
            callback_group=callback_group
        )
        self._logger.info(f"Initialized. Publishing calibrated images at {self.publish_rate} FPS "
                          f"with resolution {new_w}x{new_h} and quality {self._jpeg_quality}.")

    def update_frame(self, frame: np.ndarray):
        """Receives a new, decoded frame from the main capture loop."""
        with self._frame_lock:
            self._latest_frame = frame

    def _timer_callback(self):
        """Periodically rectifies and publishes the result."""
        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame_to_process = self._latest_frame.copy()

        # Rectify the image using the centralized rectifier
        rectified_image = self._rectifier.rectify(frame_to_process)
        
        # Compress the rectified image for publishing
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        result, encimg = cv2.imencode('.jpg', rectified_image, encode_param)
            
        if result:
            header = Header(stamp=self._node.get_clock().now().to_msg(), frame_id=self._frame_id)
            msg = CompressedImage(header=header, format="jpeg", data=encimg.tobytes())
            self._publisher.publish(msg)

    def shutdown(self):
        """Cancels the timer to cleanly shut down the processor."""
        self._logger.info("Shutting down.")
        if self._timer: self._timer.cancel()