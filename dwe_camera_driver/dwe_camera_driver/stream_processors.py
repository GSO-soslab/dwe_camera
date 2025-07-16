import rclpy
import cv2
import numpy as np
import threading
import copy
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Header
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from rcl_interfaces.msg import ParameterDescriptor

from .image_processing import ImageRectifier

class LowBandwidthCompressor:
    """
    An optimized auxiliary processor for creating a low-bandwidth video stream.

    Optimizations:
    - Avoids race conditions by using a local reference to the frame data.
    - Prevents re-processing and re-publishing of the same source frame.
    - Pre-calculates target dimensions and encoding parameters.
    - Uses a dictionary for cleaner configuration mapping.
    """
    def __init__(self, parent_node: rclpy.node.Node, callback_group, qos_profile):
        self._node = parent_node
        self._logger = self._node.get_logger().get_child('low_bandwidth_compressor')

        # Get compression parameters
        self._target_fps = self._node.get_parameter('compression.target_fps').value
        self._jpeg_quality = self._node.get_parameter('compression.jpeg_quality').value
        self._downscale_factor = self._node.get_parameter('compression.downscale').value
        self._frame_id = self._node.get_parameter('ros.frame_id').value
        source_width = self._node.get_parameter('video.width').value
        source_height = self._node.get_parameter('video.height').value
        self._encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        
        REDUCTION_MAP = {
            2: cv2.IMREAD_REDUCED_COLOR_2,
            4: cv2.IMREAD_REDUCED_COLOR_4,
            8: cv2.IMREAD_REDUCED_COLOR_8,
        }
        self._reduction_flag = REDUCTION_MAP.get(self._downscale_factor)
        if self._reduction_flag is None:
            self._logger.warn(
                f"Unsupported downscale_factor '{self._downscale_factor}'. "
                f"Must be one of {list(REDUCTION_MAP.keys())}. Defaulting to 2."
            )
            self._downscale_factor = 2
            self._reduction_flag = REDUCTION_MAP[self._downscale_factor]
        self._width = source_width // self._downscale_factor
        self._height = source_height // self._downscale_factor

        self._last_processed_data = None

        # Create ROS publisher and timer
        self._publisher = self._node.create_publisher(CompressedImage, "image_lowbw/compressed", qos_profile=qos_profile)
        timer_period = 1.0 / self._target_fps
        self._timer = self._node.create_timer(
            timer_period,
            self._timer_callback,
            callback_group=callback_group
        )
        target_size_str = f"{self._width}x{self._height}" if self._width > 0 else "auto"
        self._logger.info(
            f"Initialized low bandwidth compressed image. Publishing at <= {self._target_fps} FPS with target resolution "
            f"{target_size_str} and quality {self._jpeg_quality}."
        )

    def _timer_callback(self):
        """Periodically processes and publishes the latest frame."""
        # Get a local, stable reference to the latest data from the parent.
        frame_data = self._node.latest_compressed_img_jpg

        # If there's no new data OR if we've already processed this exact frame, do nothing.
        # The 'is' check is a very fast identity check (memory address).
        if frame_data is None or frame_data is self._last_processed_data:
            return
        # Mark this data as "processed" to prevent reprocessing on the next tick.
        self._last_processed_data = frame_data
        try:
            resized_image = cv2.imdecode(
                np.frombuffer(frame_data, np.uint8),
                self._reduction_flag
            )
            if resized_image is None:
                self._logger.warn("Failed to decode/reduce JPEG frame.", throttle_duration_sec=5)
                return
            result, encimg = cv2.imencode('.jpg', resized_image, self._encode_param)
            if result:
                header = Header(stamp=self._node.get_clock().now().to_msg(), frame_id=self._frame_id)
                msg = CompressedImage(header=header, format="jpeg", data=encimg.tobytes())
                self._publisher.publish(msg)
        except cv2.error as e:
            self._logger.warn(f"OpenCV error during frame processing: {e}", throttle_duration_sec=5)
        except Exception as e:
            self._logger.error(f"An unexpected error occurred: {e}", throttle_duration_sec=5)

    def shutdown(self):
        """Cancels the timer to cleanly shut down the processor."""
        self._logger.info("Shutting down.")
        if self._timer:
            self._timer.cancel()

class RawImagePublisher:
    """
    An auxiliary processor that publishes the raw, uncompressed video stream.
    It operates on its own timer and is only active if enabled.
    """
    def __init__(self, parent_node: rclpy.node.Node, publish_rate: float, callback_group, qos_profile):
        """
        Initializes the raw image publisher.
        
        :param parent_node: The main camera node.
        :param publish_rate: The rate (Hz) at which to publish raw images.
        :param callback_group: The ROS 2 callback group for the timer.
        """
        self._node = parent_node
        self._logger = self._node.get_logger().get_child('raw_image_publisher')
        self._bridge = CvBridge()
        self._frame_id = self._node.get_parameter('ros.frame_id').value

        # Create ROS publisher and timer
        self._publisher = self._node.create_publisher(Image, "image_raw", qos_profile=qos_profile)
        self._info_pub = self._node.create_publisher(CameraInfo, 'camera_info', qos_profile=qos_profile)
        self._timer = self._node.create_timer(
            1.0 / publish_rate,
            self._timer_callback,
            callback_group=callback_group
        )
        self._logger.info(f"Initialized. Publishing raw images at {publish_rate} Hz.")
    
    def _timer_callback(self):
        """Periodically converts and publishes the latest frame as a raw Image message."""
        if self._node._latest_msg is not None:
            frame_data = self._node._latest_msg
            try:
                header = Header(stamp=self._node.get_clock().now().to_msg(), frame_id=self._frame_id)
                msg = self._bridge.cv2_to_imgmsg(frame_data, encoding='bgr8')
                msg.header = header
                info_msg = CameraInfo()
                info_msg.width = msg.width
                info_msg.height = msg.height
                info_msg.header = header
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
    def __init__(self, parent_node: rclpy.node.Node, publish_rate: float, callback_group, qos_profile):
        """
        Initializes the calibrated image publisher.
        
        :param parent_node: The main camera node.
        :param publish_rate: The rate (Hz) at which to publish images.
        :param callback_group: The ROS 2 callback group for the timer.
        """
        self._node = parent_node
        self._bridge = CvBridge()
        self._logger = self._node.get_logger().get_child('calibrated_image_publisher')
        self._frame_id = self._node.get_parameter('ros.frame_id').value
        self.publish_rate = publish_rate
        self._jpeg_quality = int(self._node.get_parameter('compression.jpeg_quality').value)

        ##### Camera Parameters #####
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
        self._publisher = self._node.create_publisher(CompressedImage, "image_calibrated/compressed", qos_profile=qos_profile)
        self._timer = self._node.create_timer(
            1.0 / self.publish_rate,
            self._timer_callback,
            callback_group=callback_group
        )
        self._logger.info(f"Initialized. Publishing calibrated images at {self.publish_rate} FPS "
                          f"with resolution {new_w}x{new_h} and quality {self._jpeg_quality}.")

    def _timer_callback(self):
        """Periodically rectifies and publishes the result."""
        if self._node._latest_msg is not None:
            frame_data = self._node._latest_msg
            try:
                header = Header(stamp=self._node.get_clock().now().to_msg(), frame_id=self._frame_id)
                # Rectify the image using the centralized rectifier
                rectified_image = self._rectifier.rectify(frame_data)
                # Publish calibrated image as compressed image
                calibrated_img_msg = self._bridge.cv2_to_compressed_imgmsg(rectified_image)
                calibrated_img_msg.header = header

                self._publisher.publish(calibrated_img_msg)
            except CvBridgeError as e:
                self._logger.error(f"Error converting frame to Compresseed Image message: {e}")

    def shutdown(self):
        """Cancels the timer to cleanly shut down the processor."""
        self._logger.info("Shutting down.")
        if self._timer: self._timer.cancel()