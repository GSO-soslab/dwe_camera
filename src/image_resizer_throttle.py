#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ImageResizeThrottleNode:
    def __init__(self):
        rospy.init_node('image_resizer_throttle_node', anonymous=True)

        # --- Parameters ---
        # Input topic (original camera topic)
        self.input_topic = rospy.get_param('~input_topic', '/camera/image/compressed')
        # Output topic base for resized & throttled images
        self.output_topic_base = rospy.get_param('~output_topic_base', '/camera/image/resized_throttled')
        # Target dimensions
        self.target_width = rospy.get_param('~target_width', 640)
        self.target_height = rospy.get_param('~target_height', 480)
        # Target output rate (Hz). <= 0 means no throttling.
        self.target_rate = rospy.get_param('~target_rate', 5.0)
        # Output JPEG quality
        self.jpeg_quality = int(rospy.get_param('~jpeg_quality', 80)) # 0-100

        # --- Throttling Setup ---
        self.last_pub_time = rospy.Time(0) # Initialize to ensure first message is processed
        if self.target_rate > 0:
            self.throttle_period = rospy.Duration(1.0 / self.target_rate)
            rospy.loginfo(f"Throttling enabled. Target period: {self.throttle_period.to_sec():.3f}s ({self.target_rate} Hz)")
        else:
            self.throttle_period = rospy.Duration(0) # No throttling
            rospy.loginfo("Throttling disabled (target_rate <= 0).")

        self.bridge = CvBridge()
        # Subscribe to the original high-rate topic
        self.image_sub = rospy.Subscriber(self.input_topic, CompressedImage, self.image_callback, queue_size=5, buff_size=2**24) # queue_size=5, larger buffer
        # Publish the resized/throttled compressed image
        self.image_pub = rospy.Publisher(self.output_topic_base + "/compressed", CompressedImage, queue_size=1)

        rospy.loginfo(f"Image Resizer/Throttler started.")
        rospy.loginfo(f"  Input: '{self.input_topic}'")
        rospy.loginfo(f"  Output: '{self.output_topic_base}/compressed'")
        rospy.loginfo(f"  Target resolution: {self.target_width}x{self.target_height}")
        rospy.loginfo(f"  Target rate: {self.target_rate if self.target_rate > 0 else 'Unlimited'} Hz")
        rospy.loginfo(f"  JPEG Quality: {self.jpeg_quality}")


    def image_callback(self, msg):
        try:
            # --- Throttling Check ---
            if self.throttle_period > rospy.Duration(0):
                now = rospy.Time.now()
                if (now - self.last_pub_time) < self.throttle_period:
                    # Too soon, drop this message
                    return
                # Okay to publish, update time *before* processing
                self.last_pub_time = now

            # --- Decompression ---
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                rospy.logwarn_throttle(5.0, "Failed to decode compressed image.")
                return

            # --- Resizing ---
            resized_image = cv2.resize(cv_image, (self.target_width, self.target_height), interpolation=cv2.INTER_LINEAR)

            # --- Recompression ---
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encoded_jpeg = cv2.imencode('.jpg', resized_image, encode_param)

            if not result:
                rospy.logwarn_throttle(5.0, "Failed to re-encode image to JPEG.")
                return

            # --- Publish ---
            output_msg = CompressedImage()
            output_msg.header = msg.header # Use original timestamp
            output_msg.format = "jpeg"
            output_msg.data = encoded_jpeg.tobytes()

            self.image_pub.publish(output_msg)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}", exc_info=True)

if __name__ == '__main__':
    try:
        # Make sure dependencies are installed:
        # pip install opencv-python (or sudo apt-get install python3-opencv)
        # Ensure ros-<distro>-cv-bridge is installed
        node = ImageResizeThrottleNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass