import cv2
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage, Illuminance

import gc

class ImagePublisher(object):
    def __init__(self):
        # publisher
        self.image_pub = rospy.Publisher("~image", Image, queue_size=10)
        self.compressed_image_pub = rospy.Publisher("~image/compressed", CompressedImage, queue_size=10)
        self.exposure_time_pub = rospy.Publisher("~image/exposure_time", Illuminance, queue_size=10)
        # subscriber
        self.exposure_time_sub = rospy.Subscriber("~exposure", Float32, self.exposure_callback)

        self.bridge = CvBridge()
        self.cam = self.cam_params()
        self.frame_count = 0
        self.frame_id = rospy.get_param('~frame_id', 'dwe_camera')

        # Timer to call the run method periodically
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.CAM_FPS), self.timer_callback)

    def cam_params(self):
        '''
            Camera Parameters
            For H.264, see this: https://github.com/opencv/opencv-python/issues/100#issuecomment-394159998
        '''
        # Camera Index
        CAM_IDX = rospy.get_param('~video/id', 2)
        rospy.loginfo(f"Camera ID: {CAM_IDX}")
        # resolution
        WIDTH = rospy.get_param('~video/width', 800)
        HEIGHT = rospy.get_param('~video/height', 600)
        # used to set the pixel format to MJPEG/MJPG mode.
        MJPG = cv2.VideoWriter_fourcc(*'MJPG')
        # frame rate
        CAM_FPS = rospy.get_param('~video/framerate', 15)
        self.CAM_FPS = CAM_FPS
        # auto xposure
        AUTO_EXPO = rospy.get_param('~video/auto_exposure', 1)
        # exposure time
        EXPO_TIME = rospy.get_param('~video/exposure_time', 50)
        self.expo_time = EXPO_TIME

        # -- DEVICE SETUP --
        dwe_camera = cv2.VideoCapture(CAM_IDX)

        # set to MJPEG mode, by default idx 0 is YUYV
        # MJPG needs to be set, before resolution. Pixel format is always selected first
        dwe_camera.set(cv2.CAP_PROP_FOURCC, MJPG)
        dwe_camera.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        dwe_camera.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        dwe_camera.set(cv2.CAP_PROP_FPS, CAM_FPS)
        # (Optional) Disable auto exposure
        dwe_camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, AUTO_EXPO)
        dwe_camera.set(cv2.CAP_PROP_EXPOSURE, EXPO_TIME)

        # Error Check
        if ((dwe_camera == None) or (not dwe_camera.isOpened())):
            rospy.logerr('\nError - could not open video device.\n')
            exit(0)

        return dwe_camera

    def shutdown(self):
        rospy.loginfo("Releasing Camera!")
        self.cam.release()
    
    def exposure_callback(self, exposure_time):
        rospy.loginfo(f"Setting exposure time to: {exposure_time.data}")
        self.expo_time = exposure_time.data
        self.cam.set(cv2.CAP_PROP_EXPOSURE, exposure_time.data)

    def timer_callback(self, event):
        self.run()

    def run(self):
        self.success, self.frame = self.cam.read()
        self.time_now = rospy.get_rostime()

        # Image Message
        self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        self.image_message.header.seq = self.frame_count
        self.image_message.header.stamp = self.time_now
        self.image_message.header.frame_id = self.frame_id
        # Compressed Image Message
        self.image_message_compressed = self.bridge.cv2_to_compressed_imgmsg(self.frame)
        self.image_message_compressed.header.seq = self.frame_count
        self.image_message_compressed.header.stamp = self.time_now
        self.image_message_compressed.header.frame_id = self.frame_id
        # Exposure Time Message
        self.expo_time_message = Illuminance()
        self.expo_time_message.header.seq = self.frame_count
        self.expo_time_message.header.stamp = self.time_now
        self.expo_time_message.header.frame_id = self.frame_id
        self.expo_time_message.illuminance = self.expo_time
        # Publish
        self.image_pub.publish(self.image_message)
        self.compressed_image_pub.publish(self.image_message_compressed)
        self.exposure_time_pub.publish(self.expo_time_message)

        self.frame_count += 1

def main():
    rospy.init_node('dwe_camera_node')
    try:
        dwe_camera = ImagePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.shutdown()

    if rospy.is_shutdown():
        dwe_camera.shutdown()

if __name__ == '__main__':
    main()