import cv2
import rospy
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CompressedImage, Illuminance
from dwe_camera.msg import CamParameters
from dwe_camera.cfg import CamSettingsConfig

class ImagePublisher():
    def __init__(self):
        # publisher
        self.image_pub = rospy.Publisher("~image", Image, queue_size=10)
        self.compressed_image_pub = rospy.Publisher("~image/compressed", CompressedImage, queue_size=10)
        self.cam_settings_pub = rospy.Publisher("~camera_settings", CamParameters, queue_size=10)

        # subscriber
        self.cam_settings_sub = rospy.Subscriber("~set_cam_settings", CamParameters, self.cam_settings_callback)

        self.bridge = CvBridge()
        self.cam = self.cam_params()
        self.frame_count = 0
        self.frame_id = rospy.get_param('~frame_id', 'dwe_camera')

        self.srv = Server(CamSettingsConfig, self.dynamic_config_callback)

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
        # auto xposure
        AUTO_EXPO = rospy.get_param('~video/auto_exposure', 1)
        # exposure time
        EXPO_TIME = rospy.get_param('~video/exposure_time', 50)

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
        if AUTO_EXPO == 1:
            dwe_camera.set(cv2.CAP_PROP_EXPOSURE, EXPO_TIME)

        self.expo_time = dwe_camera.get(cv2.CAP_PROP_EXPOSURE)
        self.CAM_FPS = dwe_camera.get(cv2.CAP_PROP_FPS)

        # Error Check
        if ((dwe_camera == None) or (not dwe_camera.isOpened())):
            rospy.logerr('\nError - could not open video device.\n')
            exit(0)

        return dwe_camera

    def shutdown(self):
        rospy.loginfo("Releasing Camera!")
        self.cam.release()
    
    def cam_settings_callback(self, cam_settings):
        self.cam.set(cv2.CAP_PROP_BRIGHTNESS, cam_settings.brightness)
        self.cam.set(cv2.CAP_PROP_CONTRAST, cam_settings.contrast)
        self.cam.set(cv2.CAP_PROP_SATURATION, cam_settings.saturation)
        self.cam.set(cv2.CAP_PROP_HUE, cam_settings.hue)
        self.cam.set(cv2.CAP_PROP_GAMMA, cam_settings.gamma)
        self.cam.set(cv2.CAP_PROP_GAIN, cam_settings.gain)
        self.cam.set(cv2.CAP_PROP_SHARPNESS, cam_settings.sharpness)
        self.cam.set(cv2.CAP_PROP_EXPOSURE, cam_settings.exposure)

        rospy.loginfo(f"Camera Settings: \n"
              f"brightness: {int(self.cam.get(cv2.CAP_PROP_BRIGHTNESS))}\n"
              f"contrast: {int(self.cam.get(cv2.CAP_PROP_CONTRAST))}\n"
              f"saturation: {int(self.cam.get(cv2.CAP_PROP_SATURATION))}\n"
              f"hue: {int(self.cam.get(cv2.CAP_PROP_HUE))}\n"
              f"gamma: {int(self.cam.get(cv2.CAP_PROP_GAMMA))}\n"
              f"gain: {int(self.cam.get(cv2.CAP_PROP_GAIN))}\n"
              f"sharpness: {int(self.cam.get(cv2.CAP_PROP_SHARPNESS))}\n"
              f"exposure: {int(self.cam.get(cv2.CAP_PROP_EXPOSURE))}")
        

    def timer_callback(self, event):
        self.run()
    
    def dynamic_config_callback(self, cam_params, level):
        self.cam.set(cv2.CAP_PROP_BRIGHTNESS, cam_params.brightness)
        self.cam.set(cv2.CAP_PROP_CONTRAST, cam_params.contrast)
        self.cam.set(cv2.CAP_PROP_SATURATION, cam_params.saturation)
        self.cam.set(cv2.CAP_PROP_HUE, cam_params.hue)
        self.cam.set(cv2.CAP_PROP_GAMMA, cam_params.gamma)
        self.cam.set(cv2.CAP_PROP_GAIN, cam_params.gain)
        self.cam.set(cv2.CAP_PROP_SHARPNESS, cam_params.sharpness)
        self.cam.set(cv2.CAP_PROP_EXPOSURE, cam_params.exposure)
        return cam_params

    def get_cam_settings(self):
        cam_settings = CamParameters()
        cam_settings.header.seq = self.frame_count
        cam_settings.header.stamp = self.time_now
        cam_settings.header.frame_id = self.frame_id
        cam_settings.brightness = int(self.cam.get(cv2.CAP_PROP_BRIGHTNESS))
        cam_settings.contrast = int(self.cam.get(cv2.CAP_PROP_CONTRAST))
        cam_settings.saturation = int(self.cam.get(cv2.CAP_PROP_SATURATION))
        cam_settings.hue = int(self.cam.get(cv2.CAP_PROP_HUE))
        cam_settings.gamma = int(self.cam.get(cv2.CAP_PROP_GAMMA))
        cam_settings.gain = int(self.cam.get(cv2.CAP_PROP_GAIN))
        cam_settings.sharpness = int(self.cam.get(cv2.CAP_PROP_SHARPNESS))
        cam_settings.exposure = int(self.cam.get(cv2.CAP_PROP_EXPOSURE))

        return cam_settings
        

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
        # Camera Settings Message
        self.cam_settings = self.get_cam_settings()

        # Publish
        self.image_pub.publish(self.image_message)
        self.compressed_image_pub.publish(self.image_message_compressed)
        self.cam_settings_pub.publish(self.cam_settings)

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