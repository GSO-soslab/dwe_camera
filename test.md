Of course! I will create the new files for AprilTag detection and modify the existing files as requested. Here are the full contents of the new and changed files for you to copy and paste.

### New Files

Here are the new files required for the AprilTag detection functionality.

```markdown
# file path: dwe_camera_node/config/apriltag_camera.yaml
# Parameters for a single dwe_camera_node with AprilTag detection enabled.
# The top-level key matches the full node name: /{namespace}/{node_name}
/mini_alpha/dwe_camera_node:
  ros__parameters:
    # --- AprilTag Detection Settings ---
    apriltag:
      enable: True          # Set to True to enable AprilTag detection
      family: 'tag36h11'      # The family of tags to detect (e.g., 'tag36h11', 'tag25h9', 'tag16h5')
      size: 0.16            # The size of the tag's black square in meters
      publish_rate: 1.0     # Rate (Hz) to publish the detection image topic
    
    # Video stream settings
    video:
      id: 0
      width: 1920
      height: 1080
      framerate: 15
      format: 'MJPG'

    # Settings for the low-bandwidth compressed stream (0 target_fps will disable this topic)
    compression:
      width: 320
      height: 240
      target_fps: 5
      jpeg_quality: 75

    # Camera hardware control settings
    camera:
      # --- Camera Intrinsics (REQUIRED for pose estimation) ---
      # These values must be calibrated for your specific camera.
      intrinsics:
        fx: 1000.0  # Focal length in x
        fy: 1000.0  # Focal length in y
        cx: 960.0   # Principal point x
        cy: 540.0   # Principal point y
        
      auto_exposure: True
      exposure_time: 156
      brightness: 0
      contrast: 32
      saturation: 64
      hue: 0
      gamma: 100
      gain: 0
      sharpness: 3

    # ROS-specific settings
    ros:
      frame_id: "dwe_camera_frame"
```

```markdown
# file path: dwe_camera_node/dwe_camera/apriltag_detector.py
import apriltag
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

class AprilTagDetector:
    """
    A class to detect AprilTags in an image, perform pose estimation,
    and draw the results on the image.
    """
    def __init__(self, family, tag_size, camera_intrinsics, logger):
        """
        Initializes the AprilTag detector.

        :param family: The family of AprilTags to detect (e.g., 'tag36h11').
        :param tag_size: The size of the tags in meters.
        :param camera_intrinsics: A dictionary with camera intrinsic parameters [fx, fy, cx, cy].
        :param logger: A ROS 2 logger object for logging messages.
        """
        self.logger = logger
        self.tag_size = float(tag_size)
        
        # Check if the requested tag family is valid
        try:
            self.detector = apriltag.Detector(families=family, nthreads=1)
        except ValueError:
            self.logger.error(f"Invalid AprilTag family specified: '{family}'. Detection will fail.")
            # Create a detector with a default family to avoid crashing, but it won't work as intended.
            self.detector = apriltag.Detector(families='tag36h11', nthreads=1)

        # Camera parameters for pose estimation [fx, fy, cx, cy]
        self.camera_params = [
            camera_intrinsics['fx'],
            camera_intrinsics['fy'],
            camera_intrinsics['cx'],
            camera_intrinsics['cy']
        ]

    def _rotation_matrix_to_euler_angles(self, R_matrix):
        """
        Converts a rotation matrix to Euler angles (roll, pitch, yaw).
        """
        r = R.from_matrix(R_matrix)
        return r.as_euler('xyz', degrees=True) # roll, pitch, yaw

    def detect_and_draw(self, image):
        """
        Detects AprilTags in the given image, estimates their pose, and draws
        visualizations on the image.

        :param image: A BGR image (numpy array) from OpenCV.
        :return: The image with detections and pose information drawn on it.
        """
        if image is None:
            return None
        
        # Convert the image to grayscale for detection
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect tags
        detections = self.detector.detect(gray_image)
        
        # Process each detection
        for det in detections:
            # --- Pose Estimation ---
            # Returns tuple of (pose_R, pose_t, pose_err, pose_err_alt)
            # We are interested in the combined pose matrix from detection_pose
            try:
                pose, _, _ = self.detector.detection_pose(det, self.camera_params, self.tag_size)
            except Exception as e:
                self.logger.warn(f"Could not compute pose for tag {det.tag_id}: {e}")
                continue
            
            # Extract translation (t) and rotation (R)
            t = pose[0:3, 3]  # Translation vector [x, y, z] in meters
            R_matrix = pose[0:3, 0:3] # Rotation matrix
            
            # Convert rotation matrix to Euler angles
            roll, pitch, yaw = self._rotation_matrix_to_euler_angles(R_matrix)
            
            # --- Drawing on the image ---
            
            # 1. Draw the bounding box
            corners = det.corners.astype(int)
            cv2.polylines(image, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
            
            # 2. Draw the tag center
            center = det.center.astype(int)
            cv2.circle(image, tuple(center), 5, (0, 0, 255), -1)
            
            # 3. Prepare and draw the pose text
            pose_text_pos = (corners[0][0], corners[0][1] - 15)
            
            # Text for translation
            t_str = f"T: [{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}] m"
            # Text for rotation
            r_str = f"R: [{roll:.1f}, {pitch:.1f}, {yaw:.1f}] deg"
            # Tag ID
            id_str = f"ID: {det.tag_id}"

            # Put text on the image
            cv2.putText(image, id_str, (pose_text_pos[0], pose_text_pos[1] - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(image, t_str, (pose_text_pos[0], pose_text_pos[1] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            cv2.putText(image, r_str, pose_text_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        
        return image
```

```markdown
# file path: dwe_camera_node/launch/apriltag_camera.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run a single instance of the dwe_camera_node with AprilTag
    detection enabled.
    """
    dwe_camera_pkg_share = get_package_share_directory('dwe_camera')

    # Path to the parameters file that includes AprilTag settings
    param_config_path = os.path.join(
        dwe_camera_pkg_share,
        'config',
        'apriltag_camera.yaml'
    )

    # Define the Node action. The 'name' and 'namespace' must match the
    # top-level key in the apriltag_camera.yaml file.
    camera_node = Node(
        package='dwe_camera',
        executable='dwe_camera_node',
        name='dwe_camera_node',
        namespace="mini_alpha", # This matches the key in the YAML file
        output='screen',
        # Pass the file path directly. ROS 2 will handle loading and
        # applying the correct parameters based on the node's full name.
        parameters=[param_config_path],
        remappings=[
             # Remap the AprilTag topic to be more descriptive if needed.
             # This is optional, but good practice.
            ('apriltag_detection/compressed', 'mini_alpha/apriltag_detection/compressed'),
        ]
    )

    return LaunchDescription([
        camera_node
    ])
```

### Changed Files

Here are the full contents of the files that needed to be modified.

```markdown
# file path: dwe_camera_node/package.xml
<?xml version="1.0"?>
<package format="3">
  <name>dwe_camera</name>
  <version>0.0.0</version>
  <description>dwe camera ros 2 package</description>
  <maintainer email="yweifu@uri.edu">Yuewei</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>cv_bridge</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <depend>dwe_camera_interfaces</depend>

  <depend>rcl_interfaces</depend> <!-- For parameter services -->
  <depend>launch_ros</depend> <!-- For launch file utilities -->
  
  <exec_depend>python3-opencv</exec_depend>
  <!-- Add dependency for AprilTag detection -->
  <exec_depend>python3-apriltag</exec_depend> 
  <!-- Add dependency for scipy (used for rotation matrix conversion) -->
  <exec_depend>python3-scipy</exec_depend>
  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>

</package>
```

```markdown
# file path: dwe_camera_node/readme.md
# dwe_camera_node/readme.md
### Some Useful Commands

- rqt dynamic reconfigure gui
    ```bash
    ros2 run rqt_reconfigure rqt_reconfigure
    ```
- echo clear terminal
    ```bash
    ros2 topic echo -c /topic_name
    ```
- get usb camera device id
    ```bash
    v4l2-ctl --list-devices
    ```
- get camera supported formats and resolutions
    ```bash
    v4l2-ctl -d /dev/video{i} --list-formats-ext
    ```

### Dependencies
This package now uses OpenCV's V4L2 backend to capture JPEG streams directly, removing the need for GStreamer. The AprilTag detection feature requires additional Python libraries.

- All Ubuntu / Debian-based systems (including Raspberry Pi):
    
    **Python Packages (pip)**
    The recommended way to install dependencies for Python is via pip:
    ```bash
    pip3 install opencv-python numpy lark apriltag scipy empy catkin_pkg
    ```
    (Note: `cv_bridge` is a standard ROS package and should be installed via `rosdep` or `sudo apt install ros-<distro>-cv-bridge`, not pip).

    **System Libraries (apt)**
    ```bash
    sudo apt install v4l-utils
    sudo apt install ros-<distro>-cv-bridge
    sudo apt install ros-<distro>-compressed-image-transport
    ```

- Raspberry Pi also requires:
    ```bash
    sudo apt install libgl1
    ```
```

```markdown
# file path: dwe_camera_node/dwe_camera/__init__.py
from .cv2_v4l2 import V4L2Camera
from .apriltag_detector import AprilTagDetector
```

```markdown
# file path: dwe_camera_node/dwe_camera/dwe_camera_node.py
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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

from .cv2_v4l2 import V4L2Camera
from .apriltag_detector import AprilTagDetector


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('dwe_camera_node')

        self.get_logger().info("Using separate callback groups for camera timers to ensure parameter services remain responsive.")
        self.image_capture_cb_group = MutuallyExclusiveCallbackGroup()
        self.compression_cb_group = MutuallyExclusiveCallbackGroup()
        self.cam_setting_cb_group = MutuallyExclusiveCallbackGroup()
        # Add a callback group for AprilTag detection
        self.apriltag_cb_group = MutuallyExclusiveCallbackGroup()

        self.settings_lock = threading.Lock()
        self.pending_settings_publication = False
        self.cam_settings_msg = CamParameters()
        self.supported_controls = {}

        self.v4l2_camera = None
        self.latest_raw_frame = None
        self.latest_header = Header()
        self.raw_image_timer = None
        self.compressed_image_timer = None
        self.cam_settings_timer = None
        
        # AprilTag related members
        self.apriltag_detector = None
        self.apriltag_timer = None
        self.apriltag_pub = None

        self.camera_control_params = [
            'brightness', 'contrast', 'saturation', 'hue', 'gamma', 'gain', 'sharpness',
            'auto_exposure', 'exposure_time'
        ]
        
        self.V4L2_EXPOSURE_MANUAL = 1
        self.V4L2_EXPOSURE_AUTO = 3

        try:
            self.setup_params()
            self.setup_ros_elements()
            self.setup_cam()
            self.setup_apriltag_detection() # New setup function for AprilTag
            self.set_unsupported_params_to_readonly()
            self.add_on_set_parameters_callback(self.parameters_callback)
            self.setup_timers()
            self.get_logger().info("DWE Camera Node successfully initialized.")
        except Exception as e:
            self.get_logger().error(f"Error during node initialization: {e}", exc_info=True)
            self.cleanup_resources()
            raise

    def cleanup_resources(self):
        self.get_logger().info("Executing resource cleanup...")
        if self.raw_image_timer: self.raw_image_timer.cancel()
        if self.compressed_image_timer: self.compressed_image_timer.cancel()
        if self.cam_settings_timer: self.cam_settings_timer.cancel()
        if self.apriltag_timer: self.apriltag_timer.cancel() # Cleanup AprilTag timer
        if self.v4l2_camera: self.v4l2_camera.release()

    def setup_params(self):
        """
        Declares and configures ROS parameters for the node.
        """
        # === Existing parameter declarations... (no changes here) ===
        brightness_descriptor = ParameterDescriptor(description='Image brightness [-64, 64]', integer_range=[IntegerRange(from_value=-64, to_value=64, step=1)])
        contrast_descriptor = ParameterDescriptor(description='Image contrast [0, 64]', integer_range=[IntegerRange(from_value=0, to_value=64, step=1)])
        saturation_descriptor = ParameterDescriptor(description='Image saturation [0, 128]', integer_range=[IntegerRange(from_value=0, to_value=128, step=1)])
        hue_descriptor = ParameterDescriptor(description='Image hue [-40, 40]', integer_range=[IntegerRange(from_value=-40, to_value=40, step=1)])
        gamma_descriptor = ParameterDescriptor(description='Image gamma [72, 500]', integer_range=[IntegerRange(from_value=72, to_value=500, step=1)])
        gain_descriptor = ParameterDescriptor(description='Image gain [0, 100]', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)])
        sharpness_descriptor = ParameterDescriptor(description='Image sharpness [0, 6]', integer_range=[IntegerRange(from_value=0, to_value=6, step=1)])
        exposure_descriptor = ParameterDescriptor(description='Exposure time [1, 5000]. Used when auto_exposure is False.', integer_range=[IntegerRange(from_value=1, to_value=5000, step=1)])
        auto_exposure_descriptor = ParameterDescriptor(description='Enable/disable auto exposure')
        frame_id_descriptor = ParameterDescriptor(description='The TF frame ID for the camera images. Read-only after startup.', read_only=True)
        video_id_descriptor = ParameterDescriptor(description='Camera device ID (e.g., /dev/videoX). Read-only after startup.', read_only=True)
        video_width_descriptor = ParameterDescriptor(description='Capture width in pixels. Read-only after startup.', read_only=True)
        video_height_descriptor = ParameterDescriptor(description='Capture height in pixels. Read-only after startup.', read_only=True)
        video_framerate_descriptor = ParameterDescriptor(description='Requested capture framerate (Hz). Read-only after startup.', read_only=True)
        video_format_descriptor = ParameterDescriptor(description='Capture format (e.g., MJPG). Read-only after startup.', read_only=True)
        compression_width_descriptor = ParameterDescriptor(description='Width for the low-bandwidth compressed stream. Read-only after startup.', integer_range=[IntegerRange(from_value=80, to_value=1920, step=1)], read_only=True)
        compression_height_descriptor = ParameterDescriptor(description='Height for the low-bandwidth compressed stream. Read-only after startup.', integer_range=[IntegerRange(from_value=60, to_value=1080, step=1)], read_only=True)
        compression_fps_descriptor = ParameterDescriptor(description='Target FPS for the low-bandwidth compressed stream. Read-only after startup.', floating_point_range=[FloatingPointRange(from_value=0.0, to_value=30.0, step=0.5)], read_only=True)
        jpeg_quality_descriptor = ParameterDescriptor(description='JPEG quality for low-bandwidth stream [0, 100]. Read-only after startup.', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)], read_only=True)

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

        # --- New AprilTag and Camera Intrinsics Parameters ---
        readonly_descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter('apriltag.enable', False, ParameterDescriptor(description='Enable/disable AprilTag detection.', read_only=True))
        self.declare_parameter('apriltag.family', 'tag36h11', ParameterDescriptor(description='AprilTag family to detect.', read_only=True))
        self.declare_parameter('apriltag.size', 0.16, ParameterDescriptor(description='Size of the AprilTag in meters.', read_only=True))
        self.declare_parameter('apriltag.publish_rate', 1.0, ParameterDescriptor(description='Rate (Hz) for publishing detection images.', read_only=True))
        
        self.declare_parameter('camera.intrinsics.fx', 1000.0, readonly_descriptor)
        self.declare_parameter('camera.intrinsics.fy', 1000.0, readonly_descriptor)
        self.declare_parameter('camera.intrinsics.cx', 960.0, readonly_descriptor)
        self.declare_parameter('camera.intrinsics.cy', 540.0, readonly_descriptor)

    def setup_cam(self):
        """Initializes the V4L2Camera with settings from ROS parameters."""
        cam_id = self.get_parameter('video.id').value
        width = self.get_parameter('video.width').value
        height = self.get_parameter('video.height').value
        fps_req = self.get_parameter('video.framerate').value

        initial_controls = self.get_current_controls_from_params()
        self.get_logger().info(f"Initial controls from parameters: {initial_controls}")
        
        self.v4l2_camera = V4L2Camera(
            device_id=cam_id, width=width, height=height,
            framerate=fps_req, logger=self.get_logger(),
            initial_controls=initial_controls
        )

        self.CAM_FPS = self.v4l2_camera.fps
        self.get_logger().info("V4L2 camera initialized successfully via OpenCV.")

        self.supported_controls = self.v4l2_camera.get_supported_controls()
        self.get_logger().info(f"Hardware control support map: {self.supported_controls}")

        self.update_and_publish_settings()
        self.get_logger().info("Published initial camera settings and cached for periodic updates.")
    
    def setup_apriltag_detection(self):
        """Initializes the AprilTag detector if enabled by parameters."""
        if not self.get_parameter('apriltag.enable').value:
            self.get_logger().info("AprilTag detection is disabled via parameters.")
            return

        self.get_logger().info("AprilTag detection is enabled. Initializing detector...")
        
        tag_family = self.get_parameter('apriltag.family').value
        tag_size = self.get_parameter('apriltag.size').value
        
        # Gather camera intrinsics from parameters
        camera_intrinsics = {
            'fx': self.get_parameter('camera.intrinsics.fx').value,
            'fy': self.get_parameter('camera.intrinsics.fy').value,
            'cx': self.get_parameter('camera.intrinsics.cx').value,
            'cy': self.get_parameter('camera.intrinsics.cy').value
        }
        
        # Check for default/unset intrinsic values
        if camera_intrinsics['fx'] <= 1.0 or camera_intrinsics['fy'] <= 1.0:
            self.get_logger().error(
                "Camera intrinsics (fx, fy) are not realistically set. Pose estimation will be incorrect. "
                "Please provide a camera calibration file."
            )

        self.apriltag_detector = AprilTagDetector(
            family=tag_family,
            tag_size=tag_size,
            camera_intrinsics=camera_intrinsics,
            logger=self.get_logger()
        )
        
        # Create the publisher for detection images
        self.apriltag_pub = self.create_publisher(CompressedImage, "apriltag_detection/compressed", 10)
        self.get_logger().info("AprilTag detector and publisher created.")


    def set_unsupported_params_to_readonly(self):
        descriptor_map = {
            'brightness': ParameterDescriptor(description='Image brightness [-64, 64]', integer_range=[IntegerRange(from_value=-64, to_value=64, step=1)]),
            'contrast': ParameterDescriptor(description='Image contrast [0, 64]', integer_range=[IntegerRange(from_value=0, to_value=64, step=1)]),
            'saturation': ParameterDescriptor(description='Image saturation [0, 128]', integer_range=[IntegerRange(from_value=0, to_value=128, step=1)]),
            'hue': ParameterDescriptor(description='Image hue [-40, 40]', integer_range=[IntegerRange(from_value=-40, to_value=40, step=1)]),
            'gamma': ParameterDescriptor(description='Image gamma [72, 500]', integer_range=[IntegerRange(from_value=72, to_value=500, step=1)]),
            'gain': ParameterDescriptor(description='Image gain [0, 100]', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)]),
            'sharpness': ParameterDescriptor(description='Image sharpness [0, 6]', integer_range=[IntegerRange(from_value=0, to_value=6, step=1)]),
            'auto_exposure': ParameterDescriptor(description='Enable/disable auto exposure'),
            'exposure_time': ParameterDescriptor(description='Exposure time [1, 5000]. Used when auto_exposure is False.', integer_range=[IntegerRange(from_value=1, to_value=5000, step=1)])
        }
        for control_name in self.camera_control_params:
            if not self.supported_controls.get(control_name, False):
                param_full_name = f'camera.{control_name}'
                self.get_logger().warn(f"Control '{control_name}' is not supported by this camera. Setting its ROS parameter to read-only.")
                try:
                    current_value = self.get_parameter(param_full_name).value
                    new_descriptor = descriptor_map[control_name]
                    new_descriptor.read_only = True
                    self.undeclare_parameter(param_full_name)
                    self.declare_parameter(param_full_name, current_value, new_descriptor)
                except KeyError:
                    self.get_logger().error(f"Internal error: No descriptor found for '{control_name}' while setting read-only status.")
                except Exception as e:
                    self.get_logger().error(f"Failed to set parameter '{param_full_name}' to read-only: {e}")

    def get_current_controls_from_params(self, new_params=[]):
        current_values = {p: self.get_parameter(f'camera.{p}').value for p in self.camera_control_params}
        for p in new_params:
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
            self.raw_image_timer = self.create_timer(
                1.0 / self.CAM_FPS, 
                self.raw_image_capture_callback,
                callback_group=self.image_capture_cb_group)
        else:
            self.get_logger().warn("Camera FPS is 0. Raw image timer will not be started.")

        compressed_fps = float(self.get_parameter('compression.target_fps').value)
        if compressed_fps > 0:
            self.compressed_image_timer = self.create_timer(
                1.0 / compressed_fps, 
                self.compressed_image_callback,
                callback_group=self.compression_cb_group)
        
        self.cam_settings_timer = self.create_timer(
            1.0,
            self.camera_settings_callback,
            callback_group=self.cam_setting_cb_group
        )
        
        # Setup AprilTag timer if detection is enabled
        if self.apriltag_detector is not None:
            publish_rate = self.get_parameter('apriltag.publish_rate').value
            if publish_rate > 0:
                self.apriltag_timer = self.create_timer(
                    1.0 / publish_rate,
                    self.apriltag_detection_callback,
                    callback_group=self.apriltag_cb_group
                )
                self.get_logger().info(f"AprilTag detection timer started at {publish_rate} Hz.")
            else:
                 self.get_logger().warn("AprilTag publish rate is 0. Detection topic will not be published.")
        
    def parameters_callback(self, params):
        self.get_logger().info("Parameter callback triggered!")
        controls_to_set = self.get_current_controls_from_params(params)
        self.get_logger().info(f"Applying new camera parameter set: {controls_to_set}")
        if self.v4l2_camera:
            self.v4l2_camera.set_controls(controls_to_set)
        with self.settings_lock:
            self.pending_settings_publication = True
        self.get_logger().info("Settings changed. Awaiting next frame to publish synchronized status.")
        return SetParametersResult(successful=True)

    def update_and_publish_settings(self, header_to_use=None):
        if not self.v4l2_camera:
            self.get_logger().warn("Cannot update settings, camera not initialized.")
            return
        try:
            current_controls = self.v4l2_camera.get_all_controls()
        except Exception as e:
            self.get_logger().error(f"Failed to get camera controls: {e}", exc_info=True)
            return
        settings_msg = CamParameters()
        if header_to_use:
            settings_msg.header = header_to_use
        else:
            settings_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.latest_header.frame_id)
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
        self.cam_settings_pub.publish(settings_msg)
        with self.settings_lock:
            self.cam_settings_msg = settings_msg

    def camera_settings_callback(self):
        with self.settings_lock:
            if not hasattr(self, 'cam_settings_msg') or not self.cam_settings_msg.video_format:
                return
            msg_to_publish = copy.deepcopy(self.cam_settings_msg)
        msg_to_publish.header.stamp = self.get_clock().now().to_msg()
        self.cam_settings_pub.publish(msg_to_publish)

    def raw_image_capture_callback(self):
        if not (self.v4l2_camera and self.v4l2_camera.is_opened()): return
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.latest_header.frame_id)
        jpeg_data = self.v4l2_camera.read_jpeg()
        if jpeg_data:
            is_pending = False
            with self.settings_lock:
                if self.pending_settings_publication:
                    is_pending = True
                    self.pending_settings_publication = False
            if is_pending:
                self.get_logger().info(f"Publishing synchronized settings with timestamp: {header.stamp.sec}.{header.stamp.nanosec}")
                self.update_and_publish_settings(header_to_use=header)

            compressed_msg = CompressedImage(header=header, format="jpeg", data=jpeg_data)
            self.image_pub.publish(compressed_msg)
            
            # Decode frame if either low-bw stream or apriltag detection is active
            if (self.compressed_image_timer and not self.compressed_image_timer.is_canceled()) or \
               (self.apriltag_timer and not self.apriltag_timer.is_canceled()):
                try:
                    # Thread-safe update of the latest frame
                    decoded_frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
                    with self.settings_lock: # Reuse lock for frame
                        self.latest_raw_frame = decoded_frame
                except cv2.error: 
                    with self.settings_lock:
                        self.latest_raw_frame = None

    def apriltag_detection_callback(self):
        """
        Processes the latest frame for AprilTags and publishes the result.
        """
        # Ensure the detector and publisher are ready
        if self.apriltag_detector is None or self.apriltag_pub is None:
            return

        # Get a thread-safe copy of the latest frame
        with self.settings_lock:
            frame_to_process = copy.deepcopy(self.latest_raw_frame)

        if frame_to_process is None:
            return
        
        # Perform detection and drawing
        annotated_image = self.apriltag_detector.detect_and_draw(frame_to_process)
        
        if annotated_image is not None:
            # Compress the annotated image for publishing
            q = self.get_parameter('compression.jpeg_quality').value
            result, encimg = cv2.imencode('.jpg', annotated_image, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            
            if result:
                header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.latest_header.frame_id)
                msg = CompressedImage(header=header, format="jpeg", data=encimg.tobytes())
                self.apriltag_pub.publish(msg)
            else:
                self.get_logger().warn("Failed to encode AprilTag detection image to JPEG.")

    def compressed_image_callback(self):
        with self.settings_lock: # Get a copy of the frame safely
            frame_to_process = copy.deepcopy(self.latest_raw_frame)
        
        if frame_to_process is None: return

        w = self.get_parameter('compression.width').value
        h = self.get_parameter('compression.height').value
        q = self.get_parameter('compression.jpeg_quality').value
        try:
            resized_image = cv2.resize(frame_to_process, (w, h), cv2.INTER_LINEAR)
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
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if node:
            node.get_logger().fatal("Unhandled exception in node execution:", exc_info=True)
        else:
            print("Unhandled exception during node setup:")
            traceback.print_exc()
    finally:
        if node:
            node.get_logger().info("Shutting down node and cleaning up resources.")
            node.destroy_node_custom()
            if rclpy.ok():
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
```