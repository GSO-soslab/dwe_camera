"""
A separate module for declaring all ROS parameters for the camera node.
This helps to keep the main camera_node.py file cleaner.
"""
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

def get_camera_control_descriptors():
    """
    Returns a dictionary of ParameterDescriptor objects for camera controls.
    This serves as the single source of truth for these descriptors.
    """
    return {
        'brightness': ParameterDescriptor(description='Image brightness [-64, 64]', integer_range=[IntegerRange(from_value=-64, to_value=64, step=1)]),
        'contrast': ParameterDescriptor(description='Image contrast [0, 64]', integer_range=[IntegerRange(from_value=0, to_value=64, step=1)]),
        'saturation': ParameterDescriptor(description='Image saturation [0, 128]', integer_range=[IntegerRange(from_value=0, to_value=128, step=1)]),
        'hue': ParameterDescriptor(description='Image hue [-40, 40]', integer_range=[IntegerRange(from_value=-40, to_value=40, step=1)]),
        'gamma': ParameterDescriptor(description='Image gamma [72, 500]', integer_range=[IntegerRange(from_value=72, to_value=500, step=1)]),
        'gain': ParameterDescriptor(description='Image gain [0, 100]', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)]),
        'sharpness': ParameterDescriptor(description='Image sharpness [0, 6]', integer_range=[IntegerRange(from_value=0, to_value=6, step=1)]),
        'exposure_time': ParameterDescriptor(description='Exposure time [1, 5000]. Used when auto_exposure is False.', integer_range=[IntegerRange(from_value=1, to_value=5000, step=1)]),
        'auto_exposure': ParameterDescriptor(description='Enable/disable auto exposure'),
        'white_balance_automatic': ParameterDescriptor(description='Enable/disable auto white balance'),
        'white_balance_temperature': ParameterDescriptor(description='White balance temperature [2800, 6500]. Used when auto is False.', integer_range=[IntegerRange(from_value=2800, to_value=6500, step=1)]),
        'power_line_frequency': ParameterDescriptor(description='Power line frequency filtering (0:Disabled, 1:50Hz, 2:60Hz)', integer_range=[IntegerRange(from_value=0, to_value=2, step=1)]),
        'backlight_compensation': ParameterDescriptor(description='Backlight compensation [0, 20]', integer_range=[IntegerRange(from_value=0, to_value=20, step=1)])
    }

def declare_camera_parameters(node: Node):
    """Declares and configures all possible ROS parameters for the node and its modules."""
    node.get_logger().info("Declaring parameters...")

    # Get the single source of truth for control descriptors
    control_descriptors = get_camera_control_descriptors()

    # Descriptors for read-only video properties
    frame_id_descriptor = ParameterDescriptor(description='The TF frame ID for the camera images. Read-only after startup.', read_only=True)
    video_id_descriptor = ParameterDescriptor(description='Camera device ID (e.g., /dev/videoX). Read-only after startup.', read_only=True)
    video_width_descriptor = ParameterDescriptor(description='Capture width in pixels. Read-only after startup.', read_only=True)
    video_height_descriptor = ParameterDescriptor(description='Capture height in pixels. Read-only after startup.', read_only=True)
    video_framerate_descriptor = ParameterDescriptor(description='Requested capture framerate (Hz). Read-only after startup.', read_only=True)
    video_format_descriptor = ParameterDescriptor(description='Capture format (e.g., MJPG). Read-only after startup.', read_only=True)
    video_raw_descriptor = ParameterDescriptor(description='Enable publishing to the image_raw topic. Read-only after startup.', read_only=True)
    video_raw_mono_descriptor = ParameterDescriptor(description='Enable publishing to the image_raw topic (grayscale). Read-only after startup.', read_only=True)
    video_raw_framerate_descriptor = ParameterDescriptor(description='Image_raw topic publishing rate. Read-only after startup.', read_only=True)
    video_calibrated_descriptor = ParameterDescriptor(description='Enable publishing to the img_calibrated topic. Read-only after startup.', read_only=True)
    video_calibrated_framerate_descriptor = ParameterDescriptor(description='img_calibrated topic publishing rate. Read-only after startup.', read_only=True)

    # Descriptors for compression settings (read-only after startup)
    compression_width_descriptor = ParameterDescriptor(description='Width for the low-bandwidth compressed stream. Read-only after startup.', read_only=True)
    compression_height_descriptor = ParameterDescriptor(description='Height for the low-bandwidth compressed stream. Read-only after startup.', read_only=True)
    compression_fps_descriptor = ParameterDescriptor(description='Target FPS for the low-bandwidth compressed stream. 0 to disable. Read-only after startup.', floating_point_range=[FloatingPointRange(from_value=0.0, to_value=30.0, step=0.5)])
    jpeg_quality_descriptor = ParameterDescriptor(description='JPEG quality for compressed streams [0, 100].', integer_range=[IntegerRange(from_value=0, to_value=100, step=1)])

    # AprilTag related parameters (all read-only after startup)
    node.declare_parameter('apriltag.enable', False, ParameterDescriptor(description='Enable/disable AprilTag detection.', read_only=True))
    node.declare_parameter('apriltag.family', 'tag36h11', ParameterDescriptor(description='AprilTag family to detect.', read_only=True))
    node.declare_parameter('apriltag.size', 0.16, ParameterDescriptor(description='Size of the AprilTag in meters.', read_only=True))
    node.declare_parameter('apriltag.publish_rate', 1, ParameterDescriptor(description='Rate (Hz) for publishing detection images.', read_only=True))
    node.declare_parameter('apriltag.detector.nthreads', 1, ParameterDescriptor(description='Number of threads for detection.', read_only=True))
    node.declare_parameter('apriltag.detector.quad_decimate', 2.0, ParameterDescriptor(description='Quad decimation factor.', read_only=True))
    node.declare_parameter('apriltag.detector.quad_sigma', 0.0, ParameterDescriptor(description='Gaussian blur sigma for quad detection.', read_only=True))
    node.declare_parameter('apriltag.detector.refine_edges', True, ParameterDescriptor(description='Refine quad edges to subpixel accuracy.', read_only=True))
    node.declare_parameter('apriltag.detector.decode_sharpening', 0.25, ParameterDescriptor(description='Sharpening for decoded tag bits.', read_only=True))

    # Camera Intrinsics (read-only after startup)
    node.declare_parameter('camera.intrinsics.fx', 1000.0, ParameterDescriptor(read_only=True))
    node.declare_parameter('camera.intrinsics.fy', 1000.0, ParameterDescriptor(read_only=True))
    node.declare_parameter('camera.intrinsics.cx', 960.0, ParameterDescriptor(read_only=True))
    node.declare_parameter('camera.intrinsics.cy', 540.0, ParameterDescriptor(read_only=True))
    node.declare_parameter('camera.distortion', [0.0, 0.0, 0.0, 0.0, 0.0], ParameterDescriptor(read_only=True))
    node.declare_parameter('camera.fisheye', False, ParameterDescriptor(description='Set to true if using a fisheye camera model for undistortion.', read_only=True))
    node.declare_parameter('camera.undistort_crop', False, ParameterDescriptor(description='Crop the undistorted image to valid pixels (applies to both models).', read_only=True))

    # Core node parameters
    node.declare_parameter('ros.frame_id', 'dwe_camera_frame', frame_id_descriptor)
    node.declare_parameter('video.id', 2, video_id_descriptor)
    node.declare_parameter('video.width', 1920, video_width_descriptor)
    node.declare_parameter('video.height', 1080, video_height_descriptor)
    node.declare_parameter('video.framerate', 15, video_framerate_descriptor)
    node.declare_parameter('video.format', 'MJPG', video_format_descriptor)
    node.declare_parameter('aux_process.img_raw', False, video_raw_descriptor)
    node.declare_parameter('aux_process.img_raw_mono', False, video_raw_mono_descriptor)
    node.declare_parameter('aux_process.img_raw_framerate', 5, video_raw_framerate_descriptor)
    node.declare_parameter('aux_process.img_calibrated', False, video_calibrated_descriptor)
    node.declare_parameter('aux_process.img_calibrated_framerate', 15, video_calibrated_framerate_descriptor)
    node.declare_parameter('compression.width', 320, compression_width_descriptor)
    node.declare_parameter('compression.height', 240, compression_height_descriptor)
    node.declare_parameter('compression.target_fps', 5, compression_fps_descriptor)
    node.declare_parameter('compression.jpeg_quality', 90, jpeg_quality_descriptor)
    
    # Camera control parameters
    node.declare_parameter('camera.brightness', 0, control_descriptors['brightness'])
    node.declare_parameter('camera.contrast', 32, control_descriptors['contrast'])
    node.declare_parameter('camera.saturation', 64, control_descriptors['saturation'])
    node.declare_parameter('camera.hue', 0, control_descriptors['hue'])
    node.declare_parameter('camera.gamma', 100, control_descriptors['gamma'])
    node.declare_parameter('camera.gain', 0, control_descriptors['gain'])
    node.declare_parameter('camera.sharpness', 3, control_descriptors['sharpness'])
    node.declare_parameter('camera.auto_exposure', True, control_descriptors['auto_exposure'])
    node.declare_parameter('camera.exposure_time', 156, control_descriptors['exposure_time'])
    node.declare_parameter('camera.white_balance_automatic', True, control_descriptors['white_balance_automatic'])
    node.declare_parameter('camera.white_balance_temperature', 4600, control_descriptors['white_balance_temperature'])
    node.declare_parameter('camera.power_line_frequency', 2, control_descriptors['power_line_frequency'])
    node.declare_parameter('camera.backlight_compensation', 5, control_descriptors['backlight_compensation'])