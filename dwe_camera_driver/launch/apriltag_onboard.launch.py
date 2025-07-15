import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run a single camera_node with on-board AprilTag detection.
    This launch file combines three parameter files:
    1. hardware_controls.yaml: General V4L2 camera settings.
    2. stella_air.yaml: The specific camera's calibration and video format.
    3. apriltag_detection.yaml: The application-specific settings (enables apriltags).
    The files are loaded in order, with later files overwriting earlier ones.
    """
    pkg_share = get_package_share_directory('dwe_camera_driver')

    # Paths to the different parameter files
    hardware_config_path = os.path.join(pkg_share, 'config', 'hardware_controls.yaml')
    camera_config_path = os.path.join(pkg_share, 'config', 'camera_parameters', 'stella_cam.yaml')
    aux_process_config_path = os.path.join(pkg_share, 'config', 'auxiliary_processors.yaml')

    # Define the Node action.
    camera_node = Node(
        package='dwe_camera_driver',
        executable='camera_node',
        name='camera_node',
        namespace="dwe_camera",
        output='screen',
        parameters=[
            hardware_config_path,
            camera_config_path,
            aux_process_config_path,
            # You can add overrides here, for example:
            {'apriltag.enable': True}
        ],
        remappings=[
            ('apriltag_detection/compressed', 'apriltag_detection/image/compressed'),
        ]
    )

    return LaunchDescription([
        camera_node
    ])