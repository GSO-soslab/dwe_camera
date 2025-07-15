import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run two instances of the camera_node for two different cameras.
    Each node loads a combination of shared and specific parameter files.
    """
    pkg_share = get_package_share_directory('dwe_camera_driver')

    # Common hardware settings that can be applied to both cameras
    hardware_config_path = os.path.join(pkg_share, 'config', 'hardware_controls.yaml')
    aux_process_config_path = os.path.join(pkg_share, 'config', 'aux_processes.yaml')

    # --- explore Camera Node Configuration ---
    explore_config_path = os.path.join(pkg_share, 'config', 'cameras', 'in_air', 'explore_air.yaml')
    
    exploreHD_camera_node = Node(
        package='dwe_camera_driver',
        executable='camera_node',
        name='explore_camera_node',
        namespace='mini_alpha',
        output='screen',
        parameters=[
            hardware_config_path,
            aux_process_config_path,
            explore_config_path,
            {'video.id': 0}
        ],
        remappings=[
            ('image/compressed', 'explore/image/compressed'),
            ('image_lowbw/compressed', 'explore/image_lowbw/compressed'),
            ('camera_settings', 'explore/camera_settings'),
        ]
    )

    # --- usbpcb Camera Node Configuration ---
    usbpcb_config_path = os.path.join(pkg_share, 'config', 'cameras', 'in_air', 'usbpcb_air.yaml')
    
    usbpcb_camera_node = Node(
        package='dwe_camera_driver',
        executable='camera_node',
        name='usbpcb_camera_node',
        namespace='mini_alpha',
        output='screen',
        parameters=[
            hardware_config_path,
            aux_process_config_path,
            usbpcb_config_path,
            {'video.id': 4}
        ],
        remappings=[
            ('image/compressed', 'usbpcb/image/compressed'),
            ('image_lowbw/compressed', 'usbpcb/image_lowbw/compressed'),
            ('camera_settings', 'usbpcb/camera_settings'),
        ]
    )

    return LaunchDescription([
        exploreHD_camera_node,
        usbpcb_camera_node
    ])