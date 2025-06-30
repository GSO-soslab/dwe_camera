import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run two instances of the camera_node for two cameras,
    with remappings to ensure topics are unique.
    """
    pkg_share = get_package_share_directory('dwe_camera_driver')

    dual_camera_params_path = os.path.join(pkg_share, 'config', 'dual_cameras.yaml')

    exploreHD_camera_node = Node(
        package='dwe_camera_driver',
        executable='camera_node',
        name='exploreHD_camera_node',
        namespace='mini_alpha',
        output='screen',
        parameters=[dual_camera_params_path],
        remappings=[
            ('image/compressed', 'exploreHD/image/compressed'),
            ('image_lowbw/compressed', 'exploreHD/image_lowbw/compressed'),
            ('camera_settings', 'exploreHD/camera_settings'),
        ]
    )

    usbpcb_camera_node = Node(
        package='dwe_camera_driver',
        executable='camera_node',
        name='usbpcb_camera_node',
        namespace='mini_alpha',
        output='screen',
        parameters=[dual_camera_params_path],
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