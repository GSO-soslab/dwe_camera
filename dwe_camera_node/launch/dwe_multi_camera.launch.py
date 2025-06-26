import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run two instances of the dwe_camera_node for two cameras,
    with remappings to ensure topics are unique.
    """
    # The package name is 'dwe_camera' as defined in setup.py
    dwe_camera_pkg_share = get_package_share_directory('dwe_camera')

    dual_camera_params_path = os.path.join(dwe_camera_pkg_share, 'config', 'dual_cameras.yaml')

    exploreHD_camera_node = Node(
        package='dwe_camera',
        executable='dwe_camera_node',
        name='exploreHD_camera_node',
        namespace='dwe_camera',
        output='screen',
        parameters=[dual_camera_params_path],
        remappings=[
            ('image', 'exploreHD/image'),
            ('image_lowbw', 'exploreHD/image_lowbw'),
            ('camera_settings', 'exploreHD/camera_settings'),
        ]
    )

    usbpcb_camera_node = Node(
        package='dwe_camera',
        executable='dwe_camera_node',
        name='usbpcb_camera_node',
        namespace='dwe_camera',
        output='screen',
        parameters=[dual_camera_params_path],
        remappings=[
            ('image', 'usbpcb/image'),
            ('image_lowbw', 'usbpcb/image_lowbw'),
            ('camera_settings', 'usbpcb/camera_settings'),
        ]
    )

    return LaunchDescription([
        exploreHD_camera_node,
        usbpcb_camera_node
    ])