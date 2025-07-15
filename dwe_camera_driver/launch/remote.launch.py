import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run the remote_node for off-board processing.
    This combines the camera calibration with the remote application settings.
    Note that it does not load hardware_controls.yaml, as the remote node
    does not interface with camera hardware.
    """
    pkg_share = get_package_share_directory('dwe_camera_driver')

    # Paths to the different parameter files
    camera_config_path = os.path.join(pkg_share, 'config', 'cameras', 'in_air', 'stella_air.yaml')
    print(camera_config_path)
    app_config_path = os.path.join(pkg_share, 'config', 'remote.yaml')

    # Define the Node action.
    remote_node = Node(
        package='dwe_camera_driver',
        executable='remote_node',
        name='remote_node',
        namespace="dwe_camera",
        output='screen',
        parameters=[
            camera_config_path,
            app_config_path,
            {'video.id': 2}
        ],
        remappings=[
            # Optional remappings can be placed here if needed
        ]
    )

    return LaunchDescription([
        remote_node
    ])