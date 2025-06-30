import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run a single instance of the camera_node.
    """
    pkg_share = get_package_share_directory('dwe_camera_driver')

    # Path to the parameters file
    param_config_path = os.path.join(
        pkg_share,
        'config',
        'camera.yaml'
    )

    # Define the Node action. The 'name' and 'namespace' must match the
    # top-level key in the YAML file.
    camera_node = Node(
        package='dwe_camera_driver',
        executable='camera_node',
        name='camera_node',
        namespace="mini_alpha", # This matches the key in the YAML file
        output='screen',
        parameters=[param_config_path]
    )

    return LaunchDescription([
        camera_node
    ])