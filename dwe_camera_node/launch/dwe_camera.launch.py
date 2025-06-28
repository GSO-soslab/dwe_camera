import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file to run a single instance of the dwe_camera_node.
    It follows the same parameter loading style as the multi-camera launch file.
    """
    dwe_camera_pkg_share = get_package_share_directory('dwe_camera')

    # Path to the parameters file
    param_config_path = os.path.join(
        dwe_camera_pkg_share,
        'config',
        'dwe_camera.yaml'
    )

    # Define the Node action. The 'name' and 'namespace' must match the
    # top-level key in the dwe_camera.yaml file.
    camera_node = Node(
        package='dwe_camera',
        executable='dwe_camera_node',
        name='dwe_camera_node',
        namespace="mini_alpha", # This matches the key in the YAML file
        output='screen',
        # Pass the file path directly. ROS 2 will handle loading and
        # applying the correct parameters based on the node's full name.
        parameters=[param_config_path]
    )

    return LaunchDescription([
        camera_node
    ])