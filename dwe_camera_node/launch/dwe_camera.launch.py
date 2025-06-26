import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Path to the default parameters file
    param_config_path = os.path.join(
        get_package_share_directory('dwe_camera'),
        'config',
        'dwe_camera.yaml'
    )

    # Load the YAML file into a dictionary
    with open(param_config_path, 'r') as f:
        params = yaml.safe_load(f)['ros__parameters']

    # Define the Node action
    node = Node(
        package='dwe_camera',
        executable='dwe_camera_node',
        name='dwe_camera_node',
        namespace="mini_alpha", # The namespace from the original file
        output='screen',
        # Pass the loaded parameters as a dictionary.
        # The launch system will automatically apply them to this node.
        parameters=[params]
    )

    ld = LaunchDescription()
    ld.add_action(node)

    return ld