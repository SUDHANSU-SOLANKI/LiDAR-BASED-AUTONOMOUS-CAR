import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory for your package
    pkg_dir = get_package_share_directory('my_robot_bringup')
    
    # Define the path to the YAML file
    twist_mux_params_file = os.path.join(
        pkg_dir,
        'config',
        'twist_mux.yaml'
    )

    # Create the twist_mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_params_file]
    )

    # Return the launch description
    return LaunchDescription([
        twist_mux_node
    ])