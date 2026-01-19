from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Paths
    urdf_file = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "urdf",
        "my_robot_main.urdf.xacro"
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("my_robot_description"),
        "rviz",
        "rviz_config.rviz"
    ])

    gazebo_config_path = PathJoinSubstitution([
        FindPackageShare("my_robot_bringup"),
        "config",
        "gazebo_bridge.yaml"
    ])

    gz_sim_launch = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py"
    ])

    # LaunchDescription object
    return LaunchDescription([
        ## this gui code was causing problem in rviz 
        # # Joint state publisher GUI (for TF to be published)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher"
        ),

        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": Command(["xacro ", urdf_file])
            }]
        ),

        # Gazebo simulator
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    "gz_args": TextSubstitution(text="empty.sdf -r")
                }.items()

        ),

        # Spawn robot in Gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-topic", "robot_description"],
            name="spawn_entity"
        ),

        # ## Node for bridge between Ros and Gazebo
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="config_file",
            parameters=[gazebo_config_path]
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config]
        ),
    ])
