import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Setup paths and configurations
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    
    xacro_file = os.path.join(pkg_description, 'urdf', 'my_robot_main.urdf.xacro')
    controller_config = os.path.join(pkg_bringup, 'config', 'my_controller.yaml')
    rviz_config = os.path.join(pkg_description, 'rviz', 'rviz_config.rviz')
    gazebo_bridge_config = os.path.join(pkg_bringup, 'config', 'gazebo_bridge.yaml')

    # 2. Declare Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 3. Process URDF (Shared by both)
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_param = {'robot_description': robot_description_config.toxml()}

    # 4. SHARED NODE: Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )

    # robot_state_publisher_gui = Node(
    #         package="joint_state_publisher_gui",
    #         executable="joint_state_publisher_gui",
    #         name="joint_state_publisher"
    # ),

    # 5. REAL HARDWARE PATH: ros2_control_node (Runs if use_sim_time is false)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_param, controller_config],
        condition=UnlessCondition(use_sim_time),
        output="screen",
    )

    # 6. SIMULATION PATH: Gazebo (Runs if use_sim_time is true)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
        condition=IfCondition(use_sim_time)
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"],
        condition=IfCondition(use_sim_time)
    )

    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[gazebo_bridge_config],
        condition=IfCondition(use_sim_time)
    )

    # 7. CONTROLLER SPAWNERS (Shared)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    # 8. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        # robot_state_publisher_gui,
        control_node,
        gazebo_sim,
        spawn_robot,
        gazebo_bridge,
        jsb_spawner,
        diff_drive_spawner,
        rviz_node
    ])