from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    ## path of urdf file
    urdf_path = os.path.join(get_package_share_path('my_robot_description'),
                             "urdf", 'my_robot_main.urdf.xacro')
    
    ## path of .rviz file
    rviz_config_path= os.path.join(get_package_share_path('my_robot_description'),
                                   'rviz', 'empty.rviz')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description'  : robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        ## adding arguments to node and opening .rviz file with it
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])




## For ros2 control where we have to spawn controllers
## Intial file of ros2 control

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# import xacro

# def generate_launch_description():
#     # 1. Path to your xacro file
#     pkg_path = get_package_share_directory('my_robot_description')
#     xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot_main.urdf.xacro')

#     # 2. Process xacro
#     robot_description_config = xacro.process_file(xacro_file)
#     # This is a dictionary: {'robot_description': '<xml...'}
#     robot_description_param = {'robot_description': robot_description_config.toxml()}

#     # 3. Path to your controller config
#     config_pkg_path = get_package_share_directory('my_robot_bringup')
#     controller_config = os.path.join(config_pkg_path, 'config', 'my_controller.yaml')

#     # 4. Robot State Publisher (Fixed the parameters line)
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         parameters=[robot_description_param] # Removed the extra curly braces
#     )

#     # 5. The ros2_control_node
#     control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[robot_description_param, controller_config],
#         output="screen",
#     )

#     # 6. Spawners
#     jsb_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_state_broadcaster"],
#     )

#     diff_drive_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["diff_drive_controller"],
#     )

#     return LaunchDescription([
#         robot_state_publisher_node,
#         control_node,
#         jsb_spawner,
#         diff_drive_spawner
#     ])