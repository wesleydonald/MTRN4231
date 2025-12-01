import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_path = PathJoinSubstitution([FindPackageShare('end_effector'), "urdf", 'ur_with_gripper.xacro'])

    ur_launch_dir = PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch'])
    move_it_launch_dir = PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch'])

    ur_control_launch = IncludeLaunchDescription(
     PathJoinSubstitution([ur_launch_dir, 'ur_control.launch.py']),
     launch_arguments={
       'ur_type': 'ur5e', 
       'robot_ip': '192.168.0.100',
       'use_fake_hardware': 'true', 
       'launch_rviz': 'false',
       'description_file': urdf_file_path
     }.items()
    )

    moveit_launch = IncludeLaunchDescription(
     PathJoinSubstitution([move_it_launch_dir, 'ur_moveit.launch.py']),
     launch_arguments={
       'ur_type': 'ur5e', 
       'robot_ip': '192.168.0.100',
       'use_fake_hardware': 'true',
       'launch_rviz': 'false'
     }.items()
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file_path])}]
    )

    return LaunchDescription([
        ur_control_launch,
        robot_state_pub_node,
        TimerAction(
            period=10.0,
            actions=[moveit_launch]
        )
    ])

