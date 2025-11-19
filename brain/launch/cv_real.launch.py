from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  ur_launch_dir = PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch'])
  move_it_launch_dir = PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch'])
  arm_launch_dir = PathJoinSubstitution([FindPackageShare('arm'), 'launch'])
  cv_launch_dir = PathJoinSubstitution([FindPackageShare('cv'), 'launch'])
  return LaunchDescription([
    #IncludeLaunchDescription(
    #  PathJoinSubstitution([ur_launch_dir, 'ur_control.launch.py']),
    #  launch_arguments={
    #    'ur_type': 'ur5e', 
    #    'robot_ip': '192.168.0.100',
    #    'use_fake_hardware': 'false', 
    #    'launch_rviz': 'false'
    #  }.items()
    #),
    #IncludeLaunchDescription(
    #  PathJoinSubstitution([move_it_launch_dir, 'ur_moveit.launch.py']),
    #  launch_arguments={
    #    'ur_type': 'ur5e', 
    #    'robot_ip': '192.168.0.100',
    #    'launch_rviz': 'false'
    #  }.items()
    #),
    IncludeLaunchDescription(PathJoinSubstitution([arm_launch_dir, 'arm.launch.py'])), 
    IncludeLaunchDescription(PathJoinSubstitution([cv_launch_dir, 'cv_real.launch.py'])), 
    Node(
        package='visualization',
        namespace='',
        executable='visualization_node',
        name='visualization'
    ),
    Node(
        package='brain',
        namespace='',
        executable='brain',
        name='brain'
    ),
    Node(
        package='end_effector',
        namespace='',
        executable='gripper',
        name='gripper'
    )
  ])