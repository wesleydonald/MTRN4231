from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  ur_launch_dir = PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch'])
  move_it_launch_dir = PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'launch'])
  return LaunchDescription([
    IncludeLaunchDescription(
      PathJoinSubstitution([ur_launch_dir, 'ur_control.launch.py']),
      launch_arguments={
        'ur_type': 'ur5e', 
        'robot_ip': 'yyy.yyy.yyy.yyy',
        'use_fake_hardware': 'true', 
        'launch_rviz': 'false'
      }.items()
    ),
    IncludeLaunchDescription(
      PathJoinSubstitution([move_it_launch_dir, 'ur_moveit.launch.py']),
      launch_arguments={
        'ur_type': 'ur5e', 
        'use_fake_hardware': 'true', 
        'launch_rviz': 'false'
      }.items()
    )
    # Node(
    #     package='arm',
    #     namespace='',
    #     executable='arm_moveit_server',
    #     name='fake_arm_moveit_server'
    # ),
  ])
