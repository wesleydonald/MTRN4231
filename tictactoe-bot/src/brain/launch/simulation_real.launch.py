import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  # ee_launch_dir = PathJoinSubstitution([FindPackageShare('end_effector'), 'launch'])
  arm_launch_dir = PathJoinSubstitution([FindPackageShare('arm'), 'launch'])

  return LaunchDescription([
    # IncludeLaunchDescription(PathJoinSubstitution([ee_launch_dir, 'display.launch.py'])), 
    IncludeLaunchDescription(PathJoinSubstitution([arm_launch_dir, 'arm.launch.py'])), 
    Node(
        package='visualization',
        namespace='',
        executable='visualization_node',
        name='visualization'
    ),
    Node(
        package='simulation',
        namespace='',
        executable='simulator',
        name='simulation'
    ),
    Node(
        package='brain',
        namespace='',
        executable='brain',
        name='brain'
    ),
    # Node(
    #     name='rviz2',
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     arguments=['-d', os.path.join(get_package_share_directory('brain'), 'rviz_config/rviz.rviz')]
    # ),
  ])