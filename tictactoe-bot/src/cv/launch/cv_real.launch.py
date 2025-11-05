from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  realsense_dir = PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch'])
  return LaunchDescription([
    IncludeLaunchDescription(
      PathJoinSubstitution([realsense_dir, 'rs_launch.py']),
      launch_arguments={
        'align_depth.enable': 'true', 
        'enable_color': 'true',
        'enable_depth': 'true'
      }.items()
    ),
    Node(
        package='computer_vision',
        namespace='',
        executable='object_recognizer',
        name='cv'
    ),
  ])
