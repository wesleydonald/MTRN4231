from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depth_camera',
            namespace='',
            executable='camera_tf',
            name='camera_tf_publisher'
        ),
    ])
