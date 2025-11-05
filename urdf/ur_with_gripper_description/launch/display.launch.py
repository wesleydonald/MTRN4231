import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
# FIX: Import the ParameterValue class
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # --- 1. Declare the launch argument for 'ur_type' ---
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='Type of UR robot (e.g., ur3, ur3e, ur5, ur5e, ur10, ur10e, ur16e)'
    )

    # --- 2. Find the .xacro file ---
    pkg_path = get_package_share_directory('ur_with_gripper_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'ur_with_gripper.xacro')

    # --- 3. Create the 'robot_description' (URDF) ---
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' ur_type:=', LaunchConfiguration('ur_type')
    ])
    
    # --- 4. -------------------------------
    # FIX: Wrap the command output in ParameterValue(..., value_type=str)
    # This explicitly tells the launch system to treat the output as a string.
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content,
            value_type=str
        )
    }

    # --- 5. -------------------------------
    # We can re-use the rviz config from the gripper
    rviz_config_pkg_path = get_package_share_directory('my_gripper_description')
    rviz_config_file = os.path.join(rviz_config_pkg_path, 'rviz', 'ur5e_with_gripper.rviz')


    # --- 6. Define all nodes ---

    # 6.1 robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # Pass the corrected robot_description dictionary
        parameters=[robot_description] 
    )

    # 6.2 joint_state_publisher (non-GUI)
    ur5e_joints = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]
    gripper_joint = ['left_claw_joint']

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ur5e_joints + gripper_joint
        }]
    )

    # 6.3 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # --- 7. Return LaunchDescription ---
    return LaunchDescription([
        ur_type_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])

