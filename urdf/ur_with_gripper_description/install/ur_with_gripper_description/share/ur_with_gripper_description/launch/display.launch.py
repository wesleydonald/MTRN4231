import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # --- 1. 定位并处理组合的 .xacro 文件 ---
    pkg_path = get_package_share_directory('ur_with_gripper_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'ur_with_gripper.xacro')
    
    # 将 Xacro 转换为 URDF 字符串
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    # --- 2. 准备 RViz 配置文件 ---
    # 我们将重用你之前的 RViz 配置
    rviz_config_pkg_path = get_package_share_directory('ur5e_with_gripper_description')
    rviz_config_file = os.path.join(rviz_config_pkg_path, 'rviz', 'ur5e_with_gripper.rviz')


    # --- 3. 定义所有节点 ---

    # 3.1 robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 3.2 joint_state_publisher (非GUI版本)
    # 这是关键：source_list 必须包含所有可控关节！
    ur5e_joints = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]
    gripper_joint = ['left_claw_joint'] # 你的夹爪的主动关节

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ur5e_joints + gripper_joint
        }]
    )

    # 3.3 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # --- 4. 返回 LaunchDescription ---
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])

