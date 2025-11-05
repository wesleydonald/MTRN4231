#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Mesh
from ament_index_python.packages import get_package_share_directory
import os, time

class GripperAttacher(Node):
    def __init__(self):
        super().__init__('gripper_attacher')

        # === 1. CONFIGURATION ===
        self.attach_link = 'tool0' 
        self.gripper_id = 'my_gripper_mesh'

        # --- Find the path to the STL file ---
        try:
            package_share_dir = get_package_share_directory('visualization')
            self.stl_file_path = os.path.join(package_share_dir, 'meshes', 'my_gripper.stl')

            if not os.path.exists(self.stl_file_path):
                self.get_logger().error(f"STL file not found at: {self.stl_file_path}")
                raise FileNotFoundError()
        except Exception as e:
            self.get_logger().error(f"Failed to find package or STL: {e}")
            rclpy.shutdown()
            return

        # === 2. Internal Setup ===
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        while not self.scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')

        self.get_logger().info('Service found. Attaching gripper...')
        self.attach_gripper()

    def attach_gripper(self):
        collision_object = CollisionObject()
        collision_object.id = self.gripper_id
        collision_object.header.frame_id = self.attach_link

        # --- Define the mesh ---
        mesh = Mesh()
        mesh.mesh_source.resource_uri = f'file://{self.stl_file_path}'
        collision_object.meshes.append(mesh)

        # --- Define the pose of the mesh relative to the attach_link ---
        mesh_pose = PoseStamped()
        mesh_pose.header.frame_id = self.attach_link
        mesh_pose.pose.orientation.w = 1.0 # No rotation relative to tool0
        mesh_pose.pose.position.x = 0.0 # Adjust as needed
        mesh_pose.pose.position.y = 0.0
        mesh_pose.pose.position.z = 0.0 # e.g., 0.05 to move 5cm "forward"
        collision_object.mesh_poses.append(mesh_pose.pose)

        collision_object.operation = CollisionObject.ADD

        # --- Create the request ---
        scene_request = ApplyPlanningScene.Request()
        scene = scene_request.scene
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects.append(collision_object)
        scene.robot_state.attached_collision_objects.append(
            {
                'link_name': self.attach_link,
                'object': collision_object
            }
        )

        future = self.scene_client.call_async(scene_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info(f"Successfully attached '{self.gripper_id}' to '{self.attach_link}'.")
        else:
            self.get_logger().error("Failed to apply planning scene.")

def main(args=None):
    rclpy.init(args=args)
    node = GripperAttacher()
    try:
        # We spin once to let the service call complete, then shutdown
        rclpy.spin_once(node, timeout_sec=5.0) 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()