import rclpy
import cv2
import numpy as np
import pyrealsense2 as rs
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped, PoseArray, Pose
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_geometry_msgs

class ObjectRecognizer(Node):
    def __init__(self):
        super().__init__('object_recognizer')

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        self.roi = [100, 50, 600, 400]  # Format: [x_start, y_start, width, height]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, 10)

        # Publishers
        self.chessboard_pub = self.create_publisher(PointStamped, '~/detected/chessboard', 10)
        self.pieces_pub = self.create_publisher(PoseArray, '~/detected/pieces', 10)
        self.debug_image_pub = self.create_publisher(Image, '~/debug_image', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cv_bridge = CvBridge()
        self.depth_image = None
        self.intrinsics = None
        self.get_logger().info("Object Recognizer node has started.")

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 1.27677
        t.transform.translation.y = 0.0175114
        t.transform.translation.z = 0.673798
        t.transform.rotation.x = -0.414096
        t.transform.rotation.y = -0.019425
        t.transform.rotation.z = 0.910018
        t.transform.rotation.w = 0.00376407
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info("Published static transform from 'base_link' to 'camera_link'.")

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.none
            self.get_logger().info("Camera intrinsics received.")

    def depth_callback(self, msg):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def image_callback(self, msg):
        if self.depth_image is None or self.intrinsics is None:
            return

        cv_image_full = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        x, y, w, h = self.roi
        cv_image = cv_image_full[y:y+h, x:x+w]
        debug_image = cv_image.copy()

        chessboard_center_roi = self.find_chessboard(cv_image, debug_image)

        if chessboard_center_roi is None:
            self.get_logger().warn("No chessboard detected in the ROI.")
            self.debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(debug_image, 'bgr8'))
            return

        pieces_pose_array = PoseArray()
        pieces_pose_array.header.stamp = self.get_clock().now().to_msg()
        pieces_pose_array.header.frame_id = 'base_link'

        self.find_white_pieces(cv_image, debug_image, pieces_pose_array)
        self.find_black_pieces(cv_image, debug_image, pieces_pose_array)

        self.pieces_pub.publish(pieces_pose_array)
        self.get_logger().info(f"Published {len(pieces_pose_array.poses)} pieces.")

        cv2.rectangle(cv_image_full, (x, y), (x+w, y+h), (255, 255, 0), 2)
        cv_image_full[y:y+h, x:x+w] = debug_image
        self.debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image_full, 'bgr8'))

    def find_chessboard(self, image, debug_image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 5000:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(debug_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                center_x, center_y = x + w // 2, y + h // 2
                point_base_link = self.get_3d_point_and_broadcast_tf(center_x, center_y, 'chessboard')
                if point_base_link:
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = 'base_link'
                    point_msg.point = point_base_link
                    self.chessboard_pub.publish(point_msg)
                    
                    coord_text = f"C:({point_base_link.x:.2f},{point_base_link.y:.2f},{point_base_link.z:.2f})"
                    cv2.putText(debug_image, coord_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                return (center_x, center_y)
        return None

    def find_white_pieces(self, image, debug_image, pose_array):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=40)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i, c in enumerate(circles[0, :]):
                center = (c[0], c[1])
                cv2.circle(debug_image, center, c[2], (255, 0, 0), 2)
                point_base_link = self.get_3d_point_and_broadcast_tf(center[0], center[1], f'white_piece_{i}')
                if point_base_link:
                    pose = Pose()
                    pose.position = point_base_link
                    pose.orientation.w = 1.0
                    pose_array.poses.append(pose)
                    coord_text = f"W:({point_base_link.x:.2f},{point_base_link.y:.2f})"
                    cv2.putText(debug_image, coord_text, (c[0] - 30, c[1] - c[2] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

    def find_black_pieces(self, image, debug_image, pose_array):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask = cv2.inRange(hsv, lower_black, upper_black)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            if 100 < cv2.contourArea(cnt) < 2000:
                area, hull_area = cv2.contourArea(cnt), cv2.contourArea(cv2.convexHull(cnt))
                solidity = float(area) / hull_area if hull_area > 0 else 0
                if 0.4 < solidity < 0.7:
                    cv2.drawContours(debug_image, [cnt], -1, (0, 0, 255), 2)
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        point_base_link = self.get_3d_point_and_broadcast_tf(cx, cy, f'black_piece_{i}')
                        if point_base_link:
                            pose = Pose()
                            pose.position = point_base_link
                            pose.orientation.w = 1.0
                            pose_array.poses.append(pose)
                            coord_text = f"B:({point_base_link.x:.2f},{point_base_link.y:.2f})"
                            cv2.putText(debug_image, coord_text, (cx - 30, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    def get_3d_point_and_broadcast_tf(self, u_roi, v_roi, frame_id):
        roi_x, roi_y, _, _ = self.roi
        u_full, v_full = u_roi + roi_x, v_roi + roi_y
        depth = self.depth_image[v_full, u_full]
        if depth == 0:
            return None

        point_3d_camera = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u_full, v_full], depth / 1000.0)

        point_camera = PointStamped()
        point_camera.header.stamp = self.get_clock().now().to_msg()
        point_camera.header.frame_id = 'camera_color_optical_frame'
        point_camera.point.x, point_camera.point.y, point_camera.point.z = point_3d_camera

        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time())
            point_world = tf2_geometry_msgs.do_transform_point(point_camera, transform)
        except Exception as e:
            self.get_logger().error(f"Could not transform point for {frame_id}: {e}")
            return None

        t = TransformStamped()
        t.header.stamp = point_camera.header.stamp
        t.header.frame_id = 'camera_color_optical_frame'
        t.child_frame_id = frame_id
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = point_3d_camera
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        return point_world.point

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

