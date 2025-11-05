import rclpy
import cv2
import tf2_ros
import os
import numpy as np
import pyrealsense2 as rs
import tf2_geometry_msgs
from collections import Counter # Import Counter for finding most common detection
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, PoseArray, Point
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

class ObjectRecognizer(Node):
    def __init__(self):
        super().__init__('object_recognizer')

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        self.roi = [250, 50, 300, 400]  # Format: [x_start, y_start, width, height]

        # Tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.image_sub = self.create_subscription( Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.point_cloud_sub = self.create_subscription( Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.cam_info_sub = self.create_subscription( CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback,10)

        # Publishers
        self.debug_image_pub = self.create_publisher(Image, '/debug/image', 10)
        self.debug_black_and_white_pub = self.create_publisher(Image, '/debug/black_and_white', 10)
        self.board_marker_pub = self.create_publisher(Marker, '/detected/board_marker', 10)
        self.pieces_marker_pub = self.create_publisher(MarkerArray, '/detected/pieces_markers', 10)

        # Update the position of the broadcasted markers
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cv_bridge = CvBridge()
        self.depth_image = None
        self.intrinsics = None
        
        # --- Modified state to store white and black pieces separately ---
        self.detected_pieces_all = [] # Will store tuples of (white_pose_array, black_pose_array)
        # --- End Modified state ---
        
        self.get_logger().info("Object Recognizer node has started.")

    def timer_callback(self):
        # --- Updated logic to find the most stable detection count ---
        if not self.detected_pieces_all:
            # Publish empty arrays to clear RViz if no detections
            self.publish_piece_markers(PoseArray(), PoseArray())
            # self.pieces_pub.publish(PoseArray())
            self.get_logger().info(f"Published no pieces (no detections).")
            self.detected_pieces_all = [] # Clear list
            return

        # Find the most common *total* number of pieces detected
        lengths = [len(wp.poses) + len(bp.poses) for wp, bp in self.detected_pieces_all]
        if not lengths:
             self.detected_pieces_all = [] # Clear list
             return
             
        length_counts = Counter(lengths)
        most_common_length = length_counts.most_common(1)[0][0]

        best_white_pa = None
        best_black_pa = None

        # Find the first detection set that matches the most common count
        for wp, bp in self.detected_pieces_all:
            if len(wp.poses) + len(bp.poses) == most_common_length:
                best_white_pa = wp
                best_black_pa = bp
                break

        if best_white_pa is not None and best_black_pa is not None:
            # Create a combined PoseArray for the original topic
            combined_pa = PoseArray()
            combined_pa.header.stamp = self.get_clock().now().to_msg()
            combined_pa.header.frame_id = 'base_link'
            combined_pa.poses.extend(best_white_pa.poses)
            combined_pa.poses.extend(best_black_pa.poses)
            
            # self.pieces_pub.publish(combined_pa)
            
            # Publish markers for RViz
            self.publish_piece_markers(best_white_pa, best_black_pa)
            
            self.get_logger().info(f"Published {most_common_length} pieces.")
        else:
            # This case is unlikely if list wasn't empty, but good to handle
            self.publish_piece_markers(PoseArray(), PoseArray())
            # self.pieces_pub.publish(PoseArray())
            self.get_logger().info(f"Published no pieces (no best_pose_array found).")

        self.detected_pieces_all = [] # Clear list for next cycle
        # --- End Updated logic ---

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

    def info_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return

    # This gets depth_frame aligned with RGB image
    def depth_callback(self, msg):
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, msg.encoding)
                
        except Exception as e:
            self.get_logger().error(f"Error in point_cloud_callback: {str(e)}")


    def pixel_2_global(self, pixel_pt):
        if self.depth_image is not None and self.intrinsics is not None:
            [x,y,z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, (pixel_pt[0],pixel_pt[1] ), self.depth_image[pixel_pt[0],pixel_pt[1] ]*0.001)
            return [x, y,z]
        else:
            return None

    def image_callback(self, msg):
        if self.depth_image is None or self.intrinsics is None:
            return

        cv_image_full = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        x, y, w, h = self.roi
        cv_image = cv_image_full[y:y+h, x:x+w]
        debug_image = cv_image.copy()

        board_center_roi = self.find_board(cv_image, debug_image)

        if board_center_roi is None:
            self.get_logger().warn("No board detected in the ROI.")
            self.debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(debug_image, 'bgr8'))
            return

        # --- Create separate PoseArrays for white and black pieces ---
        current_stamp = self.get_clock().now().to_msg()
        
        white_pieces_pa = PoseArray()
        white_pieces_pa.header.stamp = current_stamp
        white_pieces_pa.header.frame_id = 'base_link' # Poses are in base_link frame
        
        black_pieces_pa = PoseArray()
        black_pieces_pa.header.stamp = current_stamp
        black_pieces_pa.header.frame_id = 'base_link' # Poses are in base_link frame

        self.find_white_pieces(cv_image, debug_image, white_pieces_pa)
        self.find_black_pieces(cv_image, debug_image, black_pieces_pa)
        # --- End PoseArray separation ---

        # --- Store detections for timer callback to process ---
        self.detected_pieces_all.append((white_pieces_pa, black_pieces_pa))
        # --- End storage ---

        cv2.rectangle(cv_image_full, (x, y), (x+w, y+h), (255, 255, 0), 2)
        cv_image_full[y:y+h, x:x+w] = debug_image
        self.debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_image_full, 'bgr8'))

    def find_board(self, image, debug_image):
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
                point_base_link = self.get_3d_point_and_broadcast_tf(center_x, center_y, 'board')
                if point_base_link:
                    # --- Publish board Marker ---
                    marker_msg = Marker()
                    marker_msg.header.frame_id = 'base_link'
                    marker_msg.header.stamp = self.get_clock().now().to_msg()
                    marker_msg.ns = "board"
                    marker_msg.id = 0
                    marker_msg.type = Marker.CUBE
                    marker_msg.action = Marker.ADD
                    marker_msg.pose.position = point_base_link
                    marker_msg.pose.orientation.x = 0.0
                    marker_msg.pose.orientation.y = 0.0
                    marker_msg.pose.orientation.z = 0.0
                    marker_msg.pose.orientation.w = 1.0
                    marker_msg.scale.x = 0.3 # 30cm
                    marker_msg.scale.y = 0.3 # 30cm
                    marker_msg.scale.z = 0.01 # 1cm thick
                    marker_msg.color.r = 0.0
                    marker_msg.color.g = 1.0
                    marker_msg.color.b = 0.0
                    marker_msg.color.a = 0.5 # Semi-transparent
                    self.board_marker_pub.publish(marker_msg)
                    # --- End Marker Publish ---
                    
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
        upper_black = np.array([179, 255, 85])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        # Debugging
        self.debug_black_and_white_pub.publish(self.cv_bridge.cv2_to_imgmsg(mask, 'mono8'))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt) # Get area first
            if 100 < area < 2000: # Check area
                hull = cv2.convexHull(cnt) # Get convex hull
                hull_area = float(cv2.contourArea(hull)) # Explicitly cast hull area to float

                solidity = 0.0 # Initialize solidity
                if hull_area > 0: # Now this comparison should be safe
                    solidity = float(area) / hull_area
                
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

    # --- New Helper Function to Publish Piece Markers ---
    def publish_piece_markers(self, white_pa, black_pa):
        marker_array = MarkerArray()
        current_stamp = self.get_clock().now().to_msg()
        
        # Add a DELETEALL marker for white pieces to clear old ones
        delete_marker_white = Marker()
        delete_marker_white.header.frame_id = 'base_link'
        delete_marker_white.header.stamp = current_stamp
        delete_marker_white.ns = "white_pieces"
        delete_marker_white.id = 0 # ID 0 for deleteall
        delete_marker_white.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker_white)
        
        # Add a DELETEALL marker for black pieces to clear old ones
        delete_marker_black = Marker()
        delete_marker_black.header.frame_id = 'base_link'
        delete_marker_black.header.stamp = current_stamp
        delete_marker_black.ns = "black_pieces"
        delete_marker_black.id = 0 # ID 0 for deleteall
        delete_marker_black.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker_black)

        # Add white piece markers
        for i, pose in enumerate(white_pa.poses):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = current_stamp
            marker.ns = "white_pieces"
            marker.id = i + 1 # Start IDs from 1 to avoid conflict with deleteall
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.03 # 3cm diameter
            marker.scale.y = 0.03 # 3cm diameter
            marker.scale.z = 0.05 # 5cm height
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

        # Add black piece markers
        offset_id = len(white_pa.poses) + 1
        for i, pose in enumerate(black_pa.poses):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = current_stamp
            marker.ns = "black_pieces"
            marker.id = i + offset_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.03 # 3cm diameter
            marker.scale.y = 0.03 # 3cm diameter
            marker.scale.z = 0.05 # 5cm height
            marker.color.r = 0.1
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 1.0
            marker_array.markers.append(marker)
            
        self.pieces_marker_pub.publish(marker_array)
    # --- End New Helper Function ---

    def get_3d_point_and_broadcast_tf(self, u_roi, v_roi, frame_id):
        roi_x, roi_y, _, _ = self.roi
        u_full, v_full = u_roi + roi_x, v_roi + roi_y
        
        # Check image bounds
        if not (0 <= v_full < self.depth_image.shape[0] and 0 <= u_full < self.depth_image.shape[1]):
            self.get_logger().warn(f"Pixel ({u_full}, {v_full}) for {frame_id} out of bounds.")
            return None
            
        depth = self.depth_image[v_full, u_full]
        if depth == 0:
            self.get_logger().warn(f"Zero depth for {frame_id} at pixel ({u_full}, {v_full}).")
            return None

        point_3d_camera = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u_full, v_full], depth / 1000.0)

        point_camera = PointStamped()
        point_camera.header.stamp = self.get_clock().now().to_msg()
        point_camera.header.frame_id = 'camera_color_optical_frame'
        point_camera.point.x = point_3d_camera[0]
        point_camera.point.y = point_3d_camera[1]
        point_camera.point.z = point_3d_camera[2] - 0.06 # Offset for the z (since it seems to not be calibrated 100% correctly)

        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

