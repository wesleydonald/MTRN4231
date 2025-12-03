import rclpy
import cv2
import time
import math
import numpy as np
import pyrealsense2 as rs
import tf2_geometry_msgs
import tf_transformations
from collections import Counter # Import Counter for finding most common detection
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PointStamped, Pose, PoseArray, Point
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from interfaces.msg import BoardPose

# CONSTANTS
BLACK_PIECE_MIN_AREA = 100
BLACK_PIECE_MAX_AREA = 1500
BLACK_PIECE_MIN_SOLIDITY = 0.25
BLACK_PIECE_ERODE_KERNEL_SIZE = 3
BLACK_PIECE_DILATE_KERNEL_SIZE = 7
BLACK_PIECE_BLUR_SIZE = 7
BLACK_LOWER = np.array([80, 50, 0])
BLACK_UPPER = np.array([120, 170, 130])

WHITE_PIECE_MIN_AREA = 150
WHITE_PIECE_MAX_AREA = 2500
WHITE_PIECE_MIN_SOLIDITY = 0.85
WHITE_DILATION_KERNEL_SIZE = 5
WHITE_ERODE_KERNEL_SIZE = 5
WHITE_LOWER = np.array([70, 70, 200])
WHITE_UPPER = np.array([120, 150, 255])

BOARD_COLOUR_THRESHOLD_MAX = 80
BOARD_COLOUR_THRESHOLD_MIN = 0
BOARD_ERODE_KERNEL_SIZE = 1
BOARD_DILATE_KERNEL_SIZE = 3
BOARD_MEDIAN_BLUR = 3
BOARD_GAUSSIAN_BLUR = 3
BOARD_MIN_AREA = 2500
BOARD_LOWER = np.array([80, 200, 90])
BOARD_UPPER = np.array([120, 255, 180])

ROI_TOP = 0.15
ROI_BOTTOM = 0.8
ROI_LEFT_TOP = 0.4
ROI_LEFT_BOTTOM = 0.3
ROI_RIGHT_TOP = 0.98
ROI_RIGHT_BOTTOM = 1

# List of grid indexes / offsets to help calculate grid square positions
GRID_IDEXES = [[-1, -1], [-1,0], [-1, 1], [0,-1], [0, 0], [0, 1], [1, -1], [1,0], [1,1]]
CELL_SIZE = 0.11
BOARD_POSE_ROLLING_AVERAGE_LEN = 10

class PoseRollingAverage():
    def __init__(self, size):
        self.size = size
        self.poses = []
        self.avg_pose = Point()
        self.avg_orientation = 0.0

    def updatePose(self, point, orientation):
        # Add newest pose to rolling average, returns the rolling average (point, orientation)
        if len(self.poses) > self.size:
            # Remove oldest pose
            self.poses.pop(0)

        # Append the pose
        self.poses.append([point, orientation])

        # Calculate the average and return it
        avg_p = Point()
        avg_o = 0.0
        avg_len = len(self.poses)
        for pose in self.poses:
            if pose is None or pose[0] is None:
                continue
            avg_p.x += (pose[0].x / avg_len)
            avg_p.y += (pose[0].y / avg_len)
            avg_p.z += (pose[0].z / avg_len)
            avg_o += (pose[1] / avg_len)

        self.avg_pose = avg_p
        self.avg_orientation = avg_o

        return avg_p, avg_o

    def getCurrentPose(self):
        return self.avg_pose, self.avg_orientation

class ObjectRecognizer(Node):
    def __init__(self):
        super().__init__('object_recognizer')

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # self.roi = [250, 50, 300, 400]  # Format: [x_start, y_start, width, height]

        # Tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers (depth camera info)
        self.image_sub = self.create_subscription( Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.point_cloud_sub = self.create_subscription( Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.cam_info_sub = self.create_subscription( CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback,10)

        # Publishers (debug images)
        self.debug_image_pub = self.create_publisher(Image, '/debug/detection_image', 10)
        self.debug_black_piece_pub = self.create_publisher(Image, '/debug/black_piece_detection', 10)
        self.debug_white_piece_pub = self.create_publisher(Image, '/debug/white_piece_detection', 10)
        self.debug_board_pub = self.create_publisher(Image, '/debug/board_detection', 10)
        # (object positions)
        self.board_pose_pub = self.create_publisher(BoardPose, '/detected/board', 10)
        self.white_pieces_pose_array_pub = self.create_publisher(PoseArray, '/detected/pieces/white', 10)
        self.black_pieces_pose_array_pub = self.create_publisher(PoseArray, '/detected/pieces/black', 10)

        # Update the position of the broadcasted markers
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cv_bridge = CvBridge()
        self.depth_image = None
        self.intrinsics = None
        
        # --- Modified state to store white and black pieces separately ---
        self.detected_pieces_all = [] # Will store tuples of (white_pose_array, black_pose_array)
        self.boardPoseRollingAverage = PoseRollingAverage(BOARD_POSE_ROLLING_AVERAGE_LEN)
        # --- End Modified state ---
        
        self.get_logger().info("Object Recognizer node has started.")

    def timer_callback(self):
        # --- Updated logic to find the most stable detection count ---
        if not self.detected_pieces_all:
            # Publish empty arrays to clear RViz if no detections
            self.white_pieces_pose_array_pub.publish(PoseArray())
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
            # Publish white pieces pose array
            white_pieces_pose_array = PoseArray()
            white_pieces_pose_array.header.stamp = self.get_clock().now().to_msg()
            white_pieces_pose_array.header.frame_id = 'base_link'
            white_pieces_pose_array.poses.extend(best_white_pa.poses)
            self.white_pieces_pose_array_pub.publish(white_pieces_pose_array)

            # Publish black pieces pose array
            black_pieces_pose_array = PoseArray()
            black_pieces_pose_array.header.stamp = self.get_clock().now().to_msg()
            black_pieces_pose_array.header.frame_id = 'base_link'
            black_pieces_pose_array.poses.extend(best_black_pa.poses)
            self.black_pieces_pose_array_pub.publish(black_pieces_pose_array)
            
            self.get_logger().info(f"Published {most_common_length} pieces.")
        else:
            # This case is unlikely if list wasn't empty, but good to handle
            self.white_pieces_pose_array_pub.publish(PoseArray())
            self.black_pieces_pose_array_pub.publish(PoseArray())
            self.get_logger().info(f"Published no pieces (no best_pose_array found).")

        self.detected_pieces_all = [] # Clear list for next cycle
        # --- End Updated logic ---

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 1.30938 # 1.27677
        t.transform.translation.y = 0.0206053 # 0.0175114
        t.transform.translation.z = 0.670571 # 0.673798
        t.transform.rotation.x = -0.398486 #-0.414096
        t.transform.rotation.y = 0.00254305 #-0.019425
        t.transform.rotation.z = 0.917119 # 0.910018
        t.transform.rotation.w = 0.00974536 # 0.00376407
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


    def image_callback(self, msg):
        if self.depth_image is None or self.intrinsics is None:
            return

        # Image from the depth camera
        camera_img = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        img_height, img_width = camera_img.shape[:2]

        # Only detect items within the ROI (calibrated to be the table space),
        # the mask is passed to the detection functions as it needs to be applied after 
        # colour thresholding to work properly.
        trapezoid_vertices = np.array([
            [(int(img_width * ROI_LEFT_BOTTOM), img_height * ROI_BOTTOM),     # Bottom-left
            (int(img_width * ROI_LEFT_TOP), int(img_height * ROI_TOP)), # Top-left
            (int(img_width * ROI_RIGHT_TOP), int(img_height * ROI_TOP)), # Top-right
            (int(img_width * ROI_RIGHT_BOTTOM), img_height * ROI_BOTTOM)]    # Bottom-right
        ], dtype=np.int32)
        roi_mask = np.zeros((img_height, img_width), dtype=np.uint8)
        cv2.fillPoly(roi_mask, trapezoid_vertices, 255)

        debug_image = camera_img.copy()

        board_center_roi = self.find_board(camera_img, roi_mask, debug_image)

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

        self.find_white_pieces(camera_img, roi_mask, debug_image, white_pieces_pa)
        self.find_black_pieces(camera_img, roi_mask, debug_image, black_pieces_pa)
        # --- End PoseArray separation ---

        # --- Store detections for timer callback to process ---
        self.detected_pieces_all.append((white_pieces_pa, black_pieces_pa))
        # --- End storage ---

        # Draw the polygon outline
        # Arguments: image, vertices, closed=True, color=(Blue, Green, Red), thickness=2
        cv2.polylines(
            debug_image,
            trapezoid_vertices,
            isClosed=True,
            color=(0, 255, 0),  # Green BGR color
            thickness=4
        )

        # cv2.rectangle(cv_image_full, (x, y), (x+w, y+h), (255, 255, 0), 2)
        # cv_image_full[y:y+h, x:x+w] = debug_image
        self.debug_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(debug_image, 'bgr8'))

    def find_board(self, camera_img, roi_mask, debug_image):

        # Threshold to detect black
        cv_img = cv2.cvtColor(camera_img, cv2.COLOR_BGR2HSV)
        cv_img = cv2.inRange(cv_img, BOARD_LOWER, BOARD_UPPER)

        # Apply ROI to make area outside ROI black
        cv_img = cv2.bitwise_and(cv_img, cv_img, mask=roi_mask)

        # Image processing to remove noise and smooth
        kernel = np.ones((BOARD_DILATE_KERNEL_SIZE, BOARD_DILATE_KERNEL_SIZE), np.uint8)
        cv_img = cv2.dilate(cv_img, kernel, iterations=1)

        for i in range(1, 5):
            cv_img = cv2.GaussianBlur(cv_img, (BOARD_GAUSSIAN_BLUR, BOARD_GAUSSIAN_BLUR), 1)
            cv_img = cv2.medianBlur(cv_img, BOARD_MEDIAN_BLUR)

        # Edge / contour detection
        edges = cv2.Canny(cv_img, 50, 150, apertureSize=3)
        # Dilate the canny edge detection image too so contours work better
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        # Contours for size detection
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Line detection for orientation
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 25, minLineLength=50, maxLineGap=30)

        # Detection debugging image for the board with contours and orientation line
        bgr_image = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(bgr_image, contours, -1, (0, 255, 0), 2)

        # Draw the detected lines and get longest line
        if lines is None:
            return None

        max_length = 0
        longest_line = None
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if length > max_length:
                max_length = length
                longest_line = (x1, y1, x2, y2)

        # Determine orientation from the longest line
        x1_px, y1_px, x2_px, y2_px = longest_line

        # Display the longeest line on the debug image
        cv2.line(bgr_image, (x1_px, y1_px), (x2_px, y2_px), (0, 0, 255), 2)

        # Debugging image
        self.debug_board_pub.publish(self.cv_bridge.cv2_to_imgmsg(bgr_image, 'bgr8'))

        # Calculate the convex hull area for each contour (min sized convex polygon)
        # to find the largest one, which should be the board
        if not contours:
            return None

        hull_sizes = []
        for cnt in contours:
            hull = cv2.convexHull(cnt) # Get convex hull
            hull_area = float(cv2.contourArea(hull)) # Explicitly cast hull area to float
            hull_sizes.append(hull_area) 

        # Get the largest convex hull contour
        largest_hull_cnt = contours[hull_sizes.index(max(hull_sizes))]
        # Check board meets min size
        if cv2.contourArea(cv2.convexHull(largest_hull_cnt)) < BOARD_MIN_AREA:
            return None

        x, y, w, h = cv2.boundingRect(largest_hull_cnt)
        cv2.rectangle(debug_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        center_x, center_y = x + w // 2, y + h // 2

        # Get line end points in 3d space so we can calculate orientation 
        # of the board. Points are returned as [x, y, z]
        orientation_point1 = self.get_point_from_pixels(x1_px, y1_px)
        orientation_point2 = self.get_point_from_pixels(x2_px, y2_px)

        if orientation_point1 is None or orientation_point2 is None:
            return None

        # Calculate orientation and get world coords
        orientation = math.atan2((orientation_point2.y - orientation_point1.y) , (orientation_point2.x - orientation_point1.x))
        point_world = self.get_point_from_pixels(center_x, center_y)

        # Use rolling average pose
        point_world, orientation = self.boardPoseRollingAverage.updatePose(point_world, orientation)
        self.broadcast_point_tf(point_world, orientation, 'board')

        if point_world:
            # Publish board pose in world frame
            board_pose = BoardPose()
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = 'base_link'
            point_msg.point = point_world
            board_pose.point = point_msg
            board_pose.anglerad = orientation
            self.board_pose_pub.publish(board_pose)

            # Publish grid square tfs
            self.broadcast_grid_square_tfs()
            
            coord_text = f"C:({point_world.x:.2f},{point_world.y:.2f},{point_world.z:.2f})"
            cv2.putText(debug_image, coord_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return (center_x, center_y)

    def find_white_pieces(self, camera_img, roi_mask, debug_image, pose_array):
        # HSV Seems to work quite well for white instead of grayscale, not sure why
        cv_img = cv2.cvtColor(camera_img, cv2.COLOR_BGR2HSV)
        cv_img = cv2.inRange(cv_img, WHITE_LOWER, WHITE_UPPER)

        # Apply ROI to make area outside ROI black
        cv_img = cv2.bitwise_and(cv_img, cv_img, mask=roi_mask)

        # Filtering and Smoothing 
        kernel = np.ones((WHITE_DILATION_KERNEL_SIZE, WHITE_DILATION_KERNEL_SIZE), np.uint8)
        cv_img = cv2.dilate(cv_img, kernel, iterations=1)
        kernel = np.ones((WHITE_ERODE_KERNEL_SIZE, WHITE_ERODE_KERNEL_SIZE), np.uint8)
        cv_img = cv2.erode(cv_img, kernel, iterations=1)
        cv_img = cv2.medianBlur(cv_img, 21)

        self.debug_white_piece_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_img, 'mono8'))

        contours, _ = cv2.findContours(cv_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt) # Get area first
            if WHITE_PIECE_MIN_AREA < area < WHITE_PIECE_MAX_AREA: # Check area
                hull = cv2.convexHull(cnt) # Get convex hull
                hull_area = float(cv2.contourArea(hull)) # Explicitly cast hull area to float

                solidity = 0.0 # Initialize solidity
                if hull_area > 0: # Now this comparison should be safe
                    solidity = float(area) / hull_area
                
                if solidity > WHITE_PIECE_MIN_SOLIDITY:
                    cv2.drawContours(debug_image, [cnt], -1, (255, 0, 0), 2)
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        point_world = self.get_point_from_pixels(cx, cy)
                        self.broadcast_point_tf(point_world, 0, f'white_piece{i}')
                        if point_world:
                            pose = Pose()
                            pose.position = point_world
                            pose.orientation.w = 1.0
                            pose_array.poses.append(pose)
                            coord_text = f"B:({point_world.x:.2f},{point_world.y:.2f})"
                            cv2.putText(debug_image, coord_text, (cx - 30, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

    def find_black_pieces(self, camera_img, roi_mask, debug_image, pose_array):
        # Image processing
        cv_img = cv2.cvtColor(camera_img, cv2.COLOR_BGR2HSV)
        cv_img = cv2.inRange(cv_img, BLACK_LOWER, BLACK_UPPER)

        # Apply ROI to make area outside ROI black
        cv_img = cv2.bitwise_and(cv_img, cv_img, mask=roi_mask)

        # Threshold to detect black
        erode_kernel = np.ones((BLACK_PIECE_ERODE_KERNEL_SIZE, BLACK_PIECE_ERODE_KERNEL_SIZE), np.uint8)
        cv_img = cv2.erode(cv_img, erode_kernel, iterations=1)
        dilate_kernel = np.ones((BLACK_PIECE_DILATE_KERNEL_SIZE,BLACK_PIECE_DILATE_KERNEL_SIZE), np.uint8)
        cv_img = cv2.dilate(cv_img, dilate_kernel, iterations=1)
        cv_img = cv2.medianBlur(cv_img, BLACK_PIECE_BLUR_SIZE)

        # Debugging
        self.debug_black_piece_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_img, 'mono8'))

        contours, _ = cv2.findContours(cv_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt) # Get area first
            if BLACK_PIECE_MIN_AREA < area < BLACK_PIECE_MAX_AREA: # Check area
                hull = cv2.convexHull(cnt) # Get convex hull
                hull_area = float(cv2.contourArea(hull)) # Explicitly cast hull area to float

                solidity = 0.0 # Initialize solidity
                if hull_area > 0: # Now this comparison should be safe
                    solidity = float(area) / hull_area
                
                if solidity > BLACK_PIECE_MIN_SOLIDITY:
                    cv2.drawContours(debug_image, [cnt], -1, (0, 0, 255), 2)
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        point_world = self.get_point_from_pixels(cx, cy)
                        self.broadcast_point_tf(point_world, 0, f'black_piece_{i}')
                        if point_world:
                            pose = Pose()
                            pose.position = point_world
                            pose.orientation.w = 1.0
                            pose_array.poses.append(pose)
                            coord_text = f"B:({point_world.x:.2f},{point_world.y:.2f})"
                            cv2.putText(debug_image, coord_text, (cx - 30, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    def get_point_from_pixels(self, x_px, y_px):
        # Check image bounds
        depth_height, depth_width = self.depth_image.shape[:2]

        if not (0 <= y_px < depth_height and 0 <= x_px < depth_width):
            self.get_logger().warn(f"Pixel ({x_px}, {y_px}) out of bounds.")
            return None

        # Get depth then 3d point from image
        depth = self.depth_image[y_px, x_px]

        if depth == 0:
            self.get_logger().warn(f"Zero depth for pixel ({x_px}, {y_px}).")
            return None

        point = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x_px, y_px], depth / 1000.0)

        if point == None:
            return None

        point_camera = PointStamped()
        point_camera.header.stamp = self.get_clock().now().to_msg()
        point_camera.header.frame_id = 'camera_color_optical_frame'
        point_camera.point.x = point[0] # - 0.038 # Offset for x (given from labs, may need tuning)
        point_camera.point.y = point[1]
        point_camera.point.z = point[2] - 0.06 # Offset for the z (since it seems to not be calibrated 100% correctly)

        # Try transform point to world coords from camera frame
        try:
            # Wait for the transform to be available
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            point_world = tf2_geometry_msgs.do_transform_point(point_camera, transform)
        except Exception as e:
            self.get_logger().error(f"Could not transform point : {e}")
            return None

        return point_world.point

    def broadcast_point_tf(self, point_world, orientation, frame_id):

        if point_world is None:
            return None

        # Publish relative to the base link so orientation is easier to manage
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = frame_id
        t.transform.translation.x = point_world.x
        t.transform.translation.y = point_world.y
        t.transform.translation.z = point_world.z

        # Rotate by the given orientation about the z axis
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, float(orientation))
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def broadcast_grid_square_tfs(self): 
        # Publish the center of each grid square on the board as a tf
        # the indexes and cell size are defined as constants at the top of the file
        for i, index in enumerate(GRID_IDEXES):

            # Publish relative to the board
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'board'
            t.child_frame_id = str(f'board_index_{i}')
            t.transform.translation.x = index[0] * CELL_SIZE
            t.transform.translation.y = index[1] * CELL_SIZE
            t.transform.translation.z = 0.0
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    time.sleep(5)
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

