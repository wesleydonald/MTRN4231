import rclpy
import cv2
import numpy as np
import pyrealsense2 as rs
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformBroadcaster, TransformStamped

class ObjectRecognizer(Node):
    """
    A ROS2 node to detect a chessboard, black cross pieces, and white round pieces.
    It identifies these objects in the camera's 2D image, converts their
    locations to 3D coordinates using depth data, and publishes them.
    """

    def __init__(self):
        super().__init__('object_recognizer')

        # --- Constants for Tuning ---
        # White piece detection (HSV color range)
        self.LOWER_WHITE = np.array([0, 0, 180])
        self.UPPER_WHITE = np.array([180, 40, 255])

        # Black piece detection (HSV color range)
        self.LOWER_BLACK = np.array([0, 0, 0])
        self.UPPER_BLACK = np.array([180, 255, 60])

        # Contour filtering parameters
        self.MIN_AREA_PIECE = 200        # Minimum contour area to be considered a piece
        self.MAX_AREA_PIECE = 2000       # Maximum contour area to be considered a piece
        self.MIN_AREA_BOARD = 10000      # Minimum contour area for the chessboard
        self.CIRCULARITY_THRESHOLD = 0.8 # Value close to 1.0 is a perfect circle

        # --- ROS2 Communications ---
        self.cv_bridge = CvBridge()

        # Subscriptions to RealSense camera topics
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)

        # Publishers for detected object locations
        self.board_pub = self.create_publisher(PointStamped, '~/detected/chessboard', 10)
        self.cross_pub = self.create_publisher(PointStamped, '~/detected/cross_piece', 10)
        self.round_pub = self.create_publisher(PointStamped, '~/detected/round_piece', 10)
        self.debug_image_pub = self.create_publisher(Image, '~/debug_image', 10)

        # TF2 Broadcaster to publish object frames
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Internal State ---
        self.color_image = None
        self.depth_image = None
        self.intrinsics = None
        self.get_logger().info("Object Recognizer node has started.")

    def camera_info_callback(self, msg):
        """Callback to receive camera intrinsic parameters."""
        if not self.intrinsics:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            if msg.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady
            else:
                self.intrinsics.model = rs.distortion.none
            self.intrinsics.coeffs = list(msg.d)

    def image_callback(self, msg):
        """Callback for the color image stream."""
        try:
            self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image()
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def depth_callback(self, msg):
        """Callback for the depth image stream."""
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def process_image(self):
        """Main image processing function."""
        if self.color_image is None:
            return

        # Create a copy for drawing visualizations
        debug_image = self.color_image.copy()
        hsv_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)

        # --- Object Detection ---
        self.detect_chessboard(self.color_image, debug_image)
        self.detect_pieces(hsv_image, debug_image)

        # Publish the debug image
        try:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error publishing debug image: {e}")

    def detect_chessboard(self, image, debug_image):
        """Detects the largest rectangle (chessboard) in the image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(cnt)
            if area < self.MIN_AREA_BOARD:
                continue

            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

            if len(approx) == 4:
                # Found the chessboard
                cv2.drawContours(debug_image, [approx], -1, (0, 255, 0), 3)
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(debug_image, (cx, cy), 7, (0, 255, 0), -1)
                    self.publish_3d_point(cx, cy, 'chessboard', self.board_pub)
                break # Assume largest rectangle is the board

    def detect_pieces(self, hsv_image, debug_image):
        """Detects round white pieces and cross-shaped black pieces."""
        # Detect white pieces
        mask_white = cv2.inRange(hsv_image, self.LOWER_WHITE, self.UPPER_WHITE)
        mask_white = cv2.medianBlur(mask_white, 5)
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours_white:
            area = cv2.contourArea(cnt)
            if self.MIN_AREA_PIECE < area < self.MAX_AREA_PIECE:
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0: continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))

                if circularity > self.CIRCULARITY_THRESHOLD:
                    # This is likely a round white piece
                    M = cv2.moments(cnt)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.circle(debug_image, (cx, cy), 10, (255, 0, 0), 2)
                    self.publish_3d_point(cx, cy, 'white_round_piece', self.round_pub)
        
        # Detect black pieces
        mask_black = cv2.inRange(hsv_image, self.LOWER_BLACK, self.UPPER_BLACK)
        mask_black = cv2.medianBlur(mask_black, 5)
        contours_black, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours_black:
            area = cv2.contourArea(cnt)
            if self.MIN_AREA_PIECE < area < self.MAX_AREA_PIECE:
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0: continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                
                if circularity < (self.CIRCULARITY_THRESHOLD - 0.3): # Crosses are not circular
                    # This is likely a black cross piece
                    M = cv2.moments(cnt)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cv2.drawContours(debug_image, [cnt], -1, (0, 0, 255), 2)
                    self.publish_3d_point(cx, cy, 'black_cross_piece', self.cross_pub)


    def publish_3d_point(self, x_pixel, y_pixel, object_name, publisher):
        """Converts pixel coordinates to 3D and publishes."""
        if self.depth_image is None or self.intrinsics is None:
            return

        # Ensure pixel coordinates are within image bounds
        h, w = self.depth_image.shape
        if not (0 <= y_pixel < h and 0 <= x_pixel < w):
            self.get_logger().warn(f"Pixel ({x_pixel}, {y_pixel}) is out of bounds.")
            return

        depth = self.depth_image[y_pixel, x_pixel]
        if depth == 0:
            # self.get_logger().warn(f"No depth data for {object_name} at pixel ({x_pixel}, {y_pixel}).")
            return

        # Convert depth from mm (uint16) to meters
        depth_in_meters = depth * 0.001
        
        # Deproject pixel to 3D point in camera frame
        point_3d = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x_pixel, y_pixel], depth_in_meters)
        
        # Create and publish PointStamped message
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = "camera_color_optical_frame"
        point_msg.point.x = point_3d[0]
        point_msg.point.y = point_3d[1]
        point_msg.point.z = point_3d[2]
        publisher.publish(point_msg)

        # Broadcast a TF frame for the object
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_color_optical_frame"
        t.child_frame_id = f"{object_name}_frame"
        t.transform.translation.x = point_3d[0]
        t.transform.translation.y = point_3d[1]
        t.transform.translation.z = point_3d[2]
        t.transform.rotation.w = 1.0  # No rotation
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(f"Published {object_name} at [x:{point_3d[0]:.3f}, y:{point_3d[1]:.3f}, z:{point_3d[2]:.3f}]")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
