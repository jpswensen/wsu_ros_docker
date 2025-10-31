#!/usr/bin/env python3
"""
Demo script for floating camera.
Subscribes to camera image and pose, publishes velocity commands, and displays the camera view.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf_transformations import quaternion_from_euler
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraDemo(Node):
    def __init__(self):
        super().__init__('camera_demo')
        
        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Current image and pose
        self.current_image = None
        self.current_pose = None
        
        # Create the OpenCV window immediately
        cv2.namedWindow('Floating Camera View', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Floating Camera View', 1280, 720)
        
        self.get_logger().info('Camera demo started!')
        self.get_logger().info('  OpenCV window created: "Floating Camera View"')
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/floating_camera/floating_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/floating_camera/pose',
            self.pose_callback,
            10
        )
        
        # Publisher for velocity commands
        self.set_pose_pub = self.create_publisher(Pose, '/floating_camera/set_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/floating_camera/cmd_vel', 10)
        
        # Give publishers time to connect
        import time
        time.sleep(0.5)
        
        # Zero the initial velocity
        zero_twist = Twist()
        self.cmd_vel_pub.publish(zero_twist)
        
        # Set initial pose
        zero_pose = Pose()
        zero_pose.position.x = 0.0
        zero_pose.position.y = 3.0
        zero_pose.position.z = -7.0

        # create an orientation in rpy, then convert to quaternion
        rpy = (0.0, 0.0, 0.0)  # Roll, Pitch, Yaw
        zero_pose_orientation = quaternion_from_euler(*rpy)
        zero_pose.orientation.x = zero_pose_orientation[0]
        zero_pose.orientation.y = zero_pose_orientation[1]
        zero_pose.orientation.z = zero_pose_orientation[2]
        zero_pose.orientation.w = zero_pose_orientation[3]
        self.set_pose_pub.publish(zero_pose)
        
        self.get_logger().info('Published initial pose and velocity commands')


        # Demo state
        self.start_time = self.get_clock().now()
        self.rotation_period = 2.0  # Swap direction every 2 seconds
        
        # Timer for control loop (20 Hz)
        control_loop_hz = 10.0
        self.vel_timer = self.create_timer(1/control_loop_hz, self.controller_callback)
        
        # Timer for image display (30 Hz)
        display_loop_hz = 30.0
        self.display_timer = self.create_timer(1.0/display_loop_hz, self.display_callback)
        
        
        self.get_logger().info('Done initializing')
        
        # Parameters to make detection thresholds tunable at runtime
        self.declare_parameter('blob_min_area', 12.0)      # px^2, SimpleBlobDetector min area
        self.declare_parameter('cc_min_area_frac', 2e-5)   # fraction of image area
        self.declare_parameter('cc_min_area_px', 12)       # absolute px^2 floor
        self.declare_parameter('cc_aspect_min', 0.3)
        self.declare_parameter('cc_aspect_max', 2.5)
        self.declare_parameter('merge_tol_px', 10.0)
        # Control gains (tunable)
        self.declare_parameter('k_yaw', -0.2)    # maps horizontal pixel error to yaw about +y
        self.declare_parameter('k_pitch', 0.2) # maps vertical pixel error to pitch about +x
        
        # Prepare a blob detector (created once)
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = True
        params.minArea = float(self.get_parameter('blob_min_area').value)
        params.maxArea = 1e6
        params.filterByCircularity = True
        params.minCircularity = 0.6
        params.filterByInertia = True
        params.minInertiaRatio = 0.2
        params.filterByConvexity = False
        params.minThreshold = 10
        params.maxThreshold = 220
        params.thresholdStep = 10
        try:
            # OpenCV 3/4 compatibility
            self.blob_detector = cv2.SimpleBlobDetector_create(params)
        except AttributeError:
            self.blob_detector = cv2.SimpleBlobDetector(params)
        
        # For visual overlays
        self._last_component_centers = []  # list of (x,y)
        self._last_blob_centers = []       # list of (x,y)
        self._last_target_center = None    # (x,y)
        
        
    def image_callback(self, msg: Image):
        """Receive camera images."""

        # Spend as little time in each callback as possible
        try:
            # Convert ROS Image to OpenCV format
            self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if not hasattr(self, '_image_received'):
                self._image_received = True
                self.get_logger().info('First image received!')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def pose_callback(self, msg: PoseStamped):
        """Receive camera pose."""
        self.current_pose = msg
        
    
    def find_points_centroid(self, img):
        h, w = img.shape[:2]
        cx0, cy0 = w / 2.0, h / 2.0

    
        # 1) Preprocess: grayscale, blur, and generate a binary mask (bright and dark spheres supported)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Otsu for bright-on-dark and dark-on-bright; combine
        _, thresh_bright = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        _, thresh_dark = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        mask = cv2.bitwise_or(thresh_bright, thresh_dark)
        
        # Morphological cleanup
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        # 2) Connected components with stats
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        
        component_centers = []
        # Adaptive minimum area: pick the max of a fraction of image area and an absolute pixel floor
        min_area_px = int(self.get_parameter('cc_min_area_px').value)
        min_area_frac = float(self.get_parameter('cc_min_area_frac').value)
        min_area = max(min_area_px, int(min_area_frac * w * h))
        max_area = int(0.25 * w * h)               # avoid giant background-like regions
        aspect_min = float(self.get_parameter('cc_aspect_min').value)
        aspect_max = float(self.get_parameter('cc_aspect_max').value)
        for i in range(1, num_labels):  # skip background = 0
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area or area > max_area:
                continue
            ww = stats[i, cv2.CC_STAT_WIDTH]
            hh = stats[i, cv2.CC_STAT_HEIGHT]
            if hh == 0:
                continue
            aspect = ww / float(hh)
            if aspect < aspect_min or aspect > aspect_max:
                # Rough circular-ish constraint
                continue
            # Accept centroid
            cx, cy = centroids[i]
            component_centers.append((float(cx), float(cy)))
        
        # 3) Blob detector to find circular blobs (works on grayscale)
        keypoints = self.blob_detector.detect(gray_blur)
        blob_centers = [(kp.pt[0], kp.pt[1]) for kp in keypoints]
        
        # 4) Merge detections: start from component centers, add blob centers not near existing ones
        merged_centers = list(component_centers)
        merge_tol = float(self.get_parameter('merge_tol_px').value)
        def is_near(p, q, tol=merge_tol):
            return (p[0]-q[0])**2 + (p[1]-q[1])**2 <= tol*tol
        for bc in blob_centers:
            if not any(is_near(bc, cc) for cc in merged_centers):
                merged_centers.append(bc)
        
        # Save for display overlays
        self._last_component_centers = component_centers
        self._last_blob_centers = blob_centers
        
        # 5) Compute geometric center (average of centers)
        target_center_raw = None
        if len(merged_centers) > 0:
            mx = float(np.mean([p[0] for p in merged_centers]))
            my = float(np.mean([p[1] for p in merged_centers]))
            target_center_raw = (mx, my)

        return target_center_raw, cx0, cy0


    def controller_callback(self):
        """Implement a visual servoing controller."""

        cmd = Twist()
        
        # Implement a controller that finds the centroid of the spheres in the image and commands
        # angular velocities in yaw (about +y) and pitch (about +x) to center them.
        img = self.current_image
        if img is None:
            # No image yet; stop motion
            self.cmd_vel_pub.publish(cmd)
            return
        
        
        target_center_raw, cx0, cy0 = self.find_points_centroid(img)
        self._filt_center = target_center_raw
        self._last_target_center = self._filt_center
        
        # 6) Control law: yaw (about y), pitch (about x), and roll (about z)
        if self._filt_center is not None:

            # Compute pixel errors
            ex = (cx0 - self._filt_center[0]) / cx0
            ey = (cy0 - self._filt_center[1]) / cy0

            # # Gains (tunable)
            k_yaw = float(self.get_parameter('k_yaw').value)
            k_pitch = float(self.get_parameter('k_pitch').value)

            # Simple proportional controller
            cmd.angular.y = k_yaw * ex  # Yaw about +y
            cmd.angular.x = k_pitch * ey  # Pitch about +x

            print(f'Errors: ex {ex:.1f}, ey {ey:.1f}')


        print(f'Cmd angular: pitch {cmd.angular.x:.3f}, yaw {cmd.angular.y:.3f}, roll {cmd.angular.z:.3f}')
        # Linear velocities remain zero; only orientation changes (pitch/yaw)
        self.cmd_vel_pub.publish(cmd)
    
    def display_callback(self):
        """Display the camera image with overlays."""
        if self.current_image is None:
            # Show a black placeholder image with waiting message
            placeholder = np.zeros((720, 1280, 3), dtype=np.uint8)
            cv2.putText(
                placeholder,
                'Waiting for camera images...',
                (400, 360),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2
            )
            cv2.imshow('Floating Camera View', placeholder)
            cv2.waitKey(1)
            return
        
        # Make a copy for display
        display_img = self.current_image.copy()
        
        # Add text overlay with pose information
        if self.current_pose is not None:
            pos = self.current_pose.pose.position
            text_lines = [
                f'Position: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})',
                f'Time: {(self.get_clock().now() - self.start_time).nanoseconds / 1e9:.1f}s'
            ]
            
            y_offset = 30
            for line in text_lines:
                cv2.putText(
                    display_img,
                    line,
                    (10, y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )
                y_offset += 30
        
        # Overlays: detections and target center
        # Draw component centers (green), blob centers (cyan), and average center (red)
        for (x, y) in getattr(self, '_last_component_centers', []):
            cv2.drawMarker(display_img, (int(x), int(y)), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=6)
        for (x, y) in getattr(self, '_last_blob_centers', []):
            cv2.circle(display_img, (int(x), int(y)), 6, (255, 255, 0), 2)
        if getattr(self, '_last_target_center', None) is not None:
            tcx, tcy = self._last_target_center
            cv2.circle(display_img, (int(tcx), int(tcy)), 8, (0, 0, 255), -1)
        # Draw image center
        h, w = display_img.shape[:2]
        cv2.drawMarker(display_img, (w//2, h//2), (100, 100, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=24, thickness=6)
        
        # Display the image
        cv2.imshow('Floating Camera View', display_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
