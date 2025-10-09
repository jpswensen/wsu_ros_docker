#!/usr/bin/env python3
"""
Demo script for floating camera.
Subscribes to camera image and pose, publishes velocity commands, and displays the camera view.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
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
        self.vel_pub = self.create_publisher(Twist, '/floating_camera/cmd_vel', 10)
        
        # Demo state
        self.start_time = self.get_clock().now()
        self.rotation_period = 2.0  # Swap direction every 2 seconds
        
        # Timer for velocity commands (10 Hz)
        self.vel_timer = self.create_timer(0.1, self.velocity_callback)
        
        # Timer for image display (30 Hz)
        self.display_timer = self.create_timer(1.0/30.0, self.display_callback)
        
        self.get_logger().info('  Subscribing to /floating_camera/floating_camera/image_raw')
        self.get_logger().info('  Subscribing to /floating_camera/pose')
        self.get_logger().info('  Publishing to /floating_camera/cmd_vel')
        
    def image_callback(self, msg: Image):
        """Receive camera images."""
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
    
    def velocity_callback(self):
        """Publish velocity commands for the demo."""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        cmd = Twist()
        
        # Swap rotation direction every 2 seconds indefinitely
        # Note: Camera uses optical frame (Z forward, X right, Y down)
        # So rotation around Y axis = yaw (left/right rotation)
        cycle_time = elapsed % (2 * self.rotation_period)
        
        if cycle_time < self.rotation_period:
            # Positive Y rotation (yaw left in optical frame)
            cmd.angular.z = 0.5  # rad/s
            state_msg = 'Rotating left (positive Y angular velocity in optical frame)'
        else:
            # Negative Y rotation (yaw right in optical frame)
            cmd.angular.z = -0.5  # rad/s
            state_msg = 'Rotating right (negative Y angular velocity in optical frame)'
        
        # Log state changes (only once per transition)
        if not hasattr(self, '_last_state') or self._last_state != state_msg:
            self._last_state = state_msg
            self.get_logger().info(f'Demo: {state_msg}')
        
        self.vel_pub.publish(cmd)
    
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
        
        # Display the image
        cv2.imshow('Floating Camera View', display_img)
        cv2.waitKey(1)
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
