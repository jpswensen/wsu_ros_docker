#!/usr/bin/env python3
"""
Simple image capture script for the floating camera.

Publishes a target pose to /floating_camera/set_pose, waits for the first
image on /floating_camera/floating_camera/image_raw, saves it to a PNG, and exits.

Usage:
    python image_capture.py --x 0.0 --y 3.0 --z -7.0 --filename capture.png

If the image isn't received within --timeout seconds, the node will exit with an error log.
"""
from __future__ import annotations

import argparse
import time
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from tf_transformations import quaternion_from_euler
from cv_bridge import CvBridge
import cv2


class ImageCapture(Node):
    def __init__(self, target_pose: Pose, filename: str, timeout: float = 5.0):
        super().__init__('image_capture')
        self.bridge = CvBridge()
        self.filename = filename
        self.timeout = float(timeout)
        self._saved = False

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/floating_camera/floating_camera/image_raw',
            self._image_callback,
            10,
        )

        self.set_pose_pub = self.create_publisher(Pose, '/floating_camera/set_pose', 10)

        # Give publishers time to connect
        time.sleep(0.5)

        # Publish desired pose once
        self.get_logger().info(f'Publishing target pose: x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}')
        self.set_pose_pub.publish(target_pose)

        time.sleep(1.0)

        # Start timeout timer to abort if no image arrives
        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(0.1, self._check_timeout)

    def _image_callback(self, msg: Image):
        if self._saved:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image message: {e}')
            return

        try:
            # Save image as PNG
            success = cv2.imwrite(self.filename, img)
            if not success:
                raise RuntimeError('cv2.imwrite returned False')
            self.get_logger().info(f'Saved image to: {self.filename}')
            self._saved = True
            # Small sleep to allow logs/IO to flush
            time.sleep(0.1)
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Failed to save image to {self.filename}: {e}')

    def _check_timeout(self):
        if self._saved:
            return
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed >= self.timeout:
            self.get_logger().error(f'Timeout waiting for image ({self.timeout}s)')
            rclpy.shutdown()


def build_pose_from_args(args) -> Pose:
    p = Pose()
    p.position.x = float(args.x)
    p.position.y = float(args.y)
    p.position.z = float(args.z)

    # Orientation: accept roll/pitch/yaw if provided, otherwise zero
    roll = float(args.roll)
    pitch = float(args.pitch)
    yaw = float(args.yaw)
    q = quaternion_from_euler(roll, pitch, yaw)
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]
    return p


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Publish pose, capture image, save PNG, and exit')
    parser.add_argument('--x', type=float, default=0.0, help='Target X position')
    parser.add_argument('--y', type=float, default=3.0, help='Target Y position')
    parser.add_argument('--z', type=float, default=-7.0, help='Target Z position')
    parser.add_argument('--roll', type=float, default=0.0, help='Target roll (radians)')
    parser.add_argument('--pitch', type=float, default=0.0, help='Target pitch (radians)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Target yaw (radians)')
    parser.add_argument('--filename', type=str, default='capture.png', help='Output PNG filename')
    parser.add_argument('--timeout', type=float, default=5.0, help='Seconds to wait for an image before aborting')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # Build Pose message
    pose = build_pose_from_args(args)

    rclpy.init(args=None)
    node = ImageCapture(pose, args.filename, timeout=args.timeout)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
