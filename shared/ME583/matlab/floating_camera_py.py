#!/usr/bin/env python3
"""
ROS 2 floating camera client controlled from MATLAB via py.* calls.

- Subscribes to camera image and pose
- Publishes velocity commands when told (no built-in motion profile)
- Exposes start()/stop()/publish_twist()/get_current_image_* for MATLAB
"""
from __future__ import annotations
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist, Pose
import numpy as np


class FloatingCameraNode(Node):
    def __init__(self):
        super().__init__('floating_camera')

        # Current image, encoding, and pose
        self.current_image = None  # numpy array: HxW or HxWxC (uint8)
        self.current_encoding = None  # e.g., 'bgr8','rgb8','mono8','bgra8','rgba8'
        self.current_pose = None

        self.get_logger().info('Floating camera node started!')

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

        # Publisher for velocity commands (external control)
        self.vel_pub = self.create_publisher(Twist, '/floating_camera/cmd_vel', 10)
        
        # Publisher for setting pose directly
        self.set_pose_pub = self.create_publisher(Pose, '/floating_camera/set_pose', 10)

        self.get_logger().info('  Subscribing to /floating_camera/floating_camera/image_raw')
        self.get_logger().info('  Subscribing to /floating_camera/pose')
        self.get_logger().info('  Publishing to /floating_camera/cmd_vel')
        self.get_logger().info('  Publishing to /floating_camera/set_pose')

    def image_callback(self, msg: Image):
        try:
            enc = (msg.encoding or '').lower()
            h = int(msg.height)
            w = int(msg.width)
            step = int(msg.step)
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if h <= 0 or w <= 0 or step <= 0 or buf.size < h * step:
                self.get_logger().warn('Invalid image dimensions or buffer; dropping frame')
                return

            # Guess channels from encoding
            if enc in ('rgb8', 'bgr8'):
                ch = 3
            elif enc in ('rgba8', 'bgra8'):
                ch = 4
            elif enc in ('mono8', '8uc1'):
                ch = 1
            else:
                # Fallback by step
                if step == w:
                    ch = 1
                elif step == w * 3:
                    ch = 3
                elif step == w * 4:
                    ch = 4
                else:
                    self.get_logger().warn(f'Unsupported image encoding {enc} (step={step}, w={w})')
                    return

            rows = buf[: h * step].reshape((h, step))
            if ch == 1:
                img = rows[:, :w]
            else:
                img = rows[:, : w * ch].reshape((h, w, ch))

            self.current_image = img
            self.current_encoding = enc
            if not hasattr(self, '_image_received'):
                self._image_received = True
                self.get_logger().info('First image received!')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg


class FloatingCameraWrapper:
    """
    Small helper so MATLAB can:
      - construct Python object
      - start spinning (non-blocking thread)
      - stop/clean up
      - publish manual twists
      - fetch latest image
    """
    def __init__(self):
        self._running = False
        self._thread = None
        self._stop_flag = False

        try:
            rclpy.init(args=None)
        except Exception:
            # If already initialized or in a weird state, try to continue
            pass
        self.node = FloatingCameraNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def _spin_loop(self):
        # Spin in small steps so stop() can exit quickly
        while not self._stop_flag and rclpy.ok():
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception:
                time.sleep(0.05)

    def start(self):
        if self._running:
            return
        self._stop_flag = False
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()
        self._running = True

    def stop(self):
        if not self._running:
            return
        self._stop_flag = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)

        try:
            self.executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        self._thread = None
        self._running = False

    def is_running(self) -> bool:
        return self._running

    def publish_twist(self, lx=0.0, ly=0.0, lz=0.0, ax=0.0, ay=0.0, az=0.0):
        """
        Allow MATLAB to send a manual twist.
        """
        msg = Twist()
        msg.linear.x = float(lx); msg.linear.y = float(ly); msg.linear.z = float(lz)
        msg.angular.x = float(ax); msg.angular.y = float(ay); msg.angular.z = float(az)
        self.node.vel_pub.publish(msg)

    def set_pose(self, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        Set the camera pose directly.
        x, y, z: position
        qx, qy, qz, qw: orientation quaternion
        """
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)
        self.node.set_pose_pub.publish(msg)

    def get_current_image_rgb(self):
        """
        Return current image as an RGB numpy array (uint8) or None.
        Uses encoding to decide channel order and alpha.
        """
        img = self.node.current_image
        if img is None:
            return None
        enc = (self.node.current_encoding or '').lower()
        if img.ndim == 2:
            return img
        # Drop alpha if present
        if img.ndim == 3 and img.shape[2] >= 3:
            rgb = img[:, :, :3]
            if enc.startswith('bgr'):
                rgb = rgb[:, :, ::-1]
            return rgb.copy()
        return img

    def get_current_image_rgb_bytes(self):
        """
        Return a tuple (h, w, c, bytes) in Fortran order to match MATLAB reshape.
        bytes is Python 'bytes'. Returns None if no image yet.
        """
        img = self.node.current_image
        if img is None:
            return None
        enc = (self.node.current_encoding or '').lower()
        if img.ndim == 2:
            rgb = np.asfortranarray(img)
            h, w = int(rgb.shape[0]), int(rgb.shape[1])
            c = 1
            return (h, w, c, rgb.tobytes(order='F'))
        rgb = img[:, :, :3]
        if enc.startswith('bgr'):
            rgb = rgb[:, :, ::-1]
        rgb = np.asfortranarray(rgb)
        h, w = int(rgb.shape[0]), int(rgb.shape[1])
        c = int(rgb.shape[2]) if rgb.ndim == 3 else 1
        return (h, w, c, rgb.tobytes(order='F'))

    def get_pose(self):
        """
        Return the latest pose as a tuple:
            (x, y, z, qx, qy, qz, qw, stamp_sec, stamp_nanosec, frame_id)
        or None if no pose has been received yet.
        """
        ps = self.node.current_pose
        if ps is None:
            return None
        p = ps.pose.position
        q = ps.pose.orientation
        stamp = ps.header.stamp
        frame_id = ps.header.frame_id
        return (
            float(p.x), float(p.y), float(p.z),
            float(q.x), float(q.y), float(q.z), float(q.w),
            int(stamp.sec), int(stamp.nanosec),
            frame_id
        )
