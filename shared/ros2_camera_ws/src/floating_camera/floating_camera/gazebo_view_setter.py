#!/usr/bin/env python3
"""
Node to automatically set Gazebo GUI camera view after launch.

Robustness notes:
- Publishes a Pose to one of several common GUI camera topics (prefers topics with active subscribers).
- Retries for a short period to allow gzclient to start before attempting to set the view.
"""
import rclpy
from rclpy.node import Node
import subprocess
import time
from typing import List


class GazeboViewSetter(Node):
    def __init__(self):
        super().__init__('gazebo_view_setter')
        self._done = False
        
        # Parameters for GUI camera position
        self.declare_parameter('gui_camera_x', -5.0)
        self.declare_parameter('gui_camera_y', 0.0)
        self.declare_parameter('gui_camera_z', 2.5)
        self.declare_parameter('gui_camera_roll', 0.0)
        self.declare_parameter('gui_camera_pitch', 0.0)
        self.declare_parameter('gui_camera_yaw', 0.0)
        self.declare_parameter('delay', 3.0)  # Initial wait before first attempt
        self.declare_parameter('retries', 10)  # Additional attempts after the first
        self.declare_parameter('retry_period', 0.5)  # seconds between attempts
        self.declare_parameter('publish_count', 5)  # how many times to publish pose per attempt
        
        delay = float(self.get_parameter('delay').value)
        self.get_logger().info(f'Waiting {delay:.2f}s for Gazebo to start...')

        # Wait for Gazebo to start, then begin attempts
        self._first_timer = self.create_timer(delay, self._begin_attempts)

    def _begin_attempts(self):
        # Cancel the first-delay timer and kick off retries
        self._first_timer.cancel()
        self._remaining_attempts = int(self.get_parameter('retries').value)
        self._retry_period = float(self.get_parameter('retry_period').value)
        # Try immediately, then set periodic retry on failure
        ok = self._try_set_view_once()
        if ok:
            self._done = True
            return
        # Schedule retries
        self._retry_timer = self.create_timer(self._retry_period, self._retry_once)

    def _retry_once(self):
        if getattr(self, '_done', False):
            try:
                self._retry_timer.cancel()
            except Exception:
                pass
            return
        if self._remaining_attempts <= 0:
            self.get_logger().warn('Exhausted attempts to set GUI camera view.')
            self._retry_timer.cancel()
            return
        self._remaining_attempts -= 1
        ok = self._try_set_view_once()
        if ok:
            self._done = True
            self._retry_timer.cancel()

    def _try_set_view_once(self) -> bool:
        x = self.get_parameter('gui_camera_x').value
        y = self.get_parameter('gui_camera_y').value
        z = self.get_parameter('gui_camera_z').value
        roll = self.get_parameter('gui_camera_roll').value
        pitch = self.get_parameter('gui_camera_pitch').value
        yaw = self.get_parameter('gui_camera_yaw').value
        
        self.get_logger().info(f'Attempting to set Gazebo GUI camera to ({x}, {y}, {z}), RPY=({roll}, {pitch}, {yaw})')

        # Prefer topics that actually have subscribers to the GUI client
        candidate_topics: List[str] = [
            '/gazebo/default/user_camera/joy_pose',
            '/gazebo/default/user_camera/pose',
            '/gazebo/default/gui/camera/pose',
        ]

        topic_with_sub = None
        for t in candidate_topics:
            if self._topic_has_subscriber(t):
                topic_with_sub = t
                break

        topics_to_try = [topic_with_sub] if topic_with_sub else candidate_topics
        if topic_with_sub:
            self.get_logger().info(f'Publishing GUI pose to subscribed topic: {topic_with_sub}')
        else:
            self.get_logger().warn('No subscribed GUI camera pose topic detected; publishing to common topics anyway')

        for t in topics_to_try:
            if t and self._try_publish_pose(t, x, y, z, roll, pitch, yaw):
                self.get_logger().info(f'✓ GUI camera view set via topic: {t}')
                return True

        self.get_logger().debug('GUI camera not set this attempt; will retry if attempts remain...')
        return False

    def _topic_has_subscriber(self, topic: str) -> bool:
        """Return True if `gz topic -i <topic>` shows one or more subscribers."""
        try:
            info = subprocess.run(['gz', 'topic', '-i', topic], capture_output=True, text=True, timeout=1.5)
            if info.returncode != 0:
                return False
            # Heuristic: look for 'Subscribers:' followed by a non-empty address line
            out = info.stdout
            if 'Subscribers:' not in out:
                return False
            # If there is any line with host:port under Subscribers, consider it subscribed
            for line in out.splitlines():
                if ':' in line and '/' not in line and 'Publishers' not in line and 'Subscribers' not in line:
                    return True
            return False
        except Exception:
            return False

    def _try_publish_pose(self, topic: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> bool:
        """Publish a gazebo.msgs.Pose to a topic using 'gz topic -p <topic> -f <file>'."""
        import math
        import tempfile
        import os
        # Convert RPY to quaternion
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        msg_content = f"""position {{
  x: {x}
  y: {y}
  z: {z}
}}
orientation {{
  x: {qx}
  y: {qy}
  z: {qz}
  w: {qw}
}}
"""
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
                f.write(msg_content)
                temp_file = f.name
            count = int(self.get_parameter('publish_count').value)
            success_any = False
            for _ in range(max(1, count)):
                result = subprocess.run(
                    ['gz', 'topic', '-p', topic, '-f', temp_file],
                    capture_output=True, text=True, timeout=2.0
                )
                if result.returncode == 0:
                    success_any = True
                time.sleep(0.05)
            os.unlink(temp_file)
            return success_any
        except Exception:
            try:
                if 'temp_file' in locals():
                    os.unlink(temp_file)
            except Exception:
                pass
            return False


def main(args=None):
    rclpy.init(args=args)
    node = GazeboViewSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
