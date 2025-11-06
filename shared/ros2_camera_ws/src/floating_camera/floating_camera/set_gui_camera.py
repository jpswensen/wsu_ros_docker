#!/usr/bin/env python3
"""
Script to set the Gazebo GUI camera (viewport) position.
Can be called as a standalone script or imported.

It prefers Gazebo Classic's 'gz camera' CLI and falls back to publishing a Pose
to common GUI camera topics using 'gz topic'.
"""
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import sys
import numpy as np


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles to quaternion."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def set_gui_camera(node: Node, x: float, y: float, z: float,
                   roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> bool:
    """
    Set the Gazebo GUI camera position and orientation.
    
    Args:
        node: ROS 2 node
        x, y, z: Camera position in world frame
        roll, pitch, yaw: Camera orientation in radians
    """
    node.get_logger().info(f'Setting GUI camera to position ({x}, {y}, {z}), RPY=({roll:.2f}, {pitch:.2f}, {yaw:.2f})')
    
    import subprocess
    import tempfile
    import os

    # First attempt: use 'gz camera' if available and gzclient camera exists
    try:
        list_result = subprocess.run(
            ['gz', 'camera', '-l'], capture_output=True, text=True, timeout=1.5
        )
        if list_result.returncode == 0 and 'gzclient_camera' in list_result.stdout:
            pose_arg = f"{x},{y},{z},{roll},{pitch},{yaw}"
            result = subprocess.run(
                ['gz', 'camera', '-c', 'gzclient_camera', '-f', pose_arg],
                capture_output=True, text=True, timeout=2.0
            )
            if result.returncode == 0:
                node.get_logger().info('GUI camera position set via "gz camera"')
                return True
    except Exception:
        pass

    # Fallback: publish Pose to likely topics
    q = euler_to_quaternion(roll, pitch, yaw)
    msg_content = f"""position {{
  x: {x}
  y: {y}
  z: {z}
}}
orientation {{
  x: {q.x}
  y: {q.y}
  z: {q.z}
  w: {q.w}
}}
"""
    topics = [
        '/gazebo/default/user_camera/pose',
        '/gazebo/default/gui/camera/pose',
        '/gazebo/default/user_camera/joy_pose',
    ]
    for t in topics:
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
                f.write(msg_content)
                temp_file = f.name
            result = subprocess.run(
                ['gz', 'topic', '-p', t, '-f', temp_file],
                capture_output=True, text=True, timeout=2.0
            )
            os.unlink(temp_file)
            if result.returncode == 0:
                node.get_logger().info(f'GUI camera position set via topic: {t}')
                return True
        except Exception:
            try:
                if 'temp_file' in locals():
                    os.unlink(temp_file)
            except Exception:
                pass
    node.get_logger().warn('Failed to set GUI camera using available methods')
    return False


def main(args=None):
    """Standalone script to set GUI camera position."""
    rclpy.init(args=args)
    node = Node('set_gui_camera')
    
    # Default view: behind the scene looking along +X (camera optical axis in world)
    # Adjust as needed to match your world orientation
    x = 0.0
    y = 0.0
    z = -7.0
    roll = 0.0
    pitch = -np.pi/2
    yaw = np.pi/2
    
    # Parse command line arguments if provided
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    if len(sys.argv) >= 7:
        roll = float(sys.argv[4])
        pitch = float(sys.argv[5])
        yaw = float(sys.argv[6])
    
    set_gui_camera(node, x, y, z, roll, pitch, yaw)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
