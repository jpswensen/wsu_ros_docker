#!/usr/bin/env python3
"""
Script to set the Gazebo GUI camera (viewport) position.
Can be called as a standalone script or imported.
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
                   roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0):
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
    
    # Convert roll, pitch, yaw to quaternion
    q = euler_to_quaternion(roll, pitch, yaw)
    
    # Create a temporary file with the Gazebo Pose message
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
    
    try:
        # Write message to temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            f.write(msg_content)
            temp_file = f.name
        
        # Publish to Gazebo's user_camera joy_pose topic using gz command
        cmd = [
            'gz', 'topic',
            '-p', '/gazebo/default/user_camera/joy_pose',
            '-f', temp_file
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
        
        # Clean up temporary file
        os.unlink(temp_file)
        
        if result.returncode == 0:
            node.get_logger().info('GUI camera position set successfully')
        else:
            node.get_logger().warn(f'Failed to set GUI camera: {result.stderr}')
    except subprocess.TimeoutExpired:
        node.get_logger().warn('Timeout setting GUI camera')
        if 'temp_file' in locals():
            os.unlink(temp_file)
    except FileNotFoundError:
        node.get_logger().error('gz command not found. Make sure Gazebo is installed.')
        if 'temp_file' in locals():
            os.unlink(temp_file)


def main(args=None):
    """Standalone script to set GUI camera position."""
    rclpy.init(args=args)
    node = Node('set_gui_camera')
    
    # Default: position camera behind floating camera looking forward
    # Adjust these values as needed
    x = 0.0  # Behind the scene
    y = 0.0
    z = -7.0   # Slightly elevated
    roll = 0.0
    pitch = -np.pi/2  # Could tilt down: -0.2
    yaw = np.pi/2    # Look forward
    
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
