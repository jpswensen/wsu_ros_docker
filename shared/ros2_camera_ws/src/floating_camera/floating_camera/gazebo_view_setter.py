#!/usr/bin/env python3
"""
Node to automatically set Gazebo GUI camera view after launch.
"""
import rclpy
from rclpy.node import Node
import subprocess
import time


class GazeboViewSetter(Node):
    def __init__(self):
        super().__init__('gazebo_view_setter')
        
        # Parameters for GUI camera position
        self.declare_parameter('gui_camera_x', -5.0)
        self.declare_parameter('gui_camera_y', 0.0)
        self.declare_parameter('gui_camera_z', 2.5)
        self.declare_parameter('gui_camera_roll', 0.0)
        self.declare_parameter('gui_camera_pitch', 0.0)
        self.declare_parameter('gui_camera_yaw', 0.0)
        self.declare_parameter('delay', 3.0)  # Wait for Gazebo to fully start
        
        delay = self.get_parameter('delay').value
        self.get_logger().info(f'Waiting {delay}s for Gazebo to start...')
        
        # Wait for Gazebo to start
        self.timer = self.create_timer(delay, self.set_view_once)
    
    def set_view_once(self):
        """Set the view once and cancel the timer."""
        self.timer.cancel()
        
        x = self.get_parameter('gui_camera_x').value
        y = self.get_parameter('gui_camera_y').value
        z = self.get_parameter('gui_camera_z').value
        roll = self.get_parameter('gui_camera_roll').value
        pitch = self.get_parameter('gui_camera_pitch').value
        yaw = self.get_parameter('gui_camera_yaw').value
        
        self.get_logger().info(f'Setting Gazebo GUI camera to ({x}, {y}, {z}), RPY=({roll}, {pitch}, {yaw})')
        
        # Convert roll, pitch, yaw to quaternion
        import math
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
        
        # Create Gazebo Pose message
        import tempfile
        import os
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
            # Write message to temporary file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
                f.write(msg_content)
                temp_file = f.name
            
            # Publish to Gazebo's user_camera joy_pose topic
            cmd = ['gz', 'topic', '-p', '/gazebo/default/user_camera/joy_pose', '-f', temp_file]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
            
            # Clean up temporary file
            os.unlink(temp_file)
            
            if result.returncode == 0:
                self.get_logger().info('✓ GUI camera view set successfully')
            else:
                self.get_logger().warn(f'Failed to set GUI camera: {result.stderr}')
                self.get_logger().info('You can manually set the view in Gazebo GUI: View -> Camera Pose')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Timeout setting GUI camera')
            if 'temp_file' in locals():
                os.unlink(temp_file)
        except FileNotFoundError:
            self.get_logger().warn('gz command not found. View must be set manually in Gazebo GUI.')
            if 'temp_file' in locals():
                os.unlink(temp_file)
        except Exception as e:
            self.get_logger().error(f'Error setting GUI camera: {e}')
            if 'temp_file' in locals():
                os.unlink(temp_file)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboViewSetter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
