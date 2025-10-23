#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import random
import yaml
from typing import Tuple, Optional

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

from .camera_controller import CameraController, euler_to_quaternion


def clamp01(x: float) -> float:
  return max(0.0, min(1.0, x))


def apply_camera_frame_transform(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    """
    Apply constant transformation so that camera's natural orientation
    (Z forward, X right, Y down) corresponds to zero rotation.
    
    The camera's natural frame has:
    - Z forward (world X)
    - X right (world Y)
    - Y down (world Z)
    
    This requires a rotation of roll=90°, pitch=-90° to align with world frame.
    We apply this as a pre-rotation to the user's input.
    """
  # Constant offset: roll=pi/2, pitch=-pi/2, yaw=0
    offset_roll = math.pi / 2
    offset_pitch = -math.pi / 2
    offset_yaw = 0.0
    
  # Combine rotations: user specifies rotation in the OPTICAL frame (q_user).
  # We need the LINK frame orientation in world, so apply the fixed
  # optical->link rotation AFTER the user's optical rotation:
  #   q_world_link = q_world_optical (q_user) * q_optical_to_link (q_offset)
  # Convert both to quaternions and multiply in that order.
    # Offset quaternion
    cr = math.cos(offset_roll * 0.5)
    sr = math.sin(offset_roll * 0.5)
    cp = math.cos(offset_pitch * 0.5)
    sp = math.sin(offset_pitch * 0.5)
    cy = math.cos(offset_yaw * 0.5)
    sy = math.sin(offset_yaw * 0.5)
    
    qo_w = cr * cp * cy + sr * sp * sy
    qo_x = sr * cp * cy - cr * sp * sy
    qo_y = cr * sp * cy + sr * cp * sy
    qo_z = cr * cp * sy - sr * sp * cy
    
    # User quaternion
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    
    qu_w = cr * cp * cy + sr * sp * sy
    qu_x = sr * cp * cy - cr * sp * sy
    qu_y = cr * sp * cy + sr * cp * sy
    qu_z = cr * cp * sy - sr * sp * cy
    
    # Multiply quaternions: q_final = q_user * q_offset
    qf_w = qu_w * qo_w - qu_x * qo_x - qu_y * qo_y - qu_z * qo_z
    qf_x = qu_w * qo_x + qu_x * qo_w + qu_y * qo_z - qu_z * qo_y
    qf_y = qu_w * qo_y - qu_x * qo_z + qu_y * qo_w + qu_z * qo_x
    qf_z = qu_w * qo_z + qu_x * qo_y - qu_y * qo_x + qu_z * qo_w
    
    # Convert back to Euler
    sinr_cosp = 2 * (qf_w * qf_x + qf_y * qf_z)
    cosr_cosp = 1 - 2 * (qf_x * qf_x + qf_y * qf_y)
    final_roll = math.atan2(sinr_cosp, cosr_cosp)
    
    sinp = 2 * (qf_w * qf_y - qf_z * qf_x)
    if abs(sinp) >= 1:
        final_pitch = math.copysign(math.pi / 2, sinp)
    else:
        final_pitch = math.asin(sinp)
    
    siny_cosp = 2 * (qf_w * qf_z + qf_x * qf_y)
    cosy_cosp = 1 - 2 * (qf_y * qf_y + qf_z * qf_z)
    final_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return final_roll, final_pitch, final_yaw


def hsv_to_rgb(h: float, s: float, v: float) -> Tuple[float, float, float]:
  h = h % 1.0
  i = int(h * 6.0)
  f = h * 6.0 - i
  p = v * (1.0 - s)
  q = v * (1.0 - f * s)
  t = v * (1.0 - (1.0 - f) * s)
  i = i % 6
  if i == 0:
    r, g, b = v, t, p
  elif i == 1:
    r, g, b = q, v, p
  elif i == 2:
    r, g, b = p, v, t
  elif i == 3:
    r, g, b = p, q, v
  elif i == 4:
    r, g, b = t, p, v
  else:
    r, g, b = v, p, q
  return clamp01(r), clamp01(g), clamp01(b)


def quantize_1mm(x: float) -> float:
  return round(x * 1000.0) / 1000.0


def quantize_100mm(x: float) -> float:
  """Quantize to 0.1m (100mm) grid."""
  return round(x * 10.0) / 10.0

class SpawnCamera(Node):
    def __init__(self):
        super().__init__('spawn_camera')

        # Parameters
        self.declare_parameter('num_spheres', 16)
        self.declare_parameter('sphere_radius', 0.05)
        self.declare_parameter('sphere_distance', 5.0)
        self.declare_parameter('random_spheres', False)  # False = use explicit positions, True = random grid
        self.declare_parameter('sphere_positions', '')  # Optional: explicit world-frame positions as YAML string
        self.declare_parameter('camera_update_rate', 30.0)
        self.declare_parameter('camera_initial_pose', [0.0, 0.0, 2.0, 0.0, 0.0, 0.0])  # x y z r p y
        self.declare_parameter('camera_initial_velocity', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # vx vy vz wx wy wz
        # Intrinsics YAML path default to package share config
        pkg_share = get_package_share_directory('floating_camera')
        default_intrinsics = os.path.join(pkg_share, 'config', 'camera_intrinsics.yaml')
        self.declare_parameter('intrinsics_file', default_intrinsics)

        # Load intrinsics
        intrinsics_file = self.get_parameter('intrinsics_file').get_parameter_value().string_value
        try:
            with open(intrinsics_file, 'r') as f:
                intr = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Failed to read intrinsics file {intrinsics_file}: {e}. Falling back to defaults.')
            intr = {}

        img_w = int(intr.get('image_width', 640))
        img_h = int(intr.get('image_height', 480))
        # Try to compute horizontal FOV from intrinsics if fx present, else default to 60 deg
        fx: Optional[float] = None
        try:
            cm = intr.get('camera_matrix', {})
            data = cm.get('data', [])
            if isinstance(data, list) and len(data) >= 1:
                fx = float(data[0])
        except Exception:
            fx = None
        if fx and img_w > 0:
            h_fov = 2.0 * math.atan2(img_w, 2.0 * fx)
        else:
            h_fov = 1.047  # ~60 deg

        # Build camera SDF with ROS camera plugin to load intrinsics
        camera_info_url = f"file://{intrinsics_file}"
        update_rate = float(self.get_parameter('camera_update_rate').get_parameter_value().double_value)
        camera_sdf = f"""
        <?xml version=\"1.0\" ?>
        <sdf version=\"1.6\">
          <model name=\"floating_camera\">
            <static>false</static>
            <allow_auto_disable>false</allow_auto_disable>
            <link name=\"link\">
              <gravity>false</gravity>
              <inertial>
                <mass>0.1</mass>
                <inertia>
                  <ixx>1e-4</ixx><ixy>0</ixy><ixz>0</ixz>
                  <iyy>1e-4</iyy><iyz>0</iyz><izz>1e-4</izz>
                </inertia>
              </inertial>
              <sensor name=\"camera_sensor\" type=\"camera\">
                <always_on>true</always_on>
                <update_rate>{update_rate}</update_rate>
                <visualize>true</visualize>
                <camera>
                  <horizontal_fov>{h_fov}</horizontal_fov>
                  <image>
                    <width>{img_w}</width>
                    <height>{img_h}</height>
                    <format>R8G8B8</format>
                  </image>
                  <clip>
                    <near>0.1</near>
                    <far>100</far>
                  </clip>
                </camera>
                <plugin name=\"gazebo_ros_camera\" filename=\"libgazebo_ros_camera.so\">
                  <ros>
                    <namespace>/floating_camera</namespace>
                    <remapping>image_raw:=image_raw</remapping>
                    <remapping>camera_info:=camera_info</remapping>
                  </ros>
                  <camera_name>floating_camera</camera_name>
                  <frame_name>floating_camera_optical_frame</frame_name>
                  <update_rate>{update_rate}</update_rate>
                  <camera_info_url>{camera_info_url}</camera_info_url>
                </plugin>
              </sensor>
            </link>
          </model>
        </sdf>
        """

        spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not spawn_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity...')

        # Spawn camera
        cam_req = SpawnEntity.Request()
        cam_req.name = 'floating_camera'
        cam_req.xml = camera_sdf
        cam_req.reference_frame = 'world'
        # Parse camera_initial_pose as [x, y, z, angle, ax, ay, az] (axis-angle format)
        init_pose_vals = self._get_list_param('camera_initial_pose', 7, [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0])
        x, y, z, angle, ax, ay, az = init_pose_vals
        
        # Convert axis-angle to quaternion (user specifies in optical frame)
        angle = float(angle)
        axis_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if axis_norm < 1e-9:
            # Zero rotation
            q_user = [0.0, 0.0, 0.0, 1.0]
        else:
            ax_n, ay_n, az_n = ax / axis_norm, ay / axis_norm, az / axis_norm
            half_angle = angle * 0.5
            s = math.sin(half_angle)
            c = math.cos(half_angle)
            q_user = [ax_n * s, ay_n * s, az_n * s, c]
        
        # Apply optical->link transformation: q_link = q_user_optical * q_optical_to_link
        offset_roll = math.pi / 2
        offset_pitch = -math.pi / 2
        offset_yaw = 0.0
        cr = math.cos(offset_roll * 0.5)
        sr = math.sin(offset_roll * 0.5)
        cp = math.cos(offset_pitch * 0.5)
        sp = math.sin(offset_pitch * 0.5)
        cy = math.cos(offset_yaw * 0.5)
        sy = math.sin(offset_yaw * 0.5)
        q_offset = [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
        
        # Quaternion multiplication: q_link = q_user * q_offset
        qux, quy, quz, quw = q_user
        qox, qoy, qoz, qow = q_offset
        q_link = [
            quw * qox + qux * qow + quy * qoz - quz * qoy,
            quw * qoy - qux * qoz + quy * qow + quz * qox,
            quw * qoz + qux * qoy - quy * qox + quz * qow,
            quw * qow - qux * qox - quy * qoy - quz * qoz,
        ]
        
        cam_req.initial_pose.position.x = float(x)
        cam_req.initial_pose.position.y = float(y)
        cam_req.initial_pose.position.z = float(z)
        from geometry_msgs.msg import Quaternion as QMsg
        cam_req.initial_pose.orientation = QMsg(x=q_link[0], y=q_link[1], z=q_link[2], w=q_link[3])
        future = spawn_cli.call_async(cam_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Spawned camera: {future.result().status_message}")
        else:
            self.get_logger().error('Camera spawn failed')

        # Create controller and apply initial velocity
        self._controller = CameraController(self, 'floating_camera', update_hz=update_rate)
        # Ensure controller starts from the initial pose (quaternion [x,y,z,w])
        self._controller.set_pose((float(x), float(y), float(z)), tuple(q_link))
        vel_vals = self._get_list_param('camera_initial_velocity', 6, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._controller.set_velocity((vel_vals[0], vel_vals[1], vel_vals[2]), (vel_vals[3], vel_vals[4], vel_vals[5]))
        self._controller.start()

        # Spawn spheres
        self._spawn_spheres(spawn_cli)

    def _get_list_param(self, name: str, expected_len: int, default: list[float]) -> list[float]:
        """Robustly read list parameters (array or YAML string)."""
        try:
            param = self.get_parameter(name)
            arr = list(param.get_parameter_value().double_array_value)
            if len(arr) == expected_len:
                return [float(v) for v in arr]
        except Exception:
            pass
        try:
            s = self.get_parameter(name).get_parameter_value().string_value
            if s:
                val = yaml.safe_load(s)
                if isinstance(val, (list, tuple)) and len(val) == expected_len:
                    return [float(v) for v in val]
        except Exception:
            pass
        return default

    def _get_positions_param(self) -> list[Tuple[float, float, float]]:
        """Parse sphere_positions parameter as a list of (x,y,z) in world frame.
        
        Accepts a YAML string like: "[[x,y,z], [x,y,z], ...]"
        """
        raw = ''
        try:
            raw = self.get_parameter('sphere_positions').get_parameter_value().string_value
        except Exception:
            pass

        if not raw:
            return []

        try:
            data = yaml.safe_load(raw)
            if isinstance(data, list):
                out: list[Tuple[float, float, float]] = []
                for p in data:
                    if isinstance(p, (list, tuple)) and len(p) == 3:
                        out.append((float(p[0]), float(p[1]), float(p[2])))
                return out
        except Exception as e:
            self.get_logger().warn(f"Failed to parse sphere_positions: {e}")
        return []

    def _spawn_spheres(self, spawn_cli):
        radius = float(self.get_parameter('sphere_radius').get_parameter_value().double_value)
        
        # Check random_spheres flag
        random_spheres = self.get_parameter('random_spheres').get_parameter_value().bool_value
        
        if not random_spheres:
            # Use explicit positions
            explicit_positions = self._get_positions_param()
            if explicit_positions:
                self.get_logger().info(f"Spawning {len(explicit_positions)} spheres at explicit world positions")
                n = len(explicit_positions)
                for idx, (px, py, pz) in enumerate(explicit_positions):
                    # Color spheres with evenly spaced hues
                    hue = (idx / n) if n > 0 else 0.0
                    r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
                    name = f"sphere_{idx:02d}"
                    sdf = f"""
                <?xml version=\"1.0\" ?>
                <sdf version=\"1.6\">
                  <model name=\"{name}\">
                    <static>true</static>
                    <pose>{quantize_1mm(px)} {quantize_1mm(py)} {quantize_1mm(pz)} 0 0 0</pose>
                    <link name=\"link\">
                      <visual name=\"visual\">
                        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
                        <material>
                          <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>
                          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
                          <specular>0 0 0 1</specular>
                          <emissive>0 0 0 1</emissive>
                        </material>
                      </visual>
                      <collision name=\"collision\">
                        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
                      </collision>
                    </link>
                  </model>
                </sdf>
                """
                    req = SpawnEntity.Request()
                    req.name = name
                    req.xml = sdf
                    req.reference_frame = 'world'
                    fut = spawn_cli.call_async(req)
                    rclpy.spin_until_future_complete(self, fut)
                    if fut.result() is not None:
                        self.get_logger().info(f"Spawned {name} @ ({px:.3f}, {py:.3f}, {pz:.3f})")
                    else:
                        self.get_logger().warn(f"Failed to spawn {name}")
                return
            else:
                self.get_logger().warn("random_spheres=False but no sphere_positions provided. No spheres spawned.")
                return

        # Use camera-relative random grid placement
        self.get_logger().info("Spawning spheres using random grid layout")
        n = int(self.get_parameter('num_spheres').get_parameter_value().integer_value)
        dist = float(self.get_parameter('sphere_distance').get_parameter_value().double_value)

        if n <= 0:
            return

        # Camera initial pose (axis-angle format: [x,y,z,angle,ax,ay,az])
        init_pose_vals = self._get_list_param('camera_initial_pose', 7, [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0])
        cx, cy, cz, angle, ax, ay, az = init_pose_vals
        
        # Convert axis-angle to quaternion for optical frame
        angle = float(angle)
        axis_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if axis_norm < 1e-9:
            q_user = [0.0, 0.0, 0.0, 1.0]
        else:
            ax_n, ay_n, az_n = ax / axis_norm, ay / axis_norm, az / axis_norm
            half_angle = angle * 0.5
            s = math.sin(half_angle)
            c = math.cos(half_angle)
            q_user = [ax_n * s, ay_n * s, az_n * s, c]
        
        # Apply optical->link transformation
        offset_roll = math.pi / 2
        offset_pitch = -math.pi / 2
        offset_yaw = 0.0
        cr = math.cos(offset_roll * 0.5)
        sr = math.sin(offset_roll * 0.5)
        cp = math.cos(offset_pitch * 0.5)
        sp = math.sin(offset_pitch * 0.5)
        cy_ang = math.cos(offset_yaw * 0.5)
        sy = math.sin(offset_yaw * 0.5)
        q_offset = [
            sr * cp * cy_ang - cr * sp * sy,
            cr * sp * cy_ang + sr * cp * sy,
            cr * cp * sy - sr * sp * cy_ang,
            cr * cp * cy_ang + sr * sp * sy,
        ]
        
        # q_link = q_user * q_offset
        qux, quy, quz, quw = q_user
        qox, qoy, qoz, qow = q_offset
        q_link = [
            quw * qox + qux * qow + quy * qoz - quz * qoy,
            quw * qoy - qux * qoz + quy * qow + quz * qox,
            quw * qoz + qux * qoy - quy * qox + quz * qow,
            quw * qow - qux * qox - quy * qoy - quz * qoz,
        ]
        
        # Convert quaternion to rotation matrix for sphere placement
        qx, qy, qz, qw = q_link
        R = [
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ]

        # Intrinsics for FOV/aspect
        intrinsics_file = self.get_parameter('intrinsics_file').get_parameter_value().string_value
        img_w, img_h = 640, 480
        h_fov = 1.047
        try:
            with open(intrinsics_file, 'r') as f:
                intr = yaml.safe_load(f)
            img_w = int(intr.get('image_width', img_w))
            img_h = int(intr.get('image_height', img_h))
            cm = intr.get('camera_matrix', {})
            data = cm.get('data', [])
            fx = float(data[0]) if isinstance(data, list) and len(data) >= 1 else None
            if fx and img_w > 0:
                h_fov = 2.0 * math.atan2(img_w, 2.0 * fx)
        except Exception:
            pass
        aspect = (img_w / img_h) if img_h > 0 else (4.0 / 3.0)
        v_fov = 2.0 * math.atan(math.tan(h_fov / 2.0) / aspect)

        # Determine grid layout close to square
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)

        # FOV safe extents at the given distance (apply margin)
        margin = 0.6
        half_w = dist * math.tan(h_fov / 2.0) * margin
        half_h = dist * math.tan(v_fov / 2.0) * margin

        # Grid centers in camera frame (x forward, y left, z up)
        y_min, y_max = -half_w, half_w
        z_min, z_max = -half_h, half_h

        # Set random seed for reproducibility
        random.seed(42)
        
        def grid_local(i, j, sphere_idx):
            if cols > 1:
                y = y_min + (y_max - y_min) * (j / (cols - 1))
            else:
                y = 0.0
            if rows > 1:
                z = z_min + (z_max - z_min) * (i / (rows - 1))
            else:
                z = 0.0
            
            # Randomize depth within range, quantized to 0.1m
            # Keep all spheres at dist or further to ensure visibility
            depth_offset = random.uniform(0.0, 2.0)
            x = quantize_100mm(dist + depth_offset)
            
            # Quantize y and z to 0.1m before transformation
            y = quantize_100mm(y)
            z = quantize_100mm(z)
            
            return x, y, z

        # Rotation matrix R already computed above from q_link
        def cam_to_world(xc, yc, zc):
            wx = R[0][0] * xc + R[0][1] * yc + R[0][2] * zc + cx
            wy = R[1][0] * xc + R[1][1] * yc + R[1][2] * zc + cy
            wz = R[2][0] * xc + R[2][1] * yc + R[2][2] * zc + cz
            return quantize_1mm(wx), quantize_1mm(wy), quantize_1mm(wz)

        spawned = 0
        for i in range(rows):
            for j in range(cols):
                if spawned >= n:
                    break
                # Evenly spaced hues across full spectrum with maximum saturation and value
                hue = (spawned / n) if n > 0 else 0.0
                r, g, b = hsv_to_rgb(hue, 1.0, 1.0)
                lx, ly, lz = grid_local(i, j, spawned)
                px, py, pz = cam_to_world(lx, ly, lz)
                name = f"sphere_{spawned:02d}"
                sdf = f"""
                <?xml version=\"1.0\" ?>
                <sdf version=\"1.6\">
                  <model name=\"{name}\">
                    <static>true</static>
                    <pose>{px} {py} {pz} 0 0 0</pose>
                    <link name=\"link\">
                      <visual name=\"visual\">
                        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
                        <material>
                          <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>
                          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
                          <specular>0 0 0 1</specular>
                          <emissive>0 0 0 1</emissive>
                        </material>
                      </visual>
                      <collision name=\"collision\">
                        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
                      </collision>
                    </link>
                  </model>
                </sdf>
                """
                req = SpawnEntity.Request()
                req.name = name
                req.xml = sdf
                req.reference_frame = 'world'
                fut = spawn_cli.call_async(req)
                rclpy.spin_until_future_complete(self, fut)
                if fut.result() is not None:
                    self.get_logger().info(f"Spawned {name}")
                else:
                    self.get_logger().warn(f"Failed to spawn {name}")
                spawned += 1

def main():
    rclpy.init()
    node = SpawnCamera()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

