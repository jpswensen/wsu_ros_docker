#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, PoseStamped
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
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


def quaternion_to_euler(q: Quaternion) -> Tuple[float, float, float]:
    """Convert quaternion to Euler RPY."""
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def apply_camera_frame_transform(roll: float, pitch: float, yaw: float) -> tuple[float, float, float]:
    """
    Apply constant transformation so that camera's natural orientation
    (Z forward, X right, Y down) corresponds to zero rotation.
    
    This requires a rotation of roll=90°, pitch=-90° to align with world frame.
    We apply this as a pre-rotation to the user's input.
    """
    import math
    
    # Constant offset: roll=pi/2, pitch=-pi/2, yaw=0
    offset_roll = math.pi / 2
    offset_pitch = -math.pi / 2
    offset_yaw = 0.0
    
    # Convert both to quaternions and multiply
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


class CameraController:
    """
    Control a Gazebo camera entity pose and velocity.

    Contract:
            - Inputs:
                - Pose topic (/floating_camera/set_pose): geometry_msgs/Pose in world frame (zeros velocity when received).
                - Velocity topic (/floating_camera/cmd_vel): body-fixed camera OPTICAL frame Twist
                    (Z forward along optical axis, X right, Y down; REP-103 optical frame).
        - Outputs:
            - Applies state using /gazebo/set_entity_state service at a fixed rate.
            - Publishes current pose in world frame (/floating_camera/pose).
            - Publishes current velocity in body-fixed frame (/floating_camera/velocity).
    - If pose is set directly via set_pose(), linear and angular velocities are reset to zero.
    - Call start() to begin periodic updates, and stop() to cancel them.
    """

    def __init__(self, node: Node, entity_name: str, update_hz: float = 30.0):
        self._node = node
        self._entity_name = entity_name
        self._update_period = 1.0 / float(update_hz)

        # State in world frame (link frame)
        # Position and orientation (quaternion [x, y, z, w])
        self._pos = [0.0, 0.0, 1.0]
        self._quat = [0.0, 0.0, 0.0, 1.0]  # Identity quaternion
        self._lin = [0.0, 0.0, 0.0]
        self._ang = [0.0, 0.0, 0.0]

        self._timer = None
        self._last_time: Optional[Time] = None

        # Service client
        self._set_cli = self._node.create_client(SetEntityState, '/gazebo/set_entity_state')
        while not self._set_cli.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info('Waiting for /gazebo/set_entity_state...')

        # Pub/Sub endpoints
        self._vel_pub = self._node.create_publisher(Twist, '/floating_camera/velocity', 10)
        self._pose_pub = self._node.create_publisher(PoseStamped, '/floating_camera/pose', 10)
        self._vel_sub = self._node.create_subscription(Twist, '/floating_camera/cmd_vel', self._on_cmd_vel, 10)
        # Accept external pose commands via topic (Pose in world frame)
        self._set_pose_sub = self._node.create_subscription(Pose, '/floating_camera/set_pose', self._on_set_pose_topic, 10)

    # Public API
    def set_pose(self, position: Tuple[float, float, float], quaternion: Tuple[float, float, float, float]):
        """Set pose with position and quaternion [x, y, z, w] in world link frame."""
        self._pos = [float(position[0]), float(position[1]), float(position[2])]
        self._quat = [float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3])]
        # As requested: zero velocities when pose is set directly
        self._lin = [0.0, 0.0, 0.0]
        self._ang = [0.0, 0.0, 0.0]

    def set_velocity(self, linear_xyz: Tuple[float, float, float], angular_xyz: Tuple[float, float, float]):
        self._lin = [float(linear_xyz[0]), float(linear_xyz[1]), float(linear_xyz[2])]
        self._ang = [float(angular_xyz[0]), float(angular_xyz[1]), float(angular_xyz[2])]

    def start(self):
        if self._timer is None:
            self._last_time = self._node.get_clock().now()
            self._timer = self._node.create_timer(self._update_period, self._on_timer)

    def stop(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    # Internal
    def _on_cmd_vel(self, msg: Twist):
        # Interpret commanded velocity in camera OPTICAL frame (Z forward, X right, Y down)
        # Linear velocity: Convert optical frame to world frame
        R_w_l = self._quat_to_rot(self._quat)
        C_l_o = self._link_from_optical()
        R_w_o = self._matmul33(R_w_l, C_l_o)
        v_o = [msg.linear.x, msg.linear.y, msg.linear.z]
        v_w = self._matvec3(R_w_o, v_o)
        
        # Angular velocity: Keep in BODY frame (optical frame)
        # The quaternion integration dq/dt = 0.5*q*ω expects ω in body frame
        # So we store angular velocity in optical frame, convert to link frame for integration
        w_o = [msg.angular.x, msg.angular.y, msg.angular.z]
        w_l = self._matvec3(C_l_o, w_o)
        self.set_velocity(tuple(v_w), tuple(w_l))

    def _on_set_pose_topic(self, pose: Pose):
        # User specifies rotation in camera's natural OPTICAL frame (Z forward, X right, Y down)
        # We need to compose this with the optical->link transformation
        
        # User's quaternion (rotation in optical frame)
        q_user = pose.orientation
        
        # Camera frame offset quaternion (roll=π/2, pitch=-π/2)
        # This transforms optical frame to link frame
        import math
        offset_roll = math.pi / 2
        offset_pitch = -math.pi / 2
        offset_yaw = 0.0
        
        cr = math.cos(offset_roll * 0.5)
        sr = math.sin(offset_roll * 0.5)
        cp = math.cos(offset_pitch * 0.5)
        sp = math.sin(offset_pitch * 0.5)
        cy = math.cos(offset_yaw * 0.5)
        sy = math.sin(offset_yaw * 0.5)
        
        q_offset_w = cr * cp * cy + sr * sp * sy
        q_offset_x = sr * cp * cy - cr * sp * sy
        q_offset_y = cr * sp * cy + sr * cp * sy
        q_offset_z = cr * cp * sy - sr * sp * cy
        
        # Compose: q_world_link = q_user_optical * q_offset_optical_to_link
        qw = q_user.w * q_offset_w - q_user.x * q_offset_x - q_user.y * q_offset_y - q_user.z * q_offset_z
        qx = q_user.w * q_offset_x + q_user.x * q_offset_w + q_user.y * q_offset_z - q_user.z * q_offset_y
        qy = q_user.w * q_offset_y - q_user.x * q_offset_z + q_user.y * q_offset_w + q_user.z * q_offset_x
        qz = q_user.w * q_offset_z + q_user.x * q_offset_y - q_user.y * q_offset_x + q_user.z * q_offset_w
        
        # Optional diagnostic: compute optical frame orientation in world for logging
        r_user, p_user, y_user = quaternion_to_euler(pose.orientation)
        R_w_l = self._quat_to_rot([qx, qy, qz, qw])
        C_l_o = self._link_from_optical()
        R_w_o = self._matmul33(R_w_l, C_l_o)
        opt_pitch = math.asin(-R_w_o[2][0]) if abs(R_w_o[2][0]) <= 1.0 else math.copysign(math.pi/2, -R_w_o[2][0])
        if abs(math.cos(opt_pitch)) < 1e-6:
            opt_roll = 0.0
            opt_yaw = math.atan2(-R_w_o[0][1], R_w_o[1][1])
        else:
            opt_roll = math.atan2(R_w_o[2][1], R_w_o[2][2])
            opt_yaw = math.atan2(R_w_o[1][0], R_w_o[0][0])
        self._node.get_logger().info(
            f'set_pose user (optical): quat=[{q_user.x:.3f}, {q_user.y:.3f}, {q_user.z:.3f}, {q_user.w:.3f}] '
            f'RPY=({r_user:.3f}, {p_user:.3f}, {y_user:.3f}) -> '
            f'world (link): quat=[{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}] | '
            f'world (optical): RPY=({opt_roll:.3f}, {opt_pitch:.3f}, {opt_yaw:.3f})'
        )
        
        # Set pose with quaternion [x, y, z, w]; velocity is zeroed by set_pose()
        self.set_pose((pose.position.x, pose.position.y, pose.position.z), (qx, qy, qz, qw))

    def _on_timer(self):
        now = self._node.get_clock().now()
        dt = 0.0
        if self._last_time is not None:
            dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        # Integrate linear velocity in world frame
        self._pos[0] += self._lin[0] * dt
        self._pos[1] += self._lin[1] * dt
        self._pos[2] += self._lin[2] * dt

        # Integrate angular velocity in BODY frame (link frame) using quaternion derivative
        # dq/dt = 0.5 * q * omega_body (where omega_body is in link frame)
        # self._ang is already in link frame (body-fixed)
        qx, qy, qz, qw = self._quat
        wx, wy, wz = self._ang
        # Quaternion derivative: dq = 0.5 * q * [wx, wy, wz, 0]
        # This represents body-frame angular velocity
        dqx = 0.5 * (qw * wx + qy * wz - qz * wy)
        dqy = 0.5 * (qw * wy + qz * wx - qx * wz)
        dqz = 0.5 * (qw * wz + qx * wy - qy * wx)
        dqw = 0.5 * (-qx * wx - qy * wy - qz * wz)
        
        self._quat[0] += dqx * dt
        self._quat[1] += dqy * dt
        self._quat[2] += dqz * dt
        self._quat[3] += dqw * dt
        
        # Renormalize to prevent drift
        norm = math.sqrt(self._quat[0]**2 + self._quat[1]**2 + self._quat[2]**2 + self._quat[3]**2)
        if norm > 1e-9:
            self._quat[0] /= norm
            self._quat[1] /= norm
            self._quat[2] /= norm
            self._quat[3] /= norm

        # Apply state to Gazebo
        # Convert body-frame angular velocity to world frame for Gazebo
        R_w_l = self._quat_to_rot(self._quat)
        w_l = self._ang  # Angular velocity in link (body) frame
        w_w = self._matvec3(R_w_l, w_l)  # Convert to world frame
        
        state = EntityState()
        state.name = self._entity_name
        state.reference_frame = 'world'
        state.pose = Pose(
            position=Point(x=self._pos[0], y=self._pos[1], z=self._pos[2]),
            orientation=Quaternion(x=self._quat[0], y=self._quat[1], z=self._quat[2], w=self._quat[3])
        )
        state.twist = Twist(
            linear=Vector3(x=self._lin[0], y=self._lin[1], z=self._lin[2]),
            angular=Vector3(x=w_w[0], y=w_w[1], z=w_w[2])
        )

        req = SetEntityState.Request()
        req.state = state

        future = self._set_cli.call_async(req)
        # Fire-and-forget; optionally log on failure
        def _done_cb(fut):
            result = fut.result()
            if result is not None and not result.success:
                self._node.get_logger().warn(f"set_entity_state failed: {result.status_message}")

        future.add_done_callback(_done_cb)

        # Publish current velocity and pose at the same rate as updates
        # Linear velocity: Convert world frame to optical frame
        # Angular velocity: self._ang is in link (body) frame, convert to optical frame
        # Reuse R_w_l from above
        R_l_w = self._transpose(R_w_l)
        O_o_l = self._optical_from_link()
        O_o_w = self._matmul33(O_o_l, R_l_w)
        v_w = self._lin
        v_o = self._matvec3(O_o_w, v_w)
        # Angular velocity: convert from link (body) frame to optical frame
        w_o = self._matvec3(O_o_l, w_l)
        self._vel_pub.publish(Twist(
            linear=Vector3(x=v_o[0], y=v_o[1], z=v_o[2]),
            angular=Vector3(x=w_o[0], y=w_o[1], z=w_o[2])
        ))

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose = state.pose
        self._pose_pub.publish(pose_msg)

    # Rotation utilities
    def _quat_to_rot(self, q):
        """Convert quaternion [x, y, z, w] to rotation matrix."""
        qx, qy, qz, qw = q
        return [
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ]

    def _transpose(self, R):
        """Transpose a 3x3 matrix."""
        return [
            [R[0][0], R[1][0], R[2][0]],
            [R[0][1], R[1][1], R[2][1]],
            [R[0][2], R[1][2], R[2][2]],
        ]

    def _link_from_optical(self):
        # Fixed rotation from optical frame (Z forward, X right, Y down) to link frame (X forward, Y left, Z up)
        # Desired mapping: opt X -> link -Y, opt Y -> link -Z, opt Z -> link +X
        # Columns are images of basis vectors in optical frame
        # link_from_optical = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        return [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ]

    def _optical_from_link(self):
        # Inverse (transpose) of link_from_optical above
        return [
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
        ]

    def _matmul33(self, A, B):
        return [
            [A[0][0]*B[0][0] + A[0][1]*B[1][0] + A[0][2]*B[2][0], A[0][0]*B[0][1] + A[0][1]*B[1][1] + A[0][2]*B[2][1], A[0][0]*B[0][2] + A[0][1]*B[1][2] + A[0][2]*B[2][2]],
            [A[1][0]*B[0][0] + A[1][1]*B[1][0] + A[1][2]*B[2][0], A[1][0]*B[0][1] + A[1][1]*B[1][1] + A[1][2]*B[2][1], A[1][0]*B[0][2] + A[1][1]*B[1][2] + A[1][2]*B[2][2]],
            [A[2][0]*B[0][0] + A[2][1]*B[1][0] + A[2][2]*B[2][0], A[2][0]*B[0][1] + A[2][1]*B[1][1] + A[2][2]*B[2][1], A[2][0]*B[0][2] + A[2][1]*B[1][2] + A[2][2]*B[2][2]],
        ]

    def _matvec3(self, A, v):
        return [
            A[0][0]*v[0] + A[0][1]*v[1] + A[0][2]*v[2],
            A[1][0]*v[0] + A[1][1]*v[1] + A[1][2]*v[2],
            A[2][0]*v[0] + A[2][1]*v[1] + A[2][2]*v[2],
        ]
