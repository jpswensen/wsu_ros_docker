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

        # State in world frame
        self._pos = [0.0, 0.0, 1.0]
        self._rpy = [0.0, 0.0, 0.0]
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
    def set_pose(self, position: Tuple[float, float, float], rpy: Tuple[float, float, float]):
        self._pos = [float(position[0]), float(position[1]), float(position[2])]
        self._rpy = [float(rpy[0]), float(rpy[1]), float(rpy[2])]
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
        # Convert to world frame using: world_from_optical = world_from_link * link_from_optical
        R_w_l = self._rpy_to_rot(self._rpy[0], self._rpy[1], self._rpy[2])
        C_l_o = self._link_from_optical()
        R_w_o = self._matmul33(R_w_l, C_l_o)
        v_o = [msg.linear.x, msg.linear.y, msg.linear.z]
        w_o = [msg.angular.x, msg.angular.y, msg.angular.z]
        v_w = self._matvec3(R_w_o, v_o)
        w_w = self._matvec3(R_w_o, w_o)
        self.set_velocity(tuple(v_w), tuple(w_w))

    def _on_set_pose_topic(self, pose: Pose):
        # Extract pose from topic and set; velocity is zeroed by set_pose()
        r, p, y = quaternion_to_euler(pose.orientation)
        self.set_pose((pose.position.x, pose.position.y, pose.position.z), (r, p, y))

    def _on_timer(self):
        now = self._node.get_clock().now()
        dt = 0.0
        if self._last_time is not None:
            dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        # Integrate simple kinematics in world frame
        self._pos[0] += self._lin[0] * dt
        self._pos[1] += self._lin[1] * dt
        self._pos[2] += self._lin[2] * dt

        self._rpy[0] += self._ang[0] * dt
        self._rpy[1] += self._ang[1] * dt
        self._rpy[2] += self._ang[2] * dt

        # Apply state to Gazebo
        state = EntityState()
        state.name = self._entity_name
        state.reference_frame = 'world'
        state.pose = Pose(
            position=Point(x=self._pos[0], y=self._pos[1], z=self._pos[2]),
            orientation=euler_to_quaternion(self._rpy[0], self._rpy[1], self._rpy[2])
        )
        state.twist = Twist(
            linear=Vector3(x=self._lin[0], y=self._lin[1], z=self._lin[2]),
            angular=Vector3(x=self._ang[0], y=self._ang[1], z=self._ang[2])
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
        # Velocity topic is body-fixed; rotate world velocity back to body frame
        # Build optical_from_world = optical_from_link * link_from_world
        R_l_w = self._rot_transpose(self._rpy[0], self._rpy[1], self._rpy[2])
        O_o_l = self._optical_from_link()
        O_o_w = self._matmul33(O_o_l, R_l_w)
        v_w = self._lin
        w_w = self._ang
        v_o = self._matvec3(O_o_w, v_w)
        w_o = self._matvec3(O_o_w, w_w)
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
    def _rpy_to_rot(self, roll: float, pitch: float, yaw: float):
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr               ],
        ]

    def _rot_transpose(self, roll: float, pitch: float, yaw: float):
        R = self._rpy_to_rot(roll, pitch, yaw)
        # Transpose of 3x3
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
