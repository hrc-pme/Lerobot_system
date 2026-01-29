#!/usr/bin/env python3
"""
MoveIt marker bridge
====================

Lightweight helper that lets the vision GUI reposition the MoveIt interactive
marker and trigger plan/execute cycles directly from Python. It is designed to
run inside the same process as the GUI (no separate ROS launch required).

The bridge performs a few core tasks:
 - Publish ``InteractiveMarkerFeedback`` messages so the RViz marker jumps to
   a desired pose.
 - Call ``/compute_ik`` to turn the requested pose into joint positions.
 - Use the ``MoveGroup`` action server to plan (plan_only=True) or plan+execute
   (plan_only=False) with those joints.
 - Cache the latest successful goal so Execute can reuse the last IK result.
 - Cancel an active MoveGroup action when the user presses Stop.

Only a minimal subset of MoveIt is exposed here – more advanced behaviours
should still be handled in dedicated ROS nodes.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Sequence, Tuple

import numpy as np
import os
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate


@dataclass
class BridgeResult:
    """Simple container for plan/execute results."""

    success: bool
    message: str = ""

    @classmethod
    def ok(cls, message: str = "") -> "BridgeResult":
        return cls(True, message)

    @classmethod
    def fail(cls, message: str) -> "BridgeResult":
        return cls(False, message)


_RCLPY_LOCK = threading.Lock()
_RCLPY_INITIALIZED = False


def _ensure_rclpy_init() -> None:
    global _RCLPY_INITIALIZED
    with _RCLPY_LOCK:
        if not _RCLPY_INITIALIZED:
            rclpy.init(args=None)
            _RCLPY_INITIALIZED = True


def _now(node: Node):
    return node.get_clock().now().to_msg()


def _pose_copy(pose: Pose) -> Pose:
    q = Pose()
    q.position.x = pose.position.x
    q.position.y = pose.position.y
    q.position.z = pose.position.z
    q.orientation.x = pose.orientation.x
    q.orientation.y = pose.orientation.y
    q.orientation.z = pose.orientation.z
    q.orientation.w = pose.orientation.w
    return q


def _quat_normalize(q: Sequence[float]) -> np.ndarray:
    arr = np.array(q, dtype=np.float64)
    norm = np.linalg.norm(arr)
    if norm <= 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return arr / norm


def _quat_conjugate(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=np.float64)


def _quat_multiply(q1: Sequence[float], q2: Sequence[float]) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([x, y, z, w], dtype=np.float64)


class MoveItMarkerBridge:
    """Convenience wrapper around MoveIt interactive markers."""

    def __init__(
        self,
        robot_name: str,
        planning_group: str = "interbotix_arm",
        eef_link: Optional[str] = None,
        *,
        joint_order: Optional[Sequence[str]] = None,
        camera_frame: str = "camera_color_optical_frame",
        target_frame: str = "world",
        marker_update_topic: str = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
        marker_feedback_topic: str = "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
        move_group_action: str = "/move_action",
        compute_ik_service: str = "/compute_ik",
        compute_cartesian_path_service: str = "/compute_cartesian_path",
        marker_client_id: str = "vlservo_gui",
        joint_tolerance: float = math.radians(1.0),
        planning_attempts: int = 5,
        allowed_planning_time: float = 5.0,
        velocity_scaling: float = 0.2,
        acceleration_scaling: float = 0.2,
    ) -> None:
        _ensure_rclpy_init()

        self.robot_name = robot_name
        self.planning_group = planning_group
        self.eef_link = eef_link or f"{robot_name}/ee_gripper_link"
        self.camera_frame = camera_frame
        self.target_frame = target_frame
        self.marker_client_id = marker_client_id
        self.joint_order = list(joint_order) if joint_order else [
            "waist",
            "shoulder",
            "elbow",
            "forearm_roll",
            "wrist_angle",
            "wrist_rotate",
        ]
        self.joint_tolerance = float(joint_tolerance)
        self.planning_attempts = int(max(1, planning_attempts))
        self.allowed_planning_time = float(max(0.1, allowed_planning_time))
        # Use moderate scaling for smooth, controlled motions (same as RViz defaults)
        self.velocity_scaling = float(np.clip(velocity_scaling, 0.0, 1.0))
        self.acceleration_scaling = float(np.clip(acceleration_scaling, 0.0, 1.0))

        self._node = Node("moveit_marker_bridge")
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self._node,
            spin_thread=True,
        )
        self._static_broadcaster = StaticTransformBroadcaster(self._node)

        self.camera_parent_frame = os.getenv('MOVEIT_CAMERA_PARENT_FRAME', f"{self.robot_name}/gripper_link")
        self.camera_offset = self._parse_vector_env('MOVEIT_CAMERA_OFFSET', default=(-0.13, -0.01, 0.05))
        self.camera_quaternion = self._parse_vector_env('MOVEIT_CAMERA_QUATERNION', default=(0.0, 0.0, 0.0, 1.0))
        self.broadcast_camera_tf = os.getenv('MOVEIT_CAMERA_BROADCAST', 'true').lower() not in ('0', 'false', 'no')
        if self.broadcast_camera_tf:
            self._broadcast_camera_static_tf()

        self._marker_feedback_pub = self._node.create_publisher(
            InteractiveMarkerFeedback,
            marker_feedback_topic,
            10,
        )
        self._marker_update_sub = self._node.create_subscription(
            InteractiveMarkerUpdate,
            marker_update_topic,
            self._on_marker_update,
            10,
        )

        self._move_group_client = ActionClient(self._node, MoveGroup, move_group_action)
        self._compute_ik_client = self._node.create_client(GetPositionIK, compute_ik_service)
        self._cartesian_path_client = self._node.create_client(GetCartesianPath, compute_cartesian_path_service)

        self._marker_name: Optional[str] = None
        self._marker_pose: Optional[Pose] = None
        self._marker_frame: Optional[str] = None
        self._marker_ready = threading.Event()
        self._active_goal_handle = None

        self._last_joint_positions: Optional[Sequence[float]] = None
        self._last_pose_target: Optional[PoseStamped] = None

        self._joint_state_lock = threading.Lock()
        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_sub = self._node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            50,
        )
        self._arm_follow_client = None

        # Initialize Interbotix robot for gripper control (lazy initialization)
        self._interbotix_bot = None
        self._robot_model = robot_name if robot_name else "vx300s"
        self._interbotix_init_attempted = False

    # ------------------------------------------------------------------ utils
    def _await_future(self, future, timeout: float = 5.0):
        deadline = time.monotonic() + timeout if timeout else None
        while rclpy.ok():
            if future.done():
                return future.result()
            time.sleep(0.01)
            if deadline and time.monotonic() > deadline:
                break
        return None

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            with self._joint_state_lock:
                self._latest_joint_state = JointState(
                    name=list(msg.name),
                    position=list(msg.position),
                    velocity=list(msg.velocity) if msg.velocity else [],
                    effort=list(msg.effort) if msg.effort else [],
                )
        except Exception:
            pass

    def _get_current_joint_positions(self) -> Optional[Sequence[float]]:
        with self._joint_state_lock:
            msg = self._latest_joint_state
        if msg is None:
            return None
        name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
        positions = []
        try:
            for joint in self.joint_order:
                positions.append(float(name_to_pos[joint]))
        except KeyError:
            return None
        return positions

    def _make_robot_state(self) -> Optional[RobotState]:
        positions = self._get_current_joint_positions()
        if positions is None:
            return None
        state = RobotState()
        state.joint_state.name = list(self.joint_order)
        state.joint_state.position = list(positions)
        return state

    def _ensure_arm_follow_client(self):
        if self._arm_follow_client is not None:
            return self._arm_follow_client
        try:
            from control_msgs.action import FollowJointTrajectory
            self._arm_follow_client = ActionClient(
                self._node,
                FollowJointTrajectory,
                f'/{self.robot_name}/arm_controller/follow_joint_trajectory',
            )
        except Exception:
            self._arm_follow_client = None
        return self._arm_follow_client

    def _on_marker_update(self, update: InteractiveMarkerUpdate) -> None:
        try:
            # Update poses list
            for pose_msg in update.poses:
                if pose_msg.name:
                    self._marker_name = pose_msg.name
                    self._marker_frame = pose_msg.header.frame_id or self.target_frame
                    self._marker_pose = _pose_copy(pose_msg.pose)
                    self._marker_ready.set()
            for marker in update.markers:
                if marker.name:
                    self._marker_name = marker.name
                    self._marker_frame = marker.header.frame_id or self.target_frame
                    self._marker_pose = _pose_copy(marker.pose)
                    self._marker_ready.set()
        except Exception:
            pass

    # ---------------------------------------------------------------- pose ops
    def _ensure_marker_ready(self, timeout: float = 2.0) -> bool:
        if self._marker_ready.is_set():
            return True
        return self._marker_ready.wait(timeout=timeout)

    def _prepare_pose(
        self,
        position: Sequence[float],
        *,
        source_frame: Optional[str] = None,
        orientation: Optional[Sequence[float]] = None,
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = source_frame or self.camera_frame
        pose.header.stamp = _now(self._node)
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        if orientation is None:
            q = Quaternion()
            q.w = 1.0
        else:
            q = Quaternion()
            q.x = float(orientation[0])
            q.y = float(orientation[1])
            q.z = float(orientation[2])
            q.w = float(orientation[3])
        # ensure normalized quaternion
        quat = _quat_normalize([q.x, q.y, q.z, q.w])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose

    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        if pose.header.frame_id == target_frame:
            return pose
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            pose_in_target = PoseStamped()
            pose_in_target.header.frame_id = target_frame
            pose_in_target.header.stamp = pose.header.stamp

            # Apply rotation
            q_parent = transform.transform.rotation
            q_local = pose.pose.orientation
            quat_parent = np.array([q_parent.x, q_parent.y, q_parent.z, q_parent.w], dtype=np.float64)
            quat_local = np.array([q_local.x, q_local.y, q_local.z, q_local.w], dtype=np.float64)
            quat_target = _quat_multiply(quat_parent, quat_local)
            quat_target = _quat_normalize(quat_target)

            pose_in_target.pose.orientation.x = quat_target[0]
            pose_in_target.pose.orientation.y = quat_target[1]
            pose_in_target.pose.orientation.z = quat_target[2]
            pose_in_target.pose.orientation.w = quat_target[3]

            # Apply translation
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # Rotate original translation by parent orientation
            # Using quaternion multiplication: t' = q * p * q_conj
            p = [x, y, z, 0.0]
            q_parent_conj = _quat_conjugate(quat_parent)
            rotated = _quat_multiply(quat_parent, p)
            rotated = _quat_multiply(rotated, q_parent_conj)

            pose_in_target.pose.position.x = float(rotated[0] + tx)
            pose_in_target.pose.position.y = float(rotated[1] + ty)
            pose_in_target.pose.position.z = float(rotated[2] + tz)
            return pose_in_target
        except (LookupException, ConnectivityException, ExtrapolationException, TransformException):
            # Fall back to assuming frames are identical
            fallback = PoseStamped()
            fallback.header.frame_id = target_frame
            fallback.header.stamp = pose.header.stamp
            fallback.pose = pose.pose
            return fallback

    def _publish_marker(self, pose: Pose) -> None:
        if not self._marker_name:
            return
        msg = InteractiveMarkerFeedback()
        msg.header.frame_id = self._marker_frame or self.target_frame
        msg.header.stamp = _now(self._node)
        msg.client_id = self.marker_client_id
        msg.marker_name = self._marker_name
        msg.event_type = InteractiveMarkerFeedback.POSE_UPDATE
        msg.pose = pose
        self._marker_feedback_pub.publish(msg)

    # ------------------------------------------------------------ MoveIt core
    def _compute_cartesian_path(self, waypoints: Sequence[Pose], max_step: float, jump_threshold: float = 0.0):
        if not waypoints:
            return None
        if not self._cartesian_path_client.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().warn("Cartesian path service unavailable")
            return None
        request = GetCartesianPath.Request()
        request.header.frame_id = self.target_frame
        request.header.stamp = _now(self._node)
        robot_state = self._make_robot_state()
        if robot_state is not None:
            request.start_state = robot_state
        request.group_name = self.planning_group
        request.link_name = self.eef_link
        request.waypoints = list(waypoints)
        request.max_step = float(max_step)
        request.jump_threshold = float(jump_threshold)
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = True

        future = self._cartesian_path_client.call_async(request)
        response = self._await_future(future, timeout=5.0)
        return response

    def _retime_joint_trajectory(self, trajectory, path_length: float) -> None:
        """Retime trajectory with smooth velocity profile and proper velocity/acceleration."""
        if trajectory is None or not trajectory.points:
            return
        
        num_joints = len(trajectory.joint_names)
        if num_joints == 0:
            return
        
        # Calculate distances between waypoints
        total_distance = 0.0
        prev = None
        distances = []
        for idx, point in enumerate(trajectory.points):
            current = np.array(point.positions, dtype=np.float64)
            if prev is None:
                distances.append(0.0)
            else:
                delta = np.linalg.norm(current - prev)
                distances.append(delta)
                total_distance += delta
            prev = current
        
        if total_distance <= 1e-6:
            total_distance = max(path_length, 1e-3)
        
        # Use controlled speed for smooth grasping motions
        # Target speed: 0.05 m/s (5 cm/s) for smooth approach
        target_speed = 0.05  # m/s
        total_time = max(2.0, path_length / target_speed)  # Minimum 2 seconds for stability
        
        if len(trajectory.points) == 1:
            secs = int(total_time)
            nsecs = int((total_time - secs) * 1e9)
            trajectory.points[0].time_from_start = DurationMsg(sec=secs, nanosec=nsecs)
            # Set zero velocities and accelerations
            trajectory.points[0].velocities = [0.0] * num_joints
            trajectory.points[0].accelerations = [0.0] * num_joints
            return
        
        # Distribute time proportionally to distances
        denom = sum(distances) if sum(distances) > 1e-6 else float(len(distances) or 1)
        times = [0.0]
        cumulative = 0.0
        for idx in range(1, len(trajectory.points)):
            frac = distances[idx] / denom
            cumulative += total_time * frac
            times.append(cumulative)
        
        # Compute velocities and accelerations using finite differences
        for idx, point in enumerate(trajectory.points):
            # Set time from start
            secs = int(times[idx])
            nsecs = int((times[idx] - secs) * 1e9)
            point.time_from_start = DurationMsg(sec=secs, nanosec=nsecs)
            
            # Initialize velocity and acceleration arrays
            velocities = [0.0] * num_joints
            accelerations = [0.0] * num_joints
            
            if idx == 0:
                # First point: compute forward velocity
                if len(trajectory.points) > 1:
                    dt = times[1] - times[0]
                    if dt > 1e-6:
                        for j in range(num_joints):
                            dq = trajectory.points[1].positions[j] - point.positions[j]
                            velocities[j] = dq / dt
            elif idx == len(trajectory.points) - 1:
                # Last point: zero velocity (stop)
                velocities = [0.0] * num_joints
            else:
                # Middle points: central difference for velocity
                dt_prev = times[idx] - times[idx - 1]
                dt_next = times[idx + 1] - times[idx]
                dt_avg = (dt_prev + dt_next) / 2.0
                if dt_avg > 1e-6:
                    for j in range(num_joints):
                        dq = trajectory.points[idx + 1].positions[j] - trajectory.points[idx - 1].positions[j]
                        velocities[j] = dq / (dt_prev + dt_next)
            
            # Compute accelerations (simplified: assume smooth interpolation)
            if idx > 0 and idx < len(trajectory.points) - 1:
                dt = times[idx] - times[idx - 1]
                if dt > 1e-6:
                    prev_point = trajectory.points[idx - 1]
                    for j in range(num_joints):
                        # Use velocity difference for acceleration
                        if hasattr(prev_point, 'velocities') and len(prev_point.velocities) > j:
                            dv = velocities[j] - prev_point.velocities[j]
                            accelerations[j] = dv / dt
            
            point.velocities = velocities
            point.accelerations = accelerations

    def _execute_arm_trajectory(self, trajectory) -> BridgeResult:
        if trajectory is None or not trajectory.points:
            return BridgeResult.fail("Cartesian path trajectory empty")
        client = self._ensure_arm_follow_client()
        if client is None:
            return BridgeResult.fail("Arm controller client unavailable")
        if not client.wait_for_server(timeout_sec=5.0):
            return BridgeResult.fail("Arm controller action server unavailable")

        from control_msgs.action import FollowJointTrajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        try:
            goal.trajectory.header.stamp = _now(self._node)
        except Exception:
            pass

        goal_future = client.send_goal_async(goal)
        goal_handle = self._await_future(goal_future, timeout=5.0)
        if goal_handle is None or not goal_handle.accepted:
            return BridgeResult.fail("Arm trajectory goal rejected")

        result_future = goal_handle.get_result_async()
        result = self._await_future(result_future, timeout=20.0)
        if result is None:
            return BridgeResult.fail("Arm trajectory execution timeout")
        status = getattr(result, "status", GoalStatus.STATUS_UNKNOWN)
        if status != GoalStatus.STATUS_SUCCEEDED:
            error_msg = ""
            try:
                error_msg = getattr(result.result, "error_string", "")
            except Exception:
                pass
            extra = f" ({error_msg})" if error_msg else ""
            return BridgeResult.fail(f"Arm trajectory execution failed (status {status}){extra}")
        return BridgeResult.ok("Arm trajectory executed")

    def _move_linear_cartesian(self, delta_target: np.ndarray, direction_label: str) -> Optional[BridgeResult]:
        magnitude = float(np.linalg.norm(delta_target))
        if magnitude <= 1e-6:
            return BridgeResult.ok("Zero displacement")
        try:
            current_transform = self._tf_buffer.lookup_transform(
                self.target_frame,
                self.eef_link,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:
            self._node.get_logger().warn(f"Failed to get current EEF pose for Cartesian move: {exc}")
            return None

        current_pose = PoseStamped()
        current_pose.header.frame_id = self.target_frame
        current_pose.header.stamp = _now(self._node)
        current_pose.pose.position.x = current_transform.transform.translation.x
        current_pose.pose.position.y = current_transform.transform.translation.y
        current_pose.pose.position.z = current_transform.transform.translation.z
        current_pose.pose.orientation = current_transform.transform.rotation

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.target_frame
        target_pose.header.stamp = _now(self._node)
        target_pose.pose.position.x = current_pose.pose.position.x + float(delta_target[0])
        target_pose.pose.position.y = current_pose.pose.position.y + float(delta_target[1])
        target_pose.pose.position.z = current_pose.pose.position.z + float(delta_target[2])
        target_pose.pose.orientation = current_pose.pose.orientation

        # Use smaller step size for smoother trajectory (0.002m = 2mm per waypoint)
        response = self._compute_cartesian_path([target_pose.pose], max_step=0.002)
        if response is None:
            return None
        fraction = float(getattr(response, "fraction", 0.0))
        if response.error_code.val != MoveItErrorCodes.SUCCESS or fraction < 0.999:
            self._node.get_logger().warn(
                f"Cartesian path for {direction_label} incomplete "
                f"(fraction={fraction:.2f}, code={response.error_code.val})"
            )
            return None

        trajectory = getattr(response, "solution", None)
        if trajectory is None or not trajectory.joint_trajectory.points:
            self._node.get_logger().warn("Cartesian path returned empty trajectory")
            return None

        # Don't retime - let MoveIt/ros2_control handle timing via time parameterization
        # The trajectory already has proper waypoints from compute_cartesian_path
        
        # Extract final joint positions to use with MoveGroup action
        if trajectory.joint_trajectory.points:
            final_positions = trajectory.joint_trajectory.points[-1].positions
            if final_positions:
                self._last_joint_positions = list(final_positions)
                self._last_pose_target = target_pose
                # Use MoveGroup action for smooth execution (like RViz Execute button)
                return self._send_move_group_goal(final_positions, plan_only=False)
        
        return BridgeResult.fail("Cartesian path has no trajectory points")

    def _compute_ik(self, pose_stamped: PoseStamped):
        if not self._compute_ik_client.wait_for_service(timeout_sec=2.0):
            return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.planning_group
        req.ik_request.ik_link_name = self.eef_link
        req.ik_request.pose_stamped = pose_stamped
        seed_state = RobotState()
        seed_state.is_diff = False
        seed_state.joint_state.name = list(self.joint_order)
        current_positions = self._get_current_joint_positions()
        if current_positions is not None and len(current_positions) == len(self.joint_order):
            seed_positions = [float(v) for v in current_positions]
        elif self._last_joint_positions is not None and len(self._last_joint_positions) == len(self.joint_order):
            seed_positions = [float(v) for v in self._last_joint_positions]
        else:
            seed_positions = [0.0] * len(self.joint_order)
        seed_state.joint_state.position = seed_positions
        req.ik_request.robot_state = seed_state
        req.ik_request.timeout = DurationMsg(sec=1, nanosec=0)
        future = self._compute_ik_client.call_async(req)
        response = self._await_future(future, timeout=3.0)
        return response

    def _build_joint_vector(self, joint_state) -> Optional[Sequence[float]]:
        name_to_position: Dict[str, float] = {
            n: p for n, p in zip(joint_state.name, joint_state.position)
        }
        joint_values = []
        for joint in self.joint_order:
            if joint not in name_to_position:
                return None
            joint_values.append(name_to_position[joint])
        return joint_values

    def _constraints_from_joint_positions(self, joints: Sequence[float]) -> Constraints:
        constraints = Constraints()
        for joint_name, position in zip(self.joint_order, joints):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.weight = 1.0
            jc.tolerance_above = self.joint_tolerance
            jc.tolerance_below = self.joint_tolerance
            constraints.joint_constraints.append(jc)
        return constraints

    def _send_move_group_goal(self, joint_positions: Sequence[float], *, plan_only: bool) -> BridgeResult:
        self._node.get_logger().info("Waiting for MoveGroup action server...")
        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            self._node.get_logger().error("MoveGroup action server not available after 10 seconds")
            return BridgeResult.fail("MoveGroup action server not available")

        self._node.get_logger().info("MoveGroup action server connected")
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.allowed_planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        goal.request.goal_constraints = [self._constraints_from_joint_positions(joint_positions)]
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0
        goal.planning_options.replan_delay = 0.0
        goal.planning_options.look_around_attempts = 0
        goal.planning_options.max_safe_execution_cost = 0.0

        goal_future = self._move_group_client.send_goal_async(goal)
        goal_handle = self._await_future(goal_future, timeout=10.0)
        if goal_handle is None:
            return BridgeResult.fail("MoveGroup did not accept goal (timeout)")
        if not goal_handle.accepted:
            return BridgeResult.fail("MoveGroup goal rejected")

        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        client_result = self._await_future(result_future, timeout=30.0)
        self._active_goal_handle = None
        if client_result is None:
            return BridgeResult.fail("MoveGroup result timeout")

        result: MoveGroup.Result = client_result.result
        error = result.error_code.val
        if error != MoveItErrorCodes.SUCCESS:
            return BridgeResult.fail(f"MoveGroup error code: {error}")
        return BridgeResult.ok("MoveGroup succeeded")

    # --------------------------------------------------------------- public API
    def plan_to_xyz(
        self,
        position: Sequence[float],
        *,
        source_frame: Optional[str] = None,
        orientation: Optional[Sequence[float]] = None,
        target_orientation: Optional[Sequence[float]] = None,
    ) -> BridgeResult:
        if not self._ensure_marker_ready():
            return BridgeResult.fail("Interactive marker not ready in RViz")

        pose_in_source = self._prepare_pose(position, source_frame=source_frame, orientation=orientation)
        pose_in_target = self._transform_pose(pose_in_source, self.target_frame)
        if target_orientation is not None:
            q = _quat_normalize(target_orientation)
            pose_in_target.pose.orientation.x = float(q[0])
            pose_in_target.pose.orientation.y = float(q[1])
            pose_in_target.pose.orientation.z = float(q[2])
            pose_in_target.pose.orientation.w = float(q[3])

        self._publish_marker(pose_in_target.pose)

        ik_response = self._compute_ik(pose_in_target)
        if ik_response is None:
            return BridgeResult.fail("IK service unavailable")
        if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
            return BridgeResult.fail(f"IK failure (code {ik_response.error_code.val})")

        joint_positions = self._build_joint_vector(ik_response.solution.joint_state)
        if joint_positions is None:
            return BridgeResult.fail("IK response missing required joints")

        self._last_joint_positions = list(joint_positions)
        self._last_pose_target = pose_in_target
        return self._send_move_group_goal(joint_positions, plan_only=True)

    def execute_cached_goal(self) -> BridgeResult:
        if self._last_joint_positions is None:
            return BridgeResult.fail("No cached MoveIt plan – run Plan first")
        return self._send_move_group_goal(self._last_joint_positions, plan_only=False)

    def cancel_active_goal(self) -> BridgeResult:
        handle = self._active_goal_handle
        if handle is None:
            return BridgeResult.fail("No active MoveGroup goal")
        cancel_future = handle.cancel_goal_async()
        result = self._await_future(cancel_future, timeout=2.0)
        if result is None:
            return BridgeResult.fail("Failed to cancel goal (timeout)")
        return BridgeResult.ok("Cancel request sent")

    # ------------------------------------------------------------ gripper control
    def _ensure_interbotix_initialized(self) -> bool:
        """Lazy initialization of InterbotixManipulatorXS to avoid import issues at startup."""
        if self._interbotix_bot is not None:
            return True
        if self._interbotix_init_attempted:
            return False
        
        self._interbotix_init_attempted = True
        try:
            from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
            # Let InterbotixManipulatorXS create its own node
            # This avoids compatibility issues with node methods
            self._interbotix_bot = InterbotixManipulatorXS(
                robot_model=self._robot_model,
                group_name='arm',
                gripper_name='gripper',
            )
            self._node.get_logger().info(f"Initialized InterbotixManipulatorXS for {self._robot_model}")
            return True
        except Exception as e:
            self._node.get_logger().warn(f"Failed to initialize InterbotixManipulatorXS: {e}")
            import traceback
            traceback.print_exc()
            return False

    def open_gripper(self, delay: float = 2.0) -> BridgeResult:
        """Open the gripper using trajectory action."""
        if not self._ensure_interbotix_initialized():
            return BridgeResult.fail("Interbotix robot not initialized")
        try:
            self._node.get_logger().info(f"Opening gripper (delay={delay}s)...")
            
            # Use follow_joint_trajectory action for gripper control
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration as DurationMsg
            from rclpy.action import ActionClient
            
            # Create action client for gripper controller
            gripper_client = ActionClient(
                self._node,
                FollowJointTrajectory,
                f'/{self.robot_name}/gripper_controller/follow_joint_trajectory'
            )
            
            if not gripper_client.wait_for_server(timeout_sec=2.0):
                return BridgeResult.fail("Gripper controller action server not available")
            
            # Create trajectory goal to open gripper
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.joint_names = ['left_finger']
            
            point = JointTrajectoryPoint()
            point.positions = [0.037]  # Open position (max is ~0.057, use 0.037 for safety)
            point.time_from_start = DurationMsg(sec=int(delay), nanosec=0)
            goal.trajectory.points = [point]
            
            # Send goal and wait
            goal_future = gripper_client.send_goal_async(goal)
            goal_handle = self._await_future(goal_future, timeout=3.0)
            
            if goal_handle is None or not goal_handle.accepted:
                gripper_client.destroy()
                return BridgeResult.fail("Gripper open goal rejected")
            
            # Wait for completion
            result_future = goal_handle.get_result_async()
            result = self._await_future(result_future, timeout=delay + 2.0)
            
            gripper_client.destroy()
            
            if result is None:
                return BridgeResult.fail("Gripper open timeout")
            
            self._node.get_logger().info("Gripper opened successfully")
            return BridgeResult.ok("Gripper opened")
            
        except Exception as e:
            self._node.get_logger().error(f"Failed to open gripper: {e}")
            import traceback
            traceback.print_exc()
            return BridgeResult.fail(f"Failed to open gripper: {e}")

    def close_gripper(self, delay: float = 2.0) -> BridgeResult:
        """Close the gripper using trajectory action."""
        if not self._ensure_interbotix_initialized():
            return BridgeResult.fail("Interbotix robot not initialized")
        try:
            self._node.get_logger().info(f"Closing gripper (delay={delay}s)...")
            
            # Use follow_joint_trajectory action for gripper control
            from control_msgs.action import FollowJointTrajectory
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            from builtin_interfaces.msg import Duration as DurationMsg
            from rclpy.action import ActionClient
            
            # Create action client for gripper controller
            gripper_client = ActionClient(
                self._node,
                FollowJointTrajectory,
                f'/{self.robot_name}/gripper_controller/follow_joint_trajectory'
            )
            
            if not gripper_client.wait_for_server(timeout_sec=2.0):
                return BridgeResult.fail("Gripper controller action server not available")
            
            # Create trajectory goal to close gripper
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.joint_names = ['left_finger']
            
            point = JointTrajectoryPoint()
            point.positions = [0.021]  # Close position for grasping (min is ~0.021, use 0.015 for firm grasp)
            point.time_from_start = DurationMsg(sec=int(delay), nanosec=0)
            goal.trajectory.points = [point]
            
            # Send goal and wait
            goal_future = gripper_client.send_goal_async(goal)
            goal_handle = self._await_future(goal_future, timeout=3.0)
            
            if goal_handle is None or not goal_handle.accepted:
                gripper_client.destroy()
                return BridgeResult.fail("Gripper close goal rejected")
            
            # Wait for completion
            result_future = goal_handle.get_result_async()
            result = self._await_future(result_future, timeout=delay + 2.0)
            
            gripper_client.destroy()
            
            if result is None:
                return BridgeResult.fail("Gripper close timeout")
            
            self._node.get_logger().info("Gripper closed successfully")
            return BridgeResult.ok("Gripper closed")
            
        except Exception as e:
            self._node.get_logger().error(f"Failed to close gripper: {e}")
            import traceback
            traceback.print_exc()
            return BridgeResult.fail(f"Failed to close gripper: {e}")

    def move_relative_x(self, distance_m: float) -> BridgeResult:
        """
        Move the end-effector along the world frame X-axis by the specified distance.
        Uses MoveIt's standard planning and execution (same as RViz Execute button).
        
        :param distance_m: Distance to move in meters (positive = forward in world X direction)
        :return: BridgeResult indicating success or failure
        """
        self._node.get_logger().info(f"Moving {distance_m}m along world X-axis using MoveIt planner...")
        # Use IK fallback to move along world frame X-axis
        try:
            # Get current end-effector pose in the target frame (world)
            try:
                current_transform = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    self.eef_link,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )
            except Exception as e:
                return BridgeResult.fail(f"Failed to get current EEF pose: {e}")

            self._node.get_logger().info(f"Current EEF position in {self.target_frame}: "
                                        f"x={current_transform.transform.translation.x:.3f}, "
                                        f"y={current_transform.transform.translation.y:.3f}, "
                                        f"z={current_transform.transform.translation.z:.3f}")

            # Create target pose by adding distance_m to world X-axis
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.target_frame
            target_pose.header.stamp = _now(self._node)
            target_pose.pose.position.x = current_transform.transform.translation.x + distance_m
            target_pose.pose.position.y = current_transform.transform.translation.y
            target_pose.pose.position.z = current_transform.transform.translation.z
            target_pose.pose.orientation = current_transform.transform.rotation

            self._node.get_logger().info(f"Target EEF position in {self.target_frame}: "
                                        f"x={target_pose.pose.position.x:.3f}, "
                                        f"y={target_pose.pose.position.y:.3f}, "
                                        f"z={target_pose.pose.position.z:.3f}")

            # Compute IK for the target pose
            ik_response = self._compute_ik(target_pose)
            if ik_response is None:
                return BridgeResult.fail("IK service unavailable")
            if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
                return BridgeResult.fail(f"IK failure (code {ik_response.error_code.val})")

            joint_positions = self._build_joint_vector(ik_response.solution.joint_state)
            if joint_positions is None:
                return BridgeResult.fail("IK response missing required joints")

            # Execute the motion
            self._last_joint_positions = list(joint_positions)
            self._last_pose_target = target_pose
            
            self._node.get_logger().info(f"Moving {distance_m}m along world X-axis...")
            result = self._send_move_group_goal(joint_positions, plan_only=False)
            
            if result.success:
                self._node.get_logger().info(f"Successfully moved {distance_m}m along world X-axis")
            else:
                self._node.get_logger().error(f"Failed to move: {result.message}")
            
            return result
            
        except Exception as e:
            import traceback
            traceback.print_exc()
            return BridgeResult.fail(f"Failed to move along world X-axis: {e}")

    def move_relative_camera_forward(self, distance_m: float) -> BridgeResult:
        """
        Move the end-effector forward along the camera optical Z-axis by ``distance_m``.

        The request is expressed in the camera frame but executed in the MoveIt
        target frame by projecting the camera forward direction into the target
        coordinate system before planning.
        """
        delta_target = None
        try:
            # Current end-effector pose in target frame (e.g. world)
            try:
                current_transform = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    self.eef_link,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )
            except Exception as exc:
                return BridgeResult.fail(f"Failed to get current EEF pose: {exc}")

            # Camera orientation relative to target frame
            try:
                camera_transform = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )
            except Exception as exc:
                return BridgeResult.fail(f"Failed to get camera pose: {exc}")

            q_cam = np.array(
                [
                    camera_transform.transform.rotation.x,
                    camera_transform.transform.rotation.y,
                    camera_transform.transform.rotation.z,
                    camera_transform.transform.rotation.w,
                ],
                dtype=np.float64,
            )
            q_cam = _quat_normalize(q_cam)

            # Rotate the forward offset vector into the target frame
            forward_vec = np.array([0.0, 0.0, float(distance_m)], dtype=np.float64)
            forward_quat = np.array([forward_vec[0], forward_vec[1], forward_vec[2], 0.0], dtype=np.float64)
            rotated = _quat_multiply(
                _quat_multiply(q_cam, forward_quat),
                _quat_conjugate(q_cam),
            )
            delta_target = rotated[:3]

            self._node.get_logger().info(
                f"Moving {distance_m:.3f}m along camera depth -> delta in {self.target_frame}: "
                f"dx={delta_target[0]:.4f}, dy={delta_target[1]:.4f}, dz={delta_target[2]:.4f}"
            )

            cartesian_result = self._move_linear_cartesian(delta_target, "camera depth axis")
            if cartesian_result is not None:
                if cartesian_result.success:
                    return cartesian_result
                self._node.get_logger().warn(
                    f"Cartesian camera-depth move failed: {cartesian_result.message}. Falling back to IK."
                )
            else:
                self._node.get_logger().warn("Cartesian camera-depth move unavailable; using IK fallback.")

            self._node.get_logger().warn("Falling back to IK-based camera depth motion")

            target_pose = PoseStamped()
            target_pose.header.frame_id = self.target_frame
            target_pose.header.stamp = _now(self._node)
            target_pose.pose.position.x = current_transform.transform.translation.x + float(delta_target[0])
            target_pose.pose.position.y = current_transform.transform.translation.y + float(delta_target[1])
            target_pose.pose.position.z = current_transform.transform.translation.z + float(delta_target[2])
            target_pose.pose.orientation = current_transform.transform.rotation

            ik_response = self._compute_ik(target_pose)
            if ik_response is None:
                return BridgeResult.fail("IK service unavailable")
            if ik_response.error_code.val != MoveItErrorCodes.SUCCESS:
                return BridgeResult.fail(f"IK failure (code {ik_response.error_code.val})")

            joint_positions = self._build_joint_vector(ik_response.solution.joint_state)
            if joint_positions is None:
                return BridgeResult.fail("IK response missing required joints")

            self._last_joint_positions = list(joint_positions)
            self._last_pose_target = target_pose
            result = self._send_move_group_goal(joint_positions, plan_only=False)
            if result.success:
                self._node.get_logger().info("Successfully moved along camera depth axis (IK fallback)")
            else:
                self._node.get_logger().error(f"Failed camera-depth move: {result.message}")
            return result

        except Exception as exc:
            import traceback
            traceback.print_exc()
            return BridgeResult.fail(f"Failed to move along camera depth axis: {exc}")

    # ---------------------------------------------------------------- cleanup
    def destroy(self) -> None:
        try:
            if self._active_goal_handle is not None:
                try:
                    self._active_goal_handle.cancel_goal_async()
                except Exception:
                    pass
            if self._marker_update_sub is not None:
                self._node.destroy_subscription(self._marker_update_sub)
            if self._marker_feedback_pub is not None:
                self._node.destroy_publisher(self._marker_feedback_pub)
            if self._joint_state_sub is not None:
                self._node.destroy_subscription(self._joint_state_sub)
            if self._compute_ik_client is not None:
                self._node.destroy_client(self._compute_ik_client)
            if self._cartesian_path_client is not None:
                self._node.destroy_client(self._cartesian_path_client)
            if self._arm_follow_client is not None:
                try:
                    self._arm_follow_client.destroy()
                except Exception:
                    pass
                self._arm_follow_client = None
            if self._move_group_client is not None:
                self._move_group_client.destroy()
        finally:
            try:
                if self._executor is not None:
                    self._executor.remove_node(self._node)
                    self._executor.shutdown()
            except Exception:
                pass
            try:
                self._node.destroy_node()
            except Exception:
                pass
    def _parse_vector_env(self, name: str, default: Tuple[float, float, float]) -> Tuple[float, float, float]:
        raw = os.getenv(name)
        if not raw:
            return tuple(float(v) for v in default)
        parts = [p.strip() for p in raw.replace(';', ',').split(',')]
        if len(parts) != len(default):
            return tuple(float(v) for v in default)
        try:
            return tuple(float(p) for p in parts)
        except Exception:
            return tuple(float(v) for v in default)

    def _broadcast_camera_static_tf(self):
        try:
            msg = TransformStamped()
            msg.header.stamp = _now(self._node)
            msg.header.frame_id = self.camera_parent_frame
            msg.child_frame_id = self.camera_frame
            msg.transform.translation.x = float(self.camera_offset[0])
            msg.transform.translation.y = float(self.camera_offset[1])
            msg.transform.translation.z = float(self.camera_offset[2])
            q = _quat_normalize(self.camera_quaternion)
            msg.transform.rotation.x = float(q[0])
            msg.transform.rotation.y = float(q[1])
            msg.transform.rotation.z = float(q[2])
            msg.transform.rotation.w = float(q[3])
            self._static_broadcaster.sendTransform(msg)
        except Exception as exc:
            self._node.get_logger().warn(f"Failed to broadcast camera static TF: {exc}")
