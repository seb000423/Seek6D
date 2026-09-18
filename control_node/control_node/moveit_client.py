"""Low-level ROS 2 Humble clients for MoveIt planning and execution."""

from __future__ import annotations

import math
import time
from typing import Iterable, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from .config import MoveItConfig
from .models import MoveItExecutionError, MoveItIKError, MoveItPlanningError
from .pose_utils import matrix_to_drl_posx, validate_homogeneous_matrix


def _duration(seconds: float) -> Duration:
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1_000_000_000))


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    from scipy.spatial.transform import Rotation

    transform = validate_homogeneous_matrix(matrix, "MoveIt target")
    quat = Rotation.from_matrix(transform[:3, :3]).as_quat()
    pose = Pose()
    pose.position.x = float(transform[0, 3]) / 1000.0
    pose.position.y = float(transform[1, 3]) / 1000.0
    pose.position.z = float(transform[2, 3]) / 1000.0
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        float(value) for value in quat
    )
    return pose


class MoveItClient:
    """Synchronous facade over MoveIt actions/services for the task worker."""

    def __init__(
        self,
        node: Node,
        config: MoveItConfig,
    ) -> None:
        self.node = node
        self.config = config
        self._move = ActionClient(node, MoveGroup, config.move_action)
        self._execute = ActionClient(
            node, ExecuteTrajectory, config.execute_trajectory_action
        )
        self._ik = node.create_client(GetPositionIK, config.compute_ik_service)
        self._cartesian = node.create_client(
            GetCartesianPath, config.compute_cartesian_path_service
        )
        self._joint_state: Optional[JointState] = None
        self._joint_sub = node.create_subscription(
            JointState, config.joint_states_topic, self._joint_state_cb, 10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node, spin_thread=False)

    def _joint_state_cb(self, message: JointState) -> None:
        self._joint_state = message

    def wait_until_ready(self) -> None:
        timeout = self.config.action_wait_timeout_sec
        if not self._move.wait_for_server(timeout_sec=timeout):
            raise MoveItPlanningError(
                f"MoveGroup action unavailable: {self.config.move_action}"
            )
        if not self._execute.wait_for_server(timeout_sec=timeout):
            raise MoveItExecutionError(
                "ExecuteTrajectory action unavailable: "
                f"{self.config.execute_trajectory_action}"
            )
        for client, name in (
            (self._ik, self.config.compute_ik_service),
            (self._cartesian, self.config.compute_cartesian_path_service),
        ):
            if not client.wait_for_service(
                timeout_sec=self.config.service_wait_timeout_sec
            ):
                raise MoveItPlanningError(f"MoveIt service unavailable: {name}")
        self.current_joint_vector(timeout_sec=self.config.service_wait_timeout_sec)
        self.current_tcp_matrix(timeout_sec=self.config.service_wait_timeout_sec)

    def _wait_future(self, future, timeout_sec: float, label: str):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise MoveItExecutionError(f"Timed out waiting for {label}")
        exception = future.exception()
        if exception is not None:
            raise MoveItExecutionError(f"{label} failed: {exception}")
        return future.result()

    def current_joint_vector(self, timeout_sec: float = 2.0) -> np.ndarray:
        deadline = time.monotonic() + timeout_sec
        while self._joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        state = self._joint_state
        if state is None:
            raise MoveItPlanningError(
                f"No JointState received on {self.config.joint_states_topic}"
            )
        positions = dict(zip(state.name, state.position))
        try:
            result = np.array(
                [positions[name] for name in self.config.joint_names], dtype=float
            )
        except KeyError as error:
            raise MoveItPlanningError(
                f"JointState is missing configured joint {error.args[0]}"
            ) from error
        if not np.all(np.isfinite(result)):
            raise MoveItPlanningError("JointState contains non-finite positions")
        return result

    def current_joint_effort(self) -> Optional[np.ndarray]:
        state = self._joint_state
        if state is None or not state.effort or len(state.effort) != len(state.name):
            return None
        efforts = dict(zip(state.name, state.effort))
        if any(name not in efforts for name in self.config.joint_names):
            return None
        vector = np.array([efforts[name] for name in self.config.joint_names])
        return vector if np.all(np.isfinite(vector)) else None

    def current_tcp_matrix(self, timeout_sec: float = 2.0) -> np.ndarray:
        from scipy.spatial.transform import Rotation

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.config.base_frame,
                    self.config.eef_link,
                    rclpy.time.Time(),
                )
                trans = transform.transform.translation
                rot = transform.transform.rotation
                matrix = np.eye(4, dtype=float)
                matrix[:3, :3] = Rotation.from_quat(
                    [rot.x, rot.y, rot.z, rot.w]
                ).as_matrix()
                matrix[:3, 3] = [trans.x * 1000.0, trans.y * 1000.0, trans.z * 1000.0]
                return validate_homogeneous_matrix(matrix, "current TCP TF")
            except Exception:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        raise MoveItPlanningError(
            f"TF unavailable: {self.config.base_frame} -> {self.config.eef_link}"
        )

    def current_tcp_posx(self) -> list[float]:
        return matrix_to_drl_posx(self.current_tcp_matrix())

    def _joint_constraints(self, joints_rad: Iterable[float]) -> Constraints:
        values = tuple(float(value) for value in joints_rad)
        if len(values) != len(self.config.joint_names):
            raise MoveItPlanningError(
                f"Expected {len(self.config.joint_names)} joints, got {len(values)}"
            )
        constraints = Constraints()
        for name, position in zip(self.config.joint_names, values):
            item = JointConstraint()
            item.joint_name = name
            item.position = position
            item.tolerance_above = self.config.joint_tolerance_rad
            item.tolerance_below = self.config.joint_tolerance_rad
            item.weight = 1.0
            constraints.joint_constraints.append(item)
        return constraints

    def _pose_constraints(self, matrix: np.ndarray) -> Constraints:
        pose = matrix_to_pose(matrix)
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.config.base_frame
        position.link_name = self.config.eef_link
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [self.config.position_tolerance_m]
        position.constraint_region.primitives.append(region)
        position.constraint_region.primitive_poses.append(pose)
        position.weight = 1.0
        constraints.position_constraints.append(position)

        orientation = OrientationConstraint()
        orientation.header.frame_id = self.config.base_frame
        orientation.link_name = self.config.eef_link
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = self.config.orientation_tolerance_rad
        orientation.absolute_y_axis_tolerance = self.config.orientation_tolerance_rad
        orientation.absolute_z_axis_tolerance = self.config.orientation_tolerance_rad
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _execute_move_group(self, constraints: Constraints, label: str) -> None:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.config.planning_group
        request.pipeline_id = self.config.planning_pipeline_id
        request.planner_id = self.config.planner_id
        request.num_planning_attempts = self.config.planning_attempts
        request.allowed_planning_time = self.config.planning_time_sec
        request.max_velocity_scaling_factor = self.config.velocity_scaling
        request.max_acceleration_scaling_factor = self.config.acceleration_scaling
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = self.config.allow_replanning
        goal.planning_options.replan_attempts = self.config.planning_attempts

        sent = self._wait_future(
            self._move.send_goal_async(goal),
            self.config.action_wait_timeout_sec,
            f"{label} goal acceptance",
        )
        if sent is None or not sent.accepted:
            raise MoveItPlanningError(f"MoveGroup rejected {label}")
        wrapped = self._wait_future(
            sent.get_result_async(), self.config.execution_timeout_sec, label
        )
        code = int(wrapped.result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            raise MoveItPlanningError(f"MoveGroup {label} failed, error_code={code}")

    def move_joints_deg(self, joints_deg: Iterable[float], label: str) -> None:
        radians = [math.radians(float(value)) for value in joints_deg]
        self._execute_move_group(self._joint_constraints(radians), label)

    def move_pose(self, matrix: np.ndarray, label: str) -> None:
        self._execute_move_group(self._pose_constraints(matrix), label)

    def solve_ik(self, matrix: np.ndarray) -> np.ndarray:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.config.planning_group
        ik.ik_link_name = self.config.eef_link
        ik.pose_stamped.header.frame_id = self.config.base_frame
        ik.pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        ik.pose_stamped.pose = matrix_to_pose(matrix)
        ik.avoid_collisions = True
        ik.timeout = _duration(self.config.ik_timeout_sec)
        ik.robot_state.is_diff = True
        response = self._wait_future(
            self._ik.call_async(request),
            self.config.ik_timeout_sec + 1.0,
            "compute_ik",
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MoveItIKError(
                f"No collision-free IK, error_code={response.error_code.val}"
            )
        mapping = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        try:
            solution = np.array(
                [mapping[name] for name in self.config.joint_names], dtype=float
            )
        except KeyError as error:
            raise MoveItIKError(
                f"IK response missing joint {error.args[0]}"
            ) from error
        if not np.all(np.isfinite(solution)):
            raise MoveItIKError("IK returned non-finite joint positions")
        return solution

    def move_cartesian(self, matrix: np.ndarray, label: str) -> None:
        request = GetCartesianPath.Request()
        request.header.frame_id = self.config.base_frame
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = self.config.planning_group
        request.link_name = self.config.eef_link
        request.waypoints = [matrix_to_pose(matrix)]
        request.max_step = self.config.cartesian_max_step_m
        request.jump_threshold = self.config.cartesian_jump_threshold
        request.avoid_collisions = True
        response = self._wait_future(
            self._cartesian.call_async(request),
            self.config.planning_time_sec + 2.0,
            f"compute_cartesian_path({label})",
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MoveItPlanningError(
                f"Cartesian plan {label} failed, error_code={response.error_code.val}"
            )
        if response.fraction < self.config.cartesian_min_fraction:
            raise MoveItPlanningError(
                f"Cartesian plan {label} incomplete: fraction={response.fraction:.3f}"
            )
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        sent = self._wait_future(
            self._execute.send_goal_async(goal),
            self.config.action_wait_timeout_sec,
            f"{label} trajectory acceptance",
        )
        if sent is None or not sent.accepted:
            raise MoveItExecutionError(f"ExecuteTrajectory rejected {label}")
        wrapped = self._wait_future(
            sent.get_result_async(), self.config.execution_timeout_sec, label
        )
        if wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MoveItExecutionError(
                f"Trajectory {label} failed, error_code="
                f"{wrapped.result.error_code.val}"
            )
