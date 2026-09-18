"""MoveIt 2 motion and unchanged OnRobot RG2 execution layer."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np
import rclpy
from dsr_msgs2.srv import GetToolForce
from rclpy.node import Node

from .config import (
    GripperConfig,
    MotionConfig,
    MoveItConfig,
    PoseConfig,
    RecenterConfig,
    RobotConfig,
    SearchConfig,
)
from .models import (
    GripperError,
    MoveItExecutionError,
    MoveItIKError,
    PoseValidationError,
    TargetPose,
)
from .moveit_client import MoveItClient
from .onrobot import RG
from .pose_utils import matrix_to_drl_posx, validate_homogeneous_matrix
from .search_planner import db_camera_view_tcp_matrix


@dataclass(frozen=True)
class GreenBoxLidRoute:
    approach: np.ndarray
    grasp: np.ndarray
    lifted: np.ndarray
    place_hover: np.ndarray
    place: np.ndarray


@dataclass(frozen=True)
class GrayBoxDrawerRoute:
    """Exact opening geometry retained for the reverse closing sequence."""

    approach: np.ndarray
    grasp_closed: np.ndarray
    grasp_opened: np.ndarray
    retreat_opened: np.ndarray
    lift_opened: np.ndarray
    observe: np.ndarray
    lift_joints_deg: tuple[float, ...]
    azimuth_offset_deg: float


class MotionExecutor:
    def __init__(
        self,
        node: Node,
        robot_config: RobotConfig,
        moveit_config: MoveItConfig,
        motion_config: MotionConfig,
        gripper_config: GripperConfig,
        search_config: SearchConfig,
        recenter_config: RecenterConfig,
        pose_config: PoseConfig,
    ) -> None:
        self.node = node
        self.robot_config = robot_config
        self.moveit_config = moveit_config
        self.motion_config = motion_config
        self.gripper_config = gripper_config
        self.search_config = search_config
        self.recenter_config = recenter_config
        self.pose_config = pose_config
        self.moveit = MoveItClient(node, moveit_config)
        self._tool_force = node.create_client(
            GetToolForce,
            motion_config.tool_force_service,
        )
        self.busy = False
        self.holding_object = False
        self.green_box_lid_route: GreenBoxLidRoute | None = None
        self.gray_box_drawer_route: GrayBoxDrawerRoute | None = None
        self.gripper = RG(
            gripper_config.name,
            gripper_config.toolchanger_ip,
            gripper_config.toolchanger_port,
        )

    def configure(self) -> None:
        """Wait until the required MoveIt actions and services are ready."""
        if not self.robot_config.enable_motion:
            return
        self.moveit.wait_until_ready()
        if not self._tool_force.wait_for_service(
            timeout_sec=self.motion_config.tool_force_service_wait_timeout_sec
        ):
            raise MoveItExecutionError(
                "TCP tool-force service unavailable: "
                f"{self.motion_config.tool_force_service}"
            )

    def _move_joint(self, joints_deg: list[float], label: str = "joint motion") -> None:
        self.moveit.move_joints_deg(joints_deg, label)

    def _move_linear_matrix(self, matrix: np.ndarray, *, vel: float, acc: float) -> None:
        del vel, acc  # Scaling is configured centrally in MoveItConfig.
        self.moveit.move_cartesian(matrix, "Cartesian motion")

    def _move_free_matrix(self, matrix: np.ndarray, *, vel: float, acc: float) -> None:
        del vel, acc
        self.moveit.move_pose(matrix, "pose motion")

    def move_home(self) -> None:
        if not self.robot_config.enable_motion:
            self.node.get_logger().info("Dry-run: home")
            return
        self._move_joint(list(self.robot_config.home_joint), "home")

    def current_tcp_posx(self) -> list[float]:
        if not self.robot_config.enable_motion:
            raise RuntimeError("Current TCP pose is unavailable in dry-run mode")
        return self.moveit.current_tcp_posx()

    def move_to_search_zone(self, zone: int) -> None:
        zones = self.search_config.search_zone_joints_deg
        if zone < 1 or zone > len(zones):
            raise ValueError(f"Unsupported search zone: {zone}")
        if not self.robot_config.enable_motion:
            self.node.get_logger().info(f"Dry-run: move to search zone {zone}")
            return
        self.busy = True
        try:
            self._move_joint(list(zones[zone - 1]), f"search zone {zone}")
        finally:
            self.busy = False

    def move_to_db_search_location(
        self,
        db_position_mm: tuple[float, float, float],
    ) -> None:
        """Move to a camera viewpoint aimed at a Base-frame DB position."""
        if not self.robot_config.enable_motion:
            self.node.get_logger().info(
                f"Dry-run: aim camera at DB location {db_position_mm}"
            )
            return
        current_tcp = self.moveit.current_tcp_matrix()
        target = db_camera_view_tcp_matrix(
            current_tcp,
            db_position_mm,
            self.pose_config.tcp_to_camera,
            self.search_config.db_search_camera_clearance_mm,
            self.search_config.db_search_workspace_min_xyz_mm,
            self.search_config.db_search_workspace_max_xyz_mm,
        )
        self.node.get_logger().info(
            f"DB search view TCP posx: {matrix_to_drl_posx(target)}"
        )
        self.busy = True
        try:
            self._move_free_matrix(
                target,
                vel=self.motion_config.approach_vel,
                acc=self.motion_config.approach_acc,
            )
        finally:
            self.busy = False

    def recenter_camera(self, target_tcp_matrix: np.ndarray) -> None:
        """Collision-check and execute one low-speed camera recenter step."""
        if not self.robot_config.enable_motion:
            raise RuntimeError("Recenter motion is unavailable in dry-run mode")
        target = validate_homogeneous_matrix(
            target_tcp_matrix,
            "camera recenter target",
        )
        self.busy = True
        try:
            self._move_linear_matrix(
                target,
                vel=self.recenter_config.velocity,
                acc=self.recenter_config.acceleration,
            )
        finally:
            self.busy = False

    def initialize(self) -> None:
        if not self.robot_config.enable_motion:
            self.node.get_logger().warning(
                "Dry-run mode: robot and gripper commands are disabled"
            )
            return
        self.move_home()
        self.gripper.open_gripper(force_val=self.gripper_config.force_tenth_newton)
        self._wait_for_gripper(require_grip=False)
        self.holding_object = False

    def _wait_for_gripper(self, *, require_grip: bool) -> None:
        deadline = time.monotonic() + self.gripper_config.timeout_sec
        while time.monotonic() < deadline:
            status = self.gripper.get_status()
            if any(status[2:7]):
                raise GripperError(f"RG2 safety status active: {status}")
            if not status[0]:
                if require_grip and not status[1]:
                    raise GripperError(
                        "RG2 motion finished but grip-detected bit is low"
                    )
                return
            time.sleep(0.1)
        raise GripperError("Timed out while waiting for RG2 motion")

    def _make_lift_matrix(self, target: np.ndarray) -> np.ndarray:
        lift = target.copy()
        lift[2, 3] += self.motion_config.lift_distance_mm
        return validate_homogeneous_matrix(lift, "lift matrix")

    def _select_lowest_joint_cost_target(self, target: TargetPose) -> TargetPose:
        if not target.grasp_candidates or not self.robot_config.enable_motion:
            return target
        current = self.moveit.current_joint_vector()
        evaluated = []
        for name, matrix in target.grasp_candidates:
            try:
                joints = self.moveit.solve_ik(matrix)
            except MoveItIKError as error:
                self.node.get_logger().warning(
                    f"IK rejected grasp candidate {name}: {error}"
                )
                continue
            delta = (joints - current + np.pi) % (2.0 * np.pi) - np.pi
            evaluated.append((float(np.linalg.norm(delta)), name, matrix))
        if not evaluated:
            raise PoseValidationError("No collision-free grasp candidate from MoveIt IK")
        cost, name, matrix = min(evaluated, key=lambda item: item[0])
        self.node.get_logger().info(
            f"Selected grasp={name}, joint_cost={np.degrees(cost):.2f} deg"
        )
        return TargetPose(
            matrix,
            matrix_to_drl_posx(matrix),
            target.source_sequence,
            grasp_candidates=target.grasp_candidates,
            object_matrix=target.object_matrix,
        )

    def _close_gripper_for_object(self, object_name: str) -> None:
        if object_name.strip().casefold() == "yellow_can":
            width = int(round(self.gripper_config.yellow_can_target_width_mm * 10.0))
            self.gripper.move_gripper(
                width_val=width,
                force_val=self.gripper_config.force_tenth_newton,
            )
        else:
            self.gripper.close_gripper(
                force_val=self.gripper_config.force_tenth_newton
            )

    def pick_and_return_home(self, target: TargetPose, *, object_name: str = "") -> bool:
        """Move directly to grasp, close RG2, lift, and return home."""
        target = self._select_lowest_joint_cost_target(target)
        lift = self._make_lift_matrix(target.matrix)
        self.node.get_logger().info(f"target posx: {target.posx}")
        self.node.get_logger().info(
            f"lift posx: {matrix_to_drl_posx(lift)}"
        )
        if not self.robot_config.enable_motion:
            self.node.get_logger().warning("Dry-run complete; no hardware motion sent")
            return False

        self.busy = True
        try:
            # No legacy 50 mm pre-grasp retreat: current TCP -> grasp directly.
            self._move_linear_matrix(
                target.matrix,
                vel=self.motion_config.grasp_vel,
                acc=self.motion_config.grasp_acc,
            )
            self._close_gripper_for_object(object_name)
            self._wait_for_gripper(require_grip=True)
            self.holding_object = True
            self._move_linear_matrix(
                lift,
                vel=self.motion_config.lift_vel,
                acc=self.motion_config.lift_acc,
            )
            self.move_home()
            self.wait_for_force_release()
            return True
        finally:
            self.busy = False

    def wait_for_force_release(self) -> bool:
        """Open RG2 after a sustained change in measured TCP force."""
        if not self.robot_config.enable_motion:
            self.holding_object = False
            return True

        settle_sec = self.motion_config.force_release_settle_sec
        if settle_sec > 0.0:
            time.sleep(settle_sec)

        sample_count = self.motion_config.force_release_baseline_samples
        if sample_count < 1:
            raise MoveItExecutionError(
                "force_release_baseline_samples must be at least 1"
            )
        baseline_samples = []
        for sample_index in range(sample_count):
            baseline_samples.append(self._read_tool_force())
            if sample_index + 1 < sample_count:
                time.sleep(self.motion_config.force_release_sample_interval_sec)
        baseline = np.median(np.stack(baseline_samples), axis=0)

        threshold = self.motion_config.force_release_threshold_n
        required_samples = self.motion_config.force_release_consecutive_samples
        if threshold <= 0.0 or required_samples < 1:
            raise MoveItExecutionError(
                "Force-release threshold and consecutive sample count must be positive"
            )
        timeout = self.motion_config.force_release_timeout_sec
        deadline = time.monotonic() + timeout if timeout > 0.0 else None
        consecutive_samples = 0
        self.node.get_logger().info(
            "Waiting for TCP force change > "
            f"{threshold:.2f} N for {required_samples} consecutive samples"
        )
        while rclpy.ok() and self.holding_object:
            current = self._read_tool_force()
            force_change = float(np.linalg.norm(current[:3] - baseline[:3]))
            if force_change >= threshold:
                consecutive_samples += 1
            else:
                consecutive_samples = 0
            if consecutive_samples >= required_samples:
                self.node.get_logger().info(
                    f"TCP force release detected: delta={force_change:.2f} N"
                )
                self.gripper.open_gripper(
                    force_val=self.gripper_config.force_tenth_newton
                )
                self._wait_for_gripper(require_grip=False)
                self.holding_object = False
                self.move_home()
                return True
            if deadline is not None and time.monotonic() >= deadline:
                raise MoveItExecutionError("Force-release monitoring timed out")
            time.sleep(self.motion_config.force_release_sample_interval_sec)
        return not self.holding_object

    def _read_tool_force(self) -> np.ndarray:
        request = GetToolForce.Request()
        request.ref = self.motion_config.tool_force_reference
        future = self._tool_force.call_async(request)
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=self.motion_config.tool_force_response_timeout_sec,
        )
        if not future.done():
            raise MoveItExecutionError(
                "Timed out waiting for TCP tool-force response"
            )
        exception = future.exception()
        if exception is not None:
            raise MoveItExecutionError(
                f"TCP tool-force service failed: {exception}"
            )
        response = future.result()
        if response is None or not response.success:
            raise MoveItExecutionError(
                "TCP tool-force service returned an unsuccessful response"
            )
        tool_force = np.asarray(response.tool_force, dtype=float)
        if tool_force.shape != (6,) or not np.all(np.isfinite(tool_force)):
            raise MoveItExecutionError(
                "TCP tool-force response must contain six finite values"
            )
        return tool_force

    def open_green_box(self, landmark: TargetPose) -> bool:
        self.green_box_lid_route = None
        landmark = self._select_lowest_joint_cost_target(landmark)
        object_matrix = landmark.object_matrix if landmark.object_matrix is not None else landmark.matrix

        def lid_pose(local_xyz, label):
            pose = landmark.matrix.copy()
            pose[:3, 3] = object_matrix[:3, 3] + object_matrix[:3, :3] @ np.asarray(local_xyz, dtype=float)
            return validate_homogeneous_matrix(pose, label)

        grasp = lid_pose(self.search_config.green_box_handle_local_xyz_mm, "green lid grasp")
        approach = grasp.copy()
        approach[2, 3] += self.search_config.green_box_approach_clearance_mm
        lifted = grasp.copy()
        lifted[2, 3] += self.search_config.green_box_lift_clearance_mm
        place_x, place_y = self.search_config.green_box_lid_place_local_xy_mm
        place = lid_pose(
            (place_x, place_y, self.search_config.green_box_lid_place_z_offset_mm),
            "green lid place",
        )
        hover = place.copy()
        hover[2, 3] += self.search_config.green_box_lift_clearance_mm
        route = GreenBoxLidRoute(approach, grasp, lifted, hover, place)
        if not self.robot_config.enable_motion:
            return False
        self.busy = True
        try:
            self._move_free_matrix(approach, vel=self.motion_config.approach_vel, acc=self.motion_config.approach_acc)
            self._move_linear_matrix(grasp, vel=self.motion_config.grasp_vel, acc=self.motion_config.grasp_acc)
            self.gripper.close_gripper(force_val=self.gripper_config.force_tenth_newton)
            self._wait_for_gripper(require_grip=True)
            self.holding_object = True
            self._move_linear_matrix(lifted, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self._move_linear_matrix(hover, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self._move_linear_matrix(place, vel=self.motion_config.grasp_vel, acc=self.motion_config.grasp_acc)
            self.gripper.open_gripper(force_val=self.gripper_config.force_tenth_newton)
            self._wait_for_gripper(require_grip=False)
            self.holding_object = False
            self._move_linear_matrix(hover, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self._move_linear_matrix(lifted, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self.green_box_lid_route = route
            return True
        finally:
            self.busy = False

    def close_green_box(self) -> bool:
        route = self.green_box_lid_route
        if route is None:
            self.node.get_logger().error("green_box close requested without saved route")
            return False
        if not self.robot_config.enable_motion:
            return False
        self.busy = True
        try:
            self._move_free_matrix(route.place_hover, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self._move_linear_matrix(route.place, vel=self.motion_config.grasp_vel, acc=self.motion_config.grasp_acc)
            self.gripper.close_gripper(force_val=self.gripper_config.force_tenth_newton)
            self._wait_for_gripper(require_grip=True)
            self.holding_object = True
            self._move_linear_matrix(route.place_hover, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self._move_linear_matrix(route.lifted, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self._move_linear_matrix(route.grasp, vel=self.motion_config.grasp_vel, acc=self.motion_config.grasp_acc)
            self.gripper.open_gripper(force_val=self.gripper_config.force_tenth_newton)
            self._wait_for_gripper(require_grip=False)
            self.holding_object = False
            self._move_linear_matrix(route.approach, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self.green_box_lid_route = None
            return True
        finally:
            self.busy = False

    def open_gray_box(self, landmark: TargetPose) -> bool:
        """Open a detected sliding drawer and finish at its camera view pose.

        This is the service-oriented equivalent of the interactive
        sliding_drawer_test sequence: A -> B -> C -> release -> D ->
        vertical lift -> observation.  The selected geometry and the actual
        lift joint state are retained so close_gray_box() can replay the safe
        reverse route after the requested object has been delivered.
        """
        self.gray_box_drawer_route = None
        object_matrix = landmark.object_matrix if landmark.object_matrix is not None else landmark.matrix
        route_geometry = self._select_gray_box_route(object_matrix)
        approach, grasp, opened, retreat, lifted, observe, azimuth = route_geometry
        if not self.robot_config.enable_motion:
            return False
        self.busy = True
        try:
            self.move_home()
            self._move_free_matrix(approach, vel=self.motion_config.approach_vel, acc=self.motion_config.approach_acc)
            self._move_linear_matrix(grasp, vel=self.motion_config.grasp_vel, acc=self.motion_config.grasp_acc)
            self.gripper.close_gripper(force_val=self.gripper_config.force_tenth_newton)
            self._wait_for_gripper(require_grip=True)
            self.holding_object = True
            self._move_linear_matrix(opened, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self.gripper.open_gripper(force_val=self.gripper_config.force_tenth_newton)
            self._wait_for_gripper(require_grip=False)
            self.holding_object = False
            self._move_linear_matrix(retreat, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self._move_linear_matrix(lifted, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            lift_joints = tuple(float(np.degrees(value)) for value in self.moveit.current_joint_vector())
            self._move_free_matrix(observe, vel=self.motion_config.approach_vel, acc=self.motion_config.approach_acc)
            self.gray_box_drawer_route = GrayBoxDrawerRoute(
                approach=approach,
                grasp_closed=grasp,
                grasp_opened=opened,
                retreat_opened=retreat,
                lift_opened=lifted,
                observe=observe,
                lift_joints_deg=lift_joints,
                azimuth_offset_deg=azimuth,
            )
            self.node.get_logger().info(
                f"gray_box opened; observation pose reached; azimuth={azimuth:+.1f}deg"
            )
            return True
        finally:
            self.busy = False

    @staticmethod
    def _gray_box_orientation(effective_yaw_rad: float, *, downward: bool) -> np.ndarray:
        """Return the RG2 orientation used by the drawer sequence."""
        from scipy.spatial.transform import Rotation

        tilt = np.pi if downward else np.pi / 2.0
        return (
            Rotation.from_rotvec([0.0, 0.0, effective_yaw_rad]).as_matrix()
            @ Rotation.from_rotvec([tilt, 0.0, 0.0]).as_matrix()
            @ Rotation.from_rotvec([0.0, 0.0, np.pi / 2.0]).as_matrix()
        )

    @staticmethod
    def _gray_box_azimuth_candidates(limit_deg: int):
        yield 0.0
        for value in range(1, max(0, int(limit_deg)) + 1):
            yield -float(value)
            yield float(value)

    def _select_gray_box_route(self, object_matrix: np.ndarray):
        """Derive A/B/C/D and choose the first all-pose IK-valid azimuth."""
        object_matrix = validate_homogeneous_matrix(object_matrix, "gray box object")
        rotation = object_matrix[:3, :3]
        origin = object_matrix[:3, 3]
        pull_direction = rotation @ np.array([1.0, 0.0, 0.0])
        pull_direction = pull_direction / np.linalg.norm(pull_direction)
        box_yaw = float(np.arctan2(-pull_direction[0], pull_direction[1]))
        handle = origin + rotation @ np.asarray(
            self.search_config.gray_box_handle_local_xyz_mm, dtype=float
        )
        pull = self.search_config.gray_box_open_distance_mm
        clearance = self.search_config.gray_box_approach_clearance_mm
        retreat_distance = self.search_config.gray_box_release_retreat_mm

        for offset_deg in self._gray_box_azimuth_candidates(
            self.search_config.gray_box_azimuth_search_limit_deg
        ):
            effective_yaw = box_yaw + np.radians(offset_deg)
            base_tool_axis = -pull_direction
            cos_a, sin_a = np.cos(np.radians(offset_deg)), np.sin(np.radians(offset_deg))
            tool_axis = np.array([
                base_tool_axis[0] * cos_a - base_tool_axis[1] * sin_a,
                base_tool_axis[0] * sin_a + base_tool_axis[1] * cos_a,
                base_tool_axis[2],
            ])
            grasp_rotation = self._gray_box_orientation(effective_yaw, downward=False)

            def pose(position, label, orientation=grasp_rotation):
                matrix = np.eye(4, dtype=float)
                matrix[:3, :3] = orientation
                matrix[:3, 3] = position
                return validate_homogeneous_matrix(matrix, label)

            grasp = pose(handle, "gray B closed grasp")
            approach = pose(handle - clearance * tool_axis, "gray A approach")
            opened_position = handle + pull * pull_direction
            opened = pose(opened_position, "gray C opened grasp")
            retreat = pose(
                opened_position - retreat_distance * tool_axis,
                "gray D release retreat",
            )
            lifted = retreat.copy()
            lifted[2, 3] += self.search_config.gray_box_observe_lift_mm
            lifted = validate_homogeneous_matrix(lifted, "gray D vertical lift")

            observe_local_x = (
                self.search_config.gray_box_shell_front_local_x_mm + pull / 2.0
            )
            observe_position = origin + rotation @ np.array([observe_local_x, 0.0, 0.0])
            observe_position[2] = self.search_config.gray_box_observe_height_mm
            observe = pose(
                observe_position,
                "gray observation",
                self._gray_box_orientation(
                    box_yaw + np.radians(self.search_config.gray_box_observe_yaw_offset_deg),
                    downward=True,
                ),
            )
            candidates = (approach, grasp, opened, retreat, lifted, observe)
            if self.robot_config.enable_motion:
                try:
                    for candidate in candidates:
                        self.moveit.solve_ik(candidate)
                except MoveItIKError:
                    continue
            self.node.get_logger().info(
                "gray_box route: "
                f"A={matrix_to_drl_posx(approach)[:3]}, "
                f"B={matrix_to_drl_posx(grasp)[:3]}, "
                f"C={matrix_to_drl_posx(opened)[:3]}, "
                f"D={matrix_to_drl_posx(retreat)[:3]}, "
                f"azimuth={offset_deg:+.1f}deg"
            )
            return approach, grasp, opened, retreat, lifted, observe, offset_deg
        raise PoseValidationError(
            "No gray_box A/B/C/D/lift/observe route has collision-free IK within +/-"
            f"{self.search_config.gray_box_azimuth_search_limit_deg}deg"
        )

    def close_gray_box(self) -> bool:
        """Replay the saved reverse route: lift -> D -> C -> B -> release -> A -> home."""
        route = self.gray_box_drawer_route
        if route is None:
            self.node.get_logger().error("gray_box close requested without saved route")
            return False
        if not self.robot_config.enable_motion:
            return False
        self.busy = True
        try:
            self.move_home()
            self._move_joint(list(route.lift_joints_deg), "gray_box saved lift joints")
            self._move_linear_matrix(route.retreat_opened, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self._move_linear_matrix(route.grasp_opened, vel=self.motion_config.grasp_vel, acc=self.motion_config.grasp_acc)
            self.gripper.close_gripper(force_val=self.gripper_config.force_tenth_newton)
            self._wait_for_gripper(require_grip=True)
            self.holding_object = True
            self._move_linear_matrix(route.grasp_closed, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self.gripper.open_gripper(force_val=self.gripper_config.force_tenth_newton)
            self._wait_for_gripper(require_grip=False)
            self.holding_object = False
            self._move_linear_matrix(route.approach, vel=self.motion_config.lift_vel, acc=self.motion_config.lift_acc)
            self.move_home()
            self.gray_box_drawer_route = None
            self.node.get_logger().info("gray_box closing sequence completed")
            return True
        finally:
            self.busy = False

    def recover_home(self) -> tuple[bool, str]:
        """Best-effort home recovery used after every task failure."""
        if not self.robot_config.enable_motion:
            return True, "dry-run"
        try:
            self.move_home()
            return True, "home recovery completed"
        except Exception as error:
            return False, f"home recovery failed: {error}"

    def shutdown(self) -> None:
        try:
            self.gripper.close_connection()
        except Exception as error:
            self.node.get_logger().warning(f"RG2 disconnect failed: {error}")
