"""Manual input and Any6D service-response pose validation."""

from __future__ import annotations

import os
from typing import Optional, Protocol

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .config import PoseConfig
from .camera_transform import CameraToBaseTransformer
from .models import PoseValidationError, TargetPose
from .pose_utils import (
    compute_optimal_grasp_matrix,
    drl_posx_to_matrix,
    matrix_to_drl_posx,
    pose_stamped_to_matrix,
    validate_homogeneous_matrix,
)


class PoseProvider(Protocol):
    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        """Get a manual target when manual mode is enabled."""

    def target_from_pose(
        self,
        message: PoseStamped,
        sequence: int,
        base_tcp_posx: Optional[list[float]] = None,
        object_local_offset_mm: Optional[tuple[float, float, float]] = None,
    ) -> TargetPose:
        """Validate a detector-service pose and create a robot target."""

    def recenter_target(
        self,
        base_tcp_posx: list[float],
        offset_camera_m: tuple[float, float, float],
        frame_id: str,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return a Base TCP target and its Base-frame translation."""


class ManualPoseProvider:
    """Read one Doosan TCP pose from standard input for each state task."""

    def __init__(self, node: Node, config: PoseConfig) -> None:
        self.node = node
        self.config = config
        self.sequence = 0

    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        del timeout_sec
        while rclpy.ok():
            try:
                raw = input(
                    "\n목표 TCP 자세를 입력하세요 "
                    "[X Y Z A B C] (mm, degree / q: 종료): "
                ).strip()
            except EOFError:
                return None
            if raw.lower() in {"q", "quit", "exit"}:
                return None
            fields = raw.replace(",", " ").split()
            if len(fields) != 6:
                self.node.get_logger().error(
                    "정확히 6개 값을 입력해야 합니다: X Y Z A B C"
                )
                continue
            try:
                pose = [float(value) for value in fields]
                matrix = drl_posx_to_matrix(pose)
            except (ValueError, PoseValidationError) as error:
                self.node.get_logger().error(f"잘못된 목표 자세: {error}")
                continue
            if matrix[2, 3] < self.config.min_depth_mm:
                self.node.get_logger().error(
                    f"목표 Z={matrix[2, 3]:.2f}mm가 "
                    f"최소값 {self.config.min_depth_mm:.2f}mm보다 낮습니다"
                )
                continue
            self.sequence += 1
            return TargetPose(matrix, pose, self.sequence)
        return None

    def target_from_pose(
        self,
        message: PoseStamped,
        sequence: int,
        base_tcp_posx: Optional[list[float]] = None,
        object_local_offset_mm: Optional[tuple[float, float, float]] = None,
    ) -> TargetPose:
        del message, sequence, base_tcp_posx, object_local_offset_mm
        raise RuntimeError("Any6D pose is unavailable in manual input mode")

    def recenter_target(
        self,
        base_tcp_posx: list[float],
        offset_camera_m: tuple[float, float, float],
        frame_id: str,
    ) -> tuple[np.ndarray, np.ndarray]:
        del base_tcp_posx, offset_camera_m, frame_id
        raise RuntimeError("Camera recentering is unavailable in manual mode")


class Any6DPoseProvider:
    """Validate a pose returned directly by the Any6D detection service."""

    def __init__(
        self,
        node: Node,
        config: PoseConfig,
        *,
        enable_motion: bool,
    ) -> None:
        self.node = node
        self.config = config
        self._camera_to_base = CameraToBaseTransformer(
            config.tcp_to_camera,
            config.accepted_camera_frames,
            config.camera_position_scale_to_mm,
        )
        self._object_to_grasp = self._load_object_to_grasp(enable_motion)

    def _load_object_to_grasp(self, enable_motion: bool) -> np.ndarray:
        if self.config.pose_is_tcp_grasp:
            return np.eye(4, dtype=float)
        path = self.config.object_to_grasp_npy
        if not path:
            if enable_motion:
                raise RuntimeError(
                    "Robot motion blocked: object_to_grasp_npy is required "
                    "when pose_is_tcp_grasp is False"
                )
            self.node.get_logger().warning(
                "object_to_grasp_npy is empty; using identity in dry-run mode"
            )
            return np.eye(4, dtype=float)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        return validate_homogeneous_matrix(np.load(path), "T_object_grasp")

    def wait_for_target(self, timeout_sec: float) -> Optional[TargetPose]:
        del timeout_sec
        raise RuntimeError("Any6D targets must come from /find_object_pose")

    def target_from_pose(
        self,
        message: PoseStamped,
        sequence: int,
        base_tcp_posx: Optional[list[float]] = None,
        object_local_offset_mm: Optional[tuple[float, float, float]] = None,
    ) -> TargetPose:
        if base_tcp_posx is None:
            raise PoseValidationError(
                "Current base-frame TCP pose is required for camera conversion"
            )
        camera_object = pose_stamped_to_matrix(message)
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns:
            age_sec = (
                self.node.get_clock().now().nanoseconds - stamp_ns
            ) / 1e9
            if age_sec < -0.1 or age_sec > self.config.max_age_sec:
                raise PoseValidationError(
                    f"Any6D pose is stale or time-invalid: age={age_sec:.3f}s"
                )
        base_object = self._camera_to_base.transform(
            camera_object,
            base_tcp_posx,
            message.header.frame_id,
        )
        camera_xyz = camera_object[:3, 3]
        base_xyz = base_object[:3, 3]
        validation_object = base_object.copy()
        if object_local_offset_mm is not None:
            local_offset = np.asarray(object_local_offset_mm, dtype=float)
            if local_offset.shape != (3,) or not np.all(np.isfinite(local_offset)):
                raise PoseValidationError(
                    "object_local_offset_mm must contain three finite values"
                )
            validation_object[:3, 3] = (
                base_object[:3, 3] + base_object[:3, :3] @ local_offset
            )
        validation_xyz = validation_object[:3, 3]
        self.node.get_logger().info(
            "Any6D pose transform: "
            f"frame={message.header.frame_id}, "
            f"camera_xyz=({camera_xyz[0]:.6f}, {camera_xyz[1]:.6f}, "
            f"{camera_xyz[2]:.6f}), "
            f"scale_to_mm={self.config.camera_position_scale_to_mm:g}, "
            f"base_xyz_mm=({base_xyz[0]:.2f}, {base_xyz[1]:.2f}, "
            f"{base_xyz[2]:.2f})"
        )
        if object_local_offset_mm is not None:
            self.node.get_logger().info(
                "Adjusted green_box grasp position: "
                f"local_offset_mm=({local_offset[0]:.2f}, "
                f"{local_offset[1]:.2f}, {local_offset[2]:.2f}), "
                f"raw_object_z_mm={base_object[2, 3]:.2f}, "
                f"Object Z={validation_object[2, 3]:.2f}mm"
            )
        if validation_object[2, 3] < self.config.min_depth_mm:
            raise PoseValidationError(
                f"Object Z={validation_object[2, 3]:.2f}mm is below "
                f"{self.config.min_depth_mm:.2f}mm"
            )
        base_grasp, mode, grasp_candidates = compute_optimal_grasp_matrix(
            base_object,
            base_tcp_posx,
        )
        if object_local_offset_mm is not None:
            base_grasp[:3, 3] = validation_xyz
        self.node.get_logger().info(
            f"Selected {mode} grasp mode at validated grasp position "
            f"({base_grasp[0, 3]:.1f}, {base_grasp[1, 3]:.1f}, {base_grasp[2, 3]:.1f})"
        )
        if base_grasp[2, 3] < self.config.min_depth_mm:
            raise PoseValidationError(
                f"Grasp Z={base_grasp[2, 3]:.2f}mm is below "
                f"{self.config.min_depth_mm:.2f}mm"
            )
        return TargetPose(
            base_grasp,
            matrix_to_drl_posx(base_grasp),
            sequence,
            grasp_candidates=grasp_candidates,
            object_matrix=base_object,
        )

    def recenter_target(
        self,
        base_tcp_posx: list[float],
        offset_camera_m: tuple[float, float, float],
        frame_id: str,
    ) -> tuple[np.ndarray, np.ndarray]:
        return self._camera_to_base.recenter_target(
            base_tcp_posx,
            offset_camera_m,
            frame_id,
        )


def create_pose_provider(
    node: Node,
    config: PoseConfig,
    *,
    enable_motion: bool,
) -> PoseProvider:
    if config.input_mode == "manual":
        return ManualPoseProvider(node, config)
    if config.input_mode == "any6d":
        return Any6DPoseProvider(
            node,
            config,
            enable_motion=enable_motion,
        )
    raise ValueError("Pose input_mode must be 'manual' or 'any6d'")
