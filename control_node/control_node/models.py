"""Shared data models and domain errors."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Optional

import numpy as np
from geometry_msgs.msg import PoseStamped


class TaskOutcome(str, Enum):
    QUEUED = "queued"
    DB_LOOKUP = "db_lookup"
    WAITING_POSE = "waiting_pose"
    SEARCHING = "searching"
    ZONE_NOT_FOUND = "zone_not_found"
    LANDMARK_SEARCHING = "landmark_searching"
    LANDMARK_FOUND = "landmark_found"
    LANDMARK_NOT_FOUND = "landmark_not_found"
    GREEN_BOX_CLEANUP = "green_box_cleanup"
    GRAY_BOX_CLEANUP = "gray_box_cleanup"
    PICK_COMPLETED = "pick_completed"
    NOT_FOUND = "not_found"
    DRY_RUN = "dry_run"
    REJECTED = "rejected"
    FAILED = "failed"


class PoseValidationError(RuntimeError):
    """A received or calculated robot pose is invalid or unsafe."""


class GripperError(RuntimeError):
    """The gripper timed out or reported a safety condition."""


class DBLookupError(RuntimeError):
    """The DB service is missing or returned an invalid response."""


class MoveItPlanningError(RuntimeError):
    """MoveIt could not produce a collision-free motion plan."""


class MoveItIKError(MoveItPlanningError):
    """MoveIt could not find a valid collision-free IK solution."""


class MoveItExecutionError(RuntimeError):
    """A planned MoveIt trajectory could not be executed safely."""


@dataclass(frozen=True)
class TargetPose:
    matrix: np.ndarray
    posx: list[float]
    source_sequence: int
    grasp_candidates: tuple[tuple[str, np.ndarray], ...] = ()
    # Raw object pose before it is converted to a robot grasp orientation.
    # Landmark workflows use this frame for object-local offsets.
    object_matrix: Optional[np.ndarray] = None


@dataclass(frozen=True)
class TargetAcquisition:
    """A target pose plus the task-level cleanup required after delivery."""

    target: TargetPose
    source: str
    close_green_box_after_delivery: bool = False
    close_gray_box_after_delivery: bool = False


@dataclass(frozen=True)
class RecenterHint:
    """Camera-optical-frame translation requested by the detector."""

    frame_id: str
    offset_camera_m: tuple[float, float, float]
    pixel_error: tuple[float, float]
    depth_m: float
    edge_sides: tuple[str, ...]


@dataclass(frozen=True)
class DetectionResult:
    detected: bool
    pose: Optional[PoseStamped]
    result: str = "not_detected"
    request_id: str = ""
    reason: str = ""
    recenter: Optional[RecenterHint] = None
    detected_name: str = ""
    detected_class_label: str = ""


@dataclass(frozen=True)
class RobotTask:
    task_id: str
    name: str
    class_label: str
    requested_by: str
    command: str = "pick"


@dataclass(frozen=True)
class DBLookupResult:
    location_known: bool
    query: dict[str, str]
    item: Optional[dict[str, Any]]

    def to_payload(self) -> dict[str, Any]:
        return {
            "location_known": self.location_known,
            "query": self.query,
            "item": self.item,
        }
