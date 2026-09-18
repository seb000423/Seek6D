"""Pure pose-validation and conversion helpers."""

from __future__ import annotations

import warnings

import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from .models import PoseValidationError


def validate_homogeneous_matrix(matrix: np.ndarray, name: str) -> np.ndarray:
    """Validate a 4x4 rigid transform and return a defensive float copy."""
    transform = np.asarray(matrix, dtype=float)
    if transform.shape != (4, 4):
        raise PoseValidationError(
            f"{name} must have shape (4, 4), got {transform.shape}"
        )
    if not np.all(np.isfinite(transform)):
        raise PoseValidationError(f"{name} contains NaN or infinity")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
        raise PoseValidationError(f"{name} has an invalid homogeneous last row")

    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        raise PoseValidationError(f"{name} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3):
        raise PoseValidationError(f"{name} rotation determinant is not +1")
    return transform.copy()


def matrix_to_drl_posx(matrix: np.ndarray) -> list[float]:
    """Convert a base-frame matrix to Doosan [X, Y, Z, A, B, C]."""
    transform = validate_homogeneous_matrix(matrix, "target matrix")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        abc = Rotation.from_matrix(transform[:3, :3]).as_euler(
            "ZYZ",
            degrees=True,
        )
    return [
        float(transform[0, 3]),
        float(transform[1, 3]),
        float(transform[2, 3]),
        float(abc[0]),
        float(abc[1]),
        float(abc[2]),
    ]


def drl_posx_to_matrix(pose: list[float]) -> np.ndarray:
    """Convert Doosan [X, Y, Z, A, B, C] to a homogeneous matrix."""
    if len(pose) != 6 or not np.all(np.isfinite(pose)):
        raise PoseValidationError("Doosan pose must contain six finite values")
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_euler(
        "ZYZ",
        pose[3:6],
        degrees=True,
    ).as_matrix()
    transform[:3, 3] = pose[:3]
    return transform


def pose_stamped_to_matrix(message: PoseStamped) -> np.ndarray:
    """Reconstruct the frame-to-object transform from PoseStamped."""
    position = message.pose.position
    orientation = message.pose.orientation
    quaternion = np.array(
        [orientation.x, orientation.y, orientation.z, orientation.w],
        dtype=float,
    )
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-8:
        raise PoseValidationError("Any6D quaternion is invalid")
    quaternion /= norm

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    transform[:3, 3] = [position.x, position.y, position.z]
    return validate_homogeneous_matrix(transform, "T_frame_object")


def rotation_distance_deg(first: np.ndarray, second: np.ndarray) -> float:
    """Return the shortest angular distance between rotation matrices."""
    relative = Rotation.from_matrix(first).inv() * Rotation.from_matrix(second)
    return float(np.degrees(relative.magnitude()))


def compute_optimal_grasp_matrix(
    base_object: np.ndarray,
    base_tcp_posx: list[float],
) -> tuple[np.ndarray, str, tuple[tuple[str, np.ndarray], ...]]:
    """Align the RG2 opening axis across the object and approach its near side.

    Tool X is the RG2 opening/closing axis, Tool Y follows Object Z, and
    Tool Z is the insertion axis.  Projecting the TCP-to-object vector onto
    the plane normal to Object Z makes Tool Z perpendicular to Object Z.
    """
    base_obj = validate_homogeneous_matrix(base_object, "base_object")
    pos_obj = base_obj[:3, 3]
    object_z = base_obj[:3, 2].copy()
    object_z /= np.linalg.norm(object_z)

    t_curr = drl_posx_to_matrix(base_tcp_posx)
    pos_curr = t_curr[:3, 3]
    r_curr = t_curr[:3, :3]

    to_object = pos_obj - pos_curr
    tool_z = to_object - np.dot(to_object, object_z) * object_z
    if np.linalg.norm(tool_z) < 1e-4:
        # When the TCP lies almost on the Object-Z line, preserve as much of
        # the current Tool-Z direction as possible.  Object X is the final,
        # deterministic fallback and is orthogonal to Object Z by definition.
        current_tool_z = r_curr[:, 2]
        tool_z = (
            current_tool_z
            - np.dot(current_tool_z, object_z) * object_z
        )
        if np.linalg.norm(tool_z) < 1e-4:
            tool_z = base_obj[:3, 0].copy()
    tool_z /= np.linalg.norm(tool_z)

    candidates: list[tuple[str, np.ndarray]] = []
    for sign, name in ((1.0, "object_z_positive"), (-1.0, "object_z_negative")):
        tool_y = sign * object_z
        tool_x = np.cross(tool_y, tool_z)
        tool_x /= np.linalg.norm(tool_x)
        # Recompute Y to remove accumulated floating-point error while
        # preserving a right-handed Tool-X/Y/Z frame.
        tool_y = np.cross(tool_z, tool_x)
        tool_y /= np.linalg.norm(tool_y)
        candidate = np.column_stack((tool_x, tool_y, tool_z))
        candidates.append((f"perpendicular_{name}", candidate))

    # Axial grasp: Tool Z follows the near-facing sign of Object Z, while
    # Tool X (the RG2 opening axis) follows either sign of Object X.
    object_x = base_obj[:3, 0].copy()
    object_x -= np.dot(object_x, object_z) * object_z
    object_x /= np.linalg.norm(object_x)
    axial_sign = 1.0 if np.dot(to_object, object_z) >= 0.0 else -1.0
    axial_tool_z = axial_sign * object_z
    for sign, name in ((1.0, "object_x_positive"), (-1.0, "object_x_negative")):
        tool_x = sign * object_x
        tool_y = np.cross(axial_tool_z, tool_x)
        tool_y /= np.linalg.norm(tool_y)
        tool_x = np.cross(tool_y, axial_tool_z)
        tool_x /= np.linalg.norm(tool_x)
        candidate = np.column_stack((tool_x, tool_y, axial_tool_z))
        candidates.append((f"parallel_{name}", candidate))

    mode, selected_r = min(
        candidates,
        key=lambda item: rotation_distance_deg(r_curr, item[1]),
    )

    grasp_matrix = np.eye(4, dtype=float)
    grasp_matrix[:3, :3] = selected_r
    grasp_matrix[:3, 3] = pos_obj
    grasp_candidates = []
    for name, rotation in candidates:
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = pos_obj
        grasp_candidates.append(
            (name, validate_homogeneous_matrix(matrix, name))
        )
    return (
        validate_homogeneous_matrix(grasp_matrix, "T_base_grasp"),
        mode,
        tuple(grasp_candidates),
    )
