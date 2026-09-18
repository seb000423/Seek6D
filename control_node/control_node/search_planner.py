"""Pure helpers for DB lookup and search-policy decisions."""

from __future__ import annotations

import math
from numbers import Real
from typing import Any, Optional

import numpy as np


def make_item_query(name: str, class_label: str) -> dict[str, str]:
    """Build the query accepted by the Seek6D ``items`` table."""
    class_name = (class_label or name).strip()
    if not class_name:
        raise ValueError("DB 조회에는 class_name이 필요합니다")
    return {"class_name": class_name}


def first_item(data: dict[str, Any]) -> Optional[dict[str, Any]]:
    """Return the unique items row while ignoring malformed entries."""
    items = data.get("items", [])
    if not isinstance(items, list):
        raise ValueError("DB response.items는 배열이어야 합니다")
    return next((item for item in items if isinstance(item, dict)), None)


def db_position_mm(
    item: Optional[dict[str, Any]],
) -> Optional[tuple[float, float, float]]:
    """Extract a finite base-frame XYZ position from an items row."""
    if item is None:
        return None
    values: list[float] = []
    for key in ("x", "y", "z"):
        value = item.get(key)
        if isinstance(value, bool) or not isinstance(value, Real):
            return None
        numeric = float(value)
        if not math.isfinite(numeric):
            return None
        values.append(numeric)
    return values[0], values[1], values[2]


def db_camera_view_tcp_matrix(
    current_tcp: np.ndarray,
    db_position: tuple[float, float, float],
    tcp_to_camera: tuple[tuple[float, ...], ...],
    clearance_mm: float,
    workspace_min_xyz_mm: tuple[float, float, float],
    workspace_max_xyz_mm: tuple[float, float, float],
) -> np.ndarray:
    """Build a TCP pose that observes a DB object along camera optical +Z.

    The DB stores the object's Base-frame XYZ, so moving the TCP directly to
    that point would risk a collision.  This helper preserves the current
    camera orientation, places the object ``clearance_mm`` forward along the
    camera optical +Z axis, then converts the camera target back to a TCP target.
    With the normal downward-looking search orientation this places the camera
    above the object without assuming that camera and Base axes are parallel.
    """
    tcp = np.asarray(current_tcp, dtype=float)
    tcp_camera = np.asarray(tcp_to_camera, dtype=float)
    position = np.asarray(db_position, dtype=float)
    lower = np.asarray(workspace_min_xyz_mm, dtype=float)
    upper = np.asarray(workspace_max_xyz_mm, dtype=float)
    if tcp.shape != (4, 4) or tcp_camera.shape != (4, 4):
        raise ValueError("TCP 및 TCP-to-camera 변환은 4x4 행렬이어야 합니다")
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("DB 위치는 유한한 Base XYZ 좌표여야 합니다")
    if lower.shape != (3,) or upper.shape != (3,) or np.any(lower > upper):
        raise ValueError("DB 탐색 작업공간 경계가 올바르지 않습니다")
    if np.any(position < lower) or np.any(position > upper):
        raise ValueError(
            f"DB 위치 {tuple(position)}가 허용 작업공간을 벗어났습니다"
        )
    if not math.isfinite(clearance_mm) or clearance_mm <= 0.0:
        raise ValueError("DB 탐색 카메라 높이는 0보다 커야 합니다")

    current_camera = tcp @ tcp_camera
    target_camera = current_camera.copy()
    optical_clearance = np.array([0.0, 0.0, clearance_mm])
    target_camera[:3, 3] = (
        position - target_camera[:3, :3] @ optical_clearance
    )
    target_tcp = target_camera @ np.linalg.inv(tcp_camera)
    if not np.all(np.isfinite(target_tcp)):
        raise ValueError(
            "DB 위치에서 유효한 TCP 관측 자세를 만들지 못했습니다"
        )
    return target_tcp
