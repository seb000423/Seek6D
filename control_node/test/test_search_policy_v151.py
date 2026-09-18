"""Regression tests for DB-priority search and object-local offset policy."""

import numpy as np

from control_node.config import SearchConfig
from control_node.search_planner import (
    db_camera_view_tcp_matrix,
    db_position_mm,
    first_item,
    make_item_query,
)


def test_green_box_offset_is_local_z_only():
    assert SearchConfig().green_box_handle_local_xyz_mm == (0.0, 0.0, 40.0)


def test_green_box_offset_adjusts_minus_34_2_to_plus_5_8_when_axes_align():
    base_object = np.eye(4)
    base_object[2, 3] = -34.2
    local_offset = np.asarray(
        SearchConfig().green_box_handle_local_xyz_mm,
        dtype=float,
    )
    adjusted_xyz = (
        base_object[:3, 3] + base_object[:3, :3] @ local_offset
    )
    assert np.isclose(adjusted_xyz[2], 5.8)


def test_yellow_can_db_query_uses_items_class_name():
    assert make_item_query("yellow_can", "yellow_can") == {
        "class_name": "yellow_can"
    }


def test_db_xyz_is_recognized_without_legacy_location_field():
    row = {
        "class_name": "yellow_can",
        "x": 412.5,
        "y": -135.8,
        "z": 125.3,
    }
    assert first_item({"count": 1, "items": [row]}) is row
    assert db_position_mm(row) == (412.5, -135.8, 125.3)


def test_db_search_places_object_on_optical_axis_and_preserves_orientation():
    current_tcp = np.eye(4)
    tcp_to_camera = np.eye(4)
    tcp_to_camera[:3, 3] = [34.0, 78.0, -182.5]
    target_tcp = db_camera_view_tcp_matrix(
        current_tcp,
        (400.0, -120.0, 125.0),
        tuple(tuple(row) for row in tcp_to_camera),
        300.0,
        (100.0, -700.0, -50.0),
        (850.0, 700.0, 600.0),
    )
    target_camera = target_tcp @ tcp_to_camera
    camera_object = np.linalg.inv(target_camera) @ np.array(
        [400.0, -120.0, 125.0, 1.0]
    )
    assert np.allclose(camera_object[:3], [0.0, 0.0, 300.0])
    assert np.allclose(target_camera[:3, :3], np.eye(3))


def test_db_position_outside_workspace_is_rejected():
    with np.testing.assert_raises(ValueError):
        db_camera_view_tcp_matrix(
            np.eye(4),
            (5000.0, 0.0, 0.0),
            tuple(tuple(row) for row in np.eye(4)),
            300.0,
            (100.0, -700.0, -50.0),
            (850.0, 700.0, 600.0),
        )
