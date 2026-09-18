"""Regression checks for the integrated sliding gray-box sequence."""

import ast
from pathlib import Path

import numpy as np

from control_node.config import SearchConfig
from control_node.models import TargetAcquisition, TargetPose
from control_node.motion_executor import MotionExecutor


def _target() -> TargetPose:
    return TargetPose(np.eye(4), [0.0] * 6, 1, object_matrix=np.eye(4))


def test_drawer_geometry_defaults_are_preserved():
    config = SearchConfig()
    assert config.gray_box_handle_local_xyz_mm == (116.5, 0.0, -6.0)
    assert config.gray_box_approach_clearance_mm == 50.0
    assert config.gray_box_open_distance_mm == 155.0
    assert config.gray_box_release_retreat_mm == 40.0
    assert config.gray_box_observe_lift_mm == 150.0
    assert config.gray_box_observe_height_mm == 380.0


def test_azimuth_candidates_are_smallest_absolute_value_first():
    assert list(MotionExecutor._gray_box_azimuth_candidates(3)) == [
        0.0, -1.0, 1.0, -2.0, 2.0, -3.0, 3.0
    ]


def test_observation_tool_z_points_down():
    rotation = MotionExecutor._gray_box_orientation(0.0, downward=True)
    assert np.allclose(rotation @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0])


def test_gray_box_acquisition_requests_delayed_close():
    acquisition = TargetAcquisition(
        _target(), "gray_box_zone_2", close_gray_box_after_delivery=True
    )
    assert acquisition.close_gray_box_after_delivery is True


def test_open_and_close_methods_both_exist_and_close_precedes_completion():
    source = (
        Path(__file__).parents[1] / "control_node" / "control_node.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    execute = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "execute_task"
    )
    close_lines = [
        node.lineno for node in ast.walk(execute)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "close_gray_box"
    ]
    completed_lines = [
        node.lineno for node in ast.walk(execute)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "publish_event"
        and any(
            keyword.arg == "outcome"
            and isinstance(keyword.value, ast.Attribute)
            and keyword.value.attr == "PICK_COMPLETED"
            for keyword in node.keywords
        )
    ]
    assert close_lines and completed_lines
    assert close_lines[0] < completed_lines[0]
