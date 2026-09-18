"""Regression checks for task-scoped green-box lid restoration."""

import ast
from pathlib import Path

import numpy as np

from control_node.config import SearchConfig
from control_node.models import TargetAcquisition, TargetPose


def _target() -> TargetPose:
    return TargetPose(
        matrix=np.eye(4),
        posx=[0.0] * 6,
        source_sequence=7,
    )


def test_acquisition_cleanup_flag_defaults_to_false():
    acquisition = TargetAcquisition(_target(), "search_zone_2")
    assert acquisition.close_green_box_after_delivery is False


def test_green_box_acquisition_can_request_delayed_cleanup():
    acquisition = TargetAcquisition(
        _target(),
        "green_box_zone_3",
        close_green_box_after_delivery=True,
    )
    assert acquisition.close_green_box_after_delivery is True
    assert SearchConfig().green_box_close_delay_sec == 2.0


def test_execute_task_closes_lid_before_completed_event():
    source = (
        Path(__file__).parents[1] / "control_node" / "control_node.py"
    ).read_text(encoding="utf-8")
    tree = ast.parse(source)
    execute = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "execute_task"
    )
    close_lines = [
        node.lineno
        for node in ast.walk(execute)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "close_green_box"
    ]
    completed_lines = [
        node.lineno
        for node in ast.walk(execute)
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
    assert len(close_lines) == 1
    assert len(completed_lines) == 1
    assert close_lines[0] < completed_lines[0]
