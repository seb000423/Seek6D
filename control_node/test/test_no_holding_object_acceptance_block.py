"""Regression checks for the v1.6.1 task-acceptance policy."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
SOURCE_ROOT = PACKAGE_ROOT / "control_node"


def _source(filename: str) -> str:
    return (SOURCE_ROOT / filename).read_text(encoding="utf-8")


def _method(filename: str, class_name: str, method_name: str) -> ast.FunctionDef:
    tree = ast.parse(_source(filename))
    class_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    return next(
        node
        for node in class_node.body
        if isinstance(node, ast.FunctionDef) and node.name == method_name
    )


def _attributes(node: ast.AST) -> set[str]:
    return {
        child.attr
        for child in ast.walk(node)
        if isinstance(child, ast.Attribute)
    }


def test_acceptance_guard_does_not_block_holding_object():
    guard = _method("control_node.py", "RobotControlNode", "_acceptance_guard")
    attributes = _attributes(guard)
    assert "holding_object" not in attributes
    assert "green_box_lid_route" in attributes
    assert "gray_box_drawer_route" in attributes


def test_execute_task_does_not_block_holding_object():
    execute_task = _method("control_node.py", "RobotControlNode", "execute_task")
    assert "holding_object" not in _attributes(execute_task)


def test_blocked_holding_object_outcome_is_removed():
    source = _source("models.py")
    assert "BLOCKED_HOLDING_OBJECT" not in source
    assert "blocked_holding_object" not in source


def test_holding_object_state_tracking_is_retained():
    status = _method("control_node.py", "RobotControlNode", "_status_payload")
    assert "holding_object" in _attributes(status)
    assert "holding_object" in _source("motion_executor.py")
