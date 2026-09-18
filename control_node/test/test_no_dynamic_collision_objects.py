"""Regression checks for the v1.6.0 no-Planning-Scene policy."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
SOURCE_ROOT = PACKAGE_ROOT / "control_node"


def _source(filename: str) -> str:
    return (SOURCE_ROOT / filename).read_text(encoding="utf-8")


def test_collision_object_configuration_is_absent():
    source = _source("config.py")
    assert "PlanningSceneConfig" not in source
    assert "apply_planning_scene_service" not in source
    assert "touch_links" not in source


def test_moveit_client_cannot_modify_the_planning_scene():
    source = _source("moveit_client.py")
    for forbidden in (
        "ApplyPlanningScene",
        "PlanningScene",
        "CollisionObject",
        "AttachedCollisionObject",
        "add_world_object",
        "attach_object",
        "detach_object",
        "apply_scene",
    ):
        assert forbidden not in source


def test_motion_executor_has_no_dynamic_object_tracking():
    source = _source("motion_executor.py")
    for forbidden in (
        "attached_object_id",
        "_attach_grasped",
        "_detach_grasped",
        "default_object_size_m",
        "green_lid_size_m",
        "gray_drawer_size_m",
    ):
        assert forbidden not in source


def test_recover_home_does_not_compare_scene_and_gripper_state():
    source = _source("motion_executor.py")
    tree = ast.parse(source)
    recover = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "recover_home"
    )
    attributes = {
        node.attr
        for node in ast.walk(recover)
        if isinstance(node, ast.Attribute)
    }
    assert "attached_object_id" not in attributes
    assert "holding_object" not in attributes
    assert "move_home" in attributes


def test_package_versions_are_1_6_1():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    package_xml = (PACKAGE_ROOT / "package.xml").read_text(encoding="utf-8")
    assert 'version="1.6.1"' in setup_source
    assert "<version>1.6.1</version>" in package_xml
