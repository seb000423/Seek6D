import math

from control_node.config import DEFAULT_CONFIG


def test_moveit_joint_mapping_matches_six_axis_robot():
    assert len(DEFAULT_CONFIG.moveit.joint_names) == 6
    assert len(DEFAULT_CONFIG.robot.home_joint) == 6


def test_moveit_uses_rg2_tcp_frame():
    assert DEFAULT_CONFIG.moveit.eef_link == "rg2_tcp"


def test_primary_pick_has_no_legacy_pregrasp_offset():
    assert math.isclose(DEFAULT_CONFIG.motion.approach_distance_mm, 0.0)


def test_cartesian_fraction_requires_nearly_complete_path():
    assert 0.99 <= DEFAULT_CONFIG.moveit.cartesian_min_fraction <= 1.0
