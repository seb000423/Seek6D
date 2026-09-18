"""Tests for object-axis-aligned RG2 grasp generation."""

import numpy as np

from control_node.pose_utils import compute_optimal_grasp_matrix


def test_grasp_aligns_tool_y_with_object_z_and_approaches_near_side():
    base_object = np.eye(4, dtype=float)
    base_object[:3, 3] = [500.0, 100.0, 80.0]
    current_tcp = [300.0, 0.0, 200.0, 0.0, 0.0, 0.0]

    grasp, mode, candidates = compute_optimal_grasp_matrix(
        base_object, current_tcp
    )

    object_z = base_object[:3, 2]
    to_object = base_object[:3, 3] - np.asarray(current_tcp[:3])
    projected = to_object - np.dot(to_object, object_z) * object_z

    assert len(candidates) == 4
    assert mode.startswith(("perpendicular_", "parallel_"))
    assert np.allclose(grasp[:3, 3], base_object[:3, 3])
    for name, candidate in candidates:
        tool_x, tool_y, tool_z = candidate[:3, :3].T
        assert np.allclose(candidate[:3, 3], base_object[:3, 3])
        assert np.allclose(np.cross(tool_x, tool_y), tool_z)
        if name.startswith("perpendicular_"):
            assert np.isclose(abs(np.dot(tool_y, object_z)), 1.0)
            assert np.isclose(np.dot(tool_z, object_z), 0.0, atol=1e-7)
            assert np.isclose(np.dot(tool_x, object_z), 0.0, atol=1e-7)
            assert np.dot(tool_z, projected) > 0.0
        else:
            assert np.isclose(abs(np.dot(tool_z, object_z)), 1.0)
            assert np.isclose(np.dot(tool_x, object_z), 0.0, atol=1e-7)


def test_grasp_has_deterministic_fallback_on_object_z_line():
    base_object = np.eye(4, dtype=float)
    base_object[:3, 3] = [0.0, 0.0, 100.0]
    current_tcp = [0.0, 0.0, 200.0, 0.0, 0.0, 0.0]

    _, _, candidates = compute_optimal_grasp_matrix(base_object, current_tcp)

    object_z = base_object[:3, 2]
    perpendicular = [
        matrix for name, matrix in candidates
        if name.startswith("perpendicular_")
    ]
    assert len(perpendicular) == 2
    for grasp in perpendicular:
        assert np.isclose(np.dot(grasp[:3, 2], object_z), 0.0, atol=1e-7)
        assert np.allclose(grasp[:3, :3].T @ grasp[:3, :3], np.eye(3))
