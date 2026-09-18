import numpy as np

from control_node.camera_transform import CameraToBaseTransformer
from control_node.pose_utils import drl_posx_to_matrix


def test_camera_pose_is_transformed_through_current_tcp():
    tcp_camera = (
        (1.0, 0.0, 0.0, 10.0),
        (0.0, 1.0, 0.0, 20.0),
        (0.0, 0.0, 1.0, 30.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    transformer = CameraToBaseTransformer(tcp_camera, ("camera",), 1000.0)
    camera_object = np.eye(4)
    camera_object[:3, 3] = [0.001, 0.002, 0.003]
    base_tcp_posx = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]

    actual = transformer.transform(
        camera_object,
        base_tcp_posx,
        "camera",
    )
    camera_object_mm = camera_object.copy()
    camera_object_mm[:3, 3] *= 1000.0
    expected = (
        drl_posx_to_matrix(base_tcp_posx)
        @ np.asarray(tcp_camera)
        @ camera_object_mm
    )

    assert np.allclose(actual, expected)


def test_recenter_offset_is_rotated_without_changing_tcp_attitude():
    tcp_camera = (
        (0.0, -1.0, 0.0, 10.0),
        (1.0, 0.0, 0.0, 20.0),
        (0.0, 0.0, 1.0, 30.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    transformer = CameraToBaseTransformer(
        tcp_camera,
        ("camera_color_optical_frame",),
        1000.0,
    )
    base_tcp_posx = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]

    target, delta_base_mm = transformer.recenter_target(
        base_tcp_posx,
        (0.010, 0.0, 0.0),
        "camera_color_optical_frame",
    )

    original = drl_posx_to_matrix(base_tcp_posx)
    assert np.allclose(delta_base_mm, [0.0, 10.0, 0.0])
    assert np.allclose(target[:3, 3], [100.0, 210.0, 300.0])
    assert np.allclose(target[:3, :3], original[:3, :3])
