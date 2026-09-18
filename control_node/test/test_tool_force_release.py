"""Tests for TCP force-triggered object handover."""

from types import SimpleNamespace

import numpy as np

from control_node.motion_executor import MotionExecutor


class _Logger:
    def info(self, message):
        del message


class _Node:
    def get_logger(self):
        return _Logger()


class _Gripper:
    def __init__(self):
        self.open_calls = []

    def open_gripper(self, force_val):
        self.open_calls.append(force_val)


def test_sustained_tcp_force_change_opens_gripper(monkeypatch):
    executor = object.__new__(MotionExecutor)
    executor.node = _Node()
    executor.robot_config = SimpleNamespace(enable_motion=True)
    executor.motion_config = SimpleNamespace(
        force_release_settle_sec=0.0,
        force_release_baseline_samples=2,
        force_release_sample_interval_sec=0.0,
        force_release_threshold_n=10.0,
        force_release_consecutive_samples=3,
        force_release_timeout_sec=0.0,
    )
    executor.gripper_config = SimpleNamespace(force_tenth_newton=200)
    executor.gripper = _Gripper()
    executor.holding_object = True

    readings = iter(
        (
            np.array([2.0, -3.0, 1.0, 0.0, 0.0, 0.0]),
            np.array([2.2, -2.8, 1.2, 0.0, 0.0, 0.0]),
            np.array([14.0, -3.0, 1.0, 0.0, 0.0, 0.0]),
            np.array([2.0, -3.0, 1.0, 0.0, 0.0, 0.0]),
            np.array([14.0, -3.0, 1.0, 0.0, 0.0, 0.0]),
            np.array([14.5, -3.0, 1.0, 0.0, 0.0, 0.0]),
            np.array([15.0, -3.0, 1.0, 0.0, 0.0, 0.0]),
        )
    )
    executor._read_tool_force = lambda: next(readings)
    executor._wait_for_gripper = lambda **kwargs: None
    home_calls = []
    executor.move_home = lambda: home_calls.append(True)
    monkeypatch.setattr(
        "control_node.motion_executor.rclpy.ok",
        lambda: True,
    )

    assert executor.wait_for_force_release() is True
    assert executor.gripper.open_calls == [200]
    assert executor.holding_object is False
    assert home_calls == [True]
