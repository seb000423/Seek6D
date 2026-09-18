"""Tests for object-specific RG2 commands."""

from types import SimpleNamespace

from control_node.motion_executor import MotionExecutor


class _Logger:
    def info(self, message):
        del message


class _Node:
    def get_logger(self):
        return _Logger()


class _Gripper:
    def __init__(self):
        self.move_calls = []
        self.close_calls = []

    def move_gripper(self, width_val, force_val):
        self.move_calls.append((width_val, force_val))

    def close_gripper(self, force_val):
        self.close_calls.append(force_val)


def _executor():
    executor = object.__new__(MotionExecutor)
    executor.node = _Node()
    executor.gripper = _Gripper()
    executor.gripper_config = SimpleNamespace(
        force_tenth_newton=200,
        yellow_can_target_width_mm=53.0,
    )
    return executor


def test_yellow_can_uses_53_mm_and_20_n():
    executor = _executor()

    executor._close_gripper_for_object("yellow_can")

    assert executor.gripper.move_calls == [(530, 200)]
    assert executor.gripper.close_calls == []


def test_other_objects_keep_full_close_command():
    executor = _executor()

    executor._close_gripper_for_object("white_bear")

    assert executor.gripper.move_calls == []
    assert executor.gripper.close_calls == [200]
