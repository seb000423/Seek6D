"""Tests for the structured clipped-object detector response."""

import json

from control_node.config import RecenterConfig
from control_node.detection_client import DetectionClient
from control_node.models import RobotTask


class _Logger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class _Node:
    def __init__(self):
        self.logger = _Logger()

    def get_logger(self):
        return self.logger


def _client():
    client = object.__new__(DetectionClient)
    client.node = _Node()
    client.recenter_config = RecenterConfig()
    client._sequence = 0
    return client


def _recenter_payload(request_id):
    return {
        "request_id": request_id,
        "result": "recenter_required",
        "detected": False,
        "detected_name": "yellow_can",
        "detected_class_label": "yellow_can",
        "pose": None,
        "reason": "object_clipped",
        "recenter": {
            "frame_id": "camera_color_optical_frame",
            "offset_camera_m": {"x": 0.042, "y": -0.018, "z": 0.0},
            "pixel_error": {"u": 85.0, "v": -36.0},
            "depth_m": 0.48,
            "edge_sides": ["right", "top"],
        },
    }


def test_attempt_number_is_part_of_each_request_id():
    task = RobotTask("task-001", "yellow_can", "yellow_can", "tester")

    first = DetectionClient._build_request_id(task, 3, "target", (), 0)
    second = DetectionClient._build_request_id(task, 3, "target", (), 1)

    assert first == "task-001:zone-3:attempt-0"
    assert second == "task-001:zone-3:attempt-1"


def test_recenter_response_is_parsed():
    client = _client()
    request_id = "task-001:zone-3:attempt-0"

    result = client._parse_response(
        json.dumps(_recenter_payload(request_id)), request_id
    )

    assert result.result == "recenter_required"
    assert result.detected is False
    assert result.request_id == request_id
    assert result.recenter is not None
    assert result.recenter.offset_camera_m == (0.042, -0.018, 0.0)
    assert result.recenter.edge_sides == ("right", "top")


def test_recenter_response_with_wrong_request_id_is_rejected():
    client = _client()
    payload = _recenter_payload("old-request")

    result = client._parse_response(
        json.dumps(payload), "task-001:zone-3:attempt-1"
    )

    assert result.result == "not_detected"
    assert result.reason == "invalid_response"
    assert result.recenter is None
    assert client.node.logger.errors
