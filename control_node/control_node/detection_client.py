"""ROS 2 service client for the Any6D detector node."""

from __future__ import annotations

import json
import math
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from interfaces.srv import DetectObject, UpdateTcpPose
from rclpy.node import Node

from .config import RecenterConfig, SearchConfig
from .models import DetectionResult, RecenterHint, RobotTask


class DetectionClient:
    """Request target or landmark detection through one ROS 2 service."""

    def __init__(
        self,
        node: Node,
        config: SearchConfig,
        recenter_config: RecenterConfig,
    ) -> None:
        self.node = node
        self.config = config
        self.recenter_config = recenter_config
        self.client = node.create_client(DetectObject, config.detection_service)
        self.tcp_pose_client = node.create_client(
            UpdateTcpPose,
            config.tcp_pose_service,
        )
        self._sequence = 0

    @property
    def sequence(self) -> int:
        return self._sequence

    def request_detection(
        self,
        task: RobotTask,
        zone: int,
        timeout_sec: float,
        *,
        request_kind: str = "target",
        candidates: tuple[tuple[str, str], ...] = (),
        base_tcp_posx: Optional[list[float]] = None,
        attempt: int = 0,
    ) -> DetectionResult:
        request_id = self._build_request_id(
            task, zone, request_kind, candidates, attempt
        )
        if base_tcp_posx is None or len(base_tcp_posx) != 6:
            self.node.get_logger().error(
                "A six-element current TCP pose is required before detection"
            )
            return DetectionResult(
                False, None, request_id=request_id,
                reason="current_tcp_unavailable",
            )

        # all_object용 TCP pose 전달은 선택 사항(best-effort)입니다.
        # /update_robot_tcp_pose 서비스가 떠 있으면 현재 TCP를 전달하고,
        # 서비스가 없거나 실패하더라도 Any6D /find_object_pose 호출은 계속합니다.
        self._send_tcp_pose(request_id, base_tcp_posx)

        if not self.client.wait_for_service(
            timeout_sec=self.config.detection_service_wait_timeout_sec
        ):
            self.node.get_logger().warning(
                f"Any6D service unavailable: {self.config.detection_service}"
            )
            return DetectionResult(
                False, None, request_id=request_id,
                reason="service_unavailable",
            )

        object_name = task.class_label or task.name
        payload: dict[str, Any] = {
            "request_id": request_id,
            "request_type": request_kind,
            "task_id": task.task_id,
            "search_zone": zone,
            "object_name": object_name,
            "name": object_name,
            "class_label": object_name,
        }
        if candidates:
            payload["candidate_targets"] = [
                {
                    "object_name": class_label,
                    "name": class_label,
                    "class_label": class_label,
                }
                for name, class_label in candidates
            ]

        request = DetectObject.Request()
        request.request = json.dumps(payload, ensure_ascii=False)
        future = self.client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.1, max(0.0, remaining)),
            )

        if not future.done():
            future.cancel()
            self.node.get_logger().warning(
                f"Any6D service timeout: request_id={request_id}"
            )
            return DetectionResult(
                False, None, request_id=request_id, reason="service_timeout"
            )
        if future.exception() is not None:
            self.node.get_logger().error(
                f"Any6D service failed: {future.exception()}"
            )
            return DetectionResult(
                False, None, request_id=request_id, reason="service_exception"
            )

        response = future.result()
        if response is None or not response.success:
            message = response.message if response else "empty response"
            self.node.get_logger().warning(f"Any6D detection failed: {message}")
            return DetectionResult(
                False, None, request_id=request_id, reason="service_rejected"
            )
        return self._parse_response(response.response, request_id)

    @staticmethod
    def _build_request_id(
        task: RobotTask,
        zone: int,
        request_kind: str,
        candidates: tuple[tuple[str, str], ...],
        attempt: int,
    ) -> str:
        if attempt < 0:
            raise ValueError("attempt must be zero or positive")
        request_id = f"{task.task_id}:zone-{zone}"
        if request_kind != "target":
            labels = "-".join(
                class_label.strip().casefold()
                for _, class_label in candidates
                if class_label.strip()
            )
            suffix = request_kind if not labels else f"{request_kind}-{labels}"
            request_id += f":{suffix}"
        return f"{request_id}:attempt-{attempt}"

    def _send_tcp_pose(
        self,
        request_id: str,
        base_tcp_posx: list[float],
    ) -> bool:
        if not self.tcp_pose_client.wait_for_service(
            timeout_sec=self.config.tcp_pose_service_wait_timeout_sec
        ):
            self.node.get_logger().warning(
                f"TCP pose service unavailable: {self.config.tcp_pose_service}; "
                "skipping all_object TCP update and continuing Any6D detection"
            )
            return False

        request = UpdateTcpPose.Request()
        request.tcp_pose = [float(value) for value in base_tcp_posx]
        future = self.tcp_pose_client.call_async(request)
        deadline = time.monotonic() + self.config.tcp_pose_response_timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.1, max(0.0, remaining)),
            )

        if not future.done():
            future.cancel()
            self.node.get_logger().warning(
                f"TCP pose service timeout: request_id={request_id}"
            )
            return False
        if future.exception() is not None:
            self.node.get_logger().error(
                f"TCP pose service failed: {future.exception()}"
            )
            return False
        response = future.result()
        if response is None or not response.success:
            message = response.message if response else "empty response"
            self.node.get_logger().warning(
                f"Detector rejected current TCP pose: {message}"
            )
            return False
        self.node.get_logger().info(
            f"Sent current TCP pose to detector: pose={base_tcp_posx}"
        )
        return True

    def _parse_response(self, raw: str, request_id: str) -> DetectionResult:
        try:
            payload = json.loads(raw) if raw else {}
            result = str(payload["result"]).strip()
            if result not in {
                "detected", "recenter_required", "not_detected"
            }:
                raise ValueError(f"unsupported result: {result}")
            detected = payload["detected"]
            if not isinstance(detected, bool):
                raise ValueError("detected must be boolean")
            response_id = str(payload["request_id"]).strip()
            if response_id != request_id:
                raise ValueError(
                    f"request_id mismatch: {response_id} != {request_id}"
                )
            pose = self._parse_pose(payload.get("pose"))
            recenter_data = payload.get("recenter")
            recenter = (
                self._parse_recenter(recenter_data)
                if recenter_data is not None else None
            )
            if result == "detected":
                if not detected or pose is None or recenter is not None:
                    raise ValueError(
                        "detected result requires detected=true, pose, and no recenter"
                    )
            elif result == "recenter_required":
                if detected or pose is not None or recenter is None:
                    raise ValueError(
                        "recenter_required requires detected=false, pose=null, "
                        "and recenter"
                    )
            elif detected or pose is not None or recenter is not None:
                raise ValueError(
                    "not_detected requires detected=false, pose=null, and no recenter"
                )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            self.node.get_logger().error(
                f"Invalid Any6D service response: {error}"
            )
            return DetectionResult(
                False, None, request_id=request_id, reason="invalid_response"
            )

        self._sequence += 1
        return DetectionResult(
            detected=detected,
            pose=pose,
            result=result,
            request_id=response_id,
            reason=str(payload.get("reason", "")),
            recenter=recenter,
            detected_name=str(payload.get("detected_name", "")),
            detected_class_label=str(
                payload.get("detected_class_label", "")
            ),
        )

    def _parse_recenter(self, data: Any) -> RecenterHint:
        if not isinstance(data, dict):
            raise ValueError("recenter must be an object")
        frame_id = str(data.get("frame_id", "")).strip()
        expected = self.recenter_config.expected_frame_id
        if frame_id != expected:
            raise ValueError(
                f"recenter.frame_id must be {expected}, got {frame_id or '<empty>'}"
            )
        offset = data.get("offset_camera_m")
        pixel_error = data.get("pixel_error")
        edge_sides = data.get("edge_sides")
        if not isinstance(offset, dict):
            raise ValueError("recenter.offset_camera_m must be an object")
        if not isinstance(pixel_error, dict):
            raise ValueError("recenter.pixel_error must be an object")
        if not isinstance(edge_sides, list):
            raise ValueError("recenter.edge_sides must be an array")

        offset_values = tuple(float(offset[key]) for key in ("x", "y", "z"))
        pixel_values = tuple(float(pixel_error[key]) for key in ("u", "v"))
        depth_m = float(data["depth_m"])
        if not all(math.isfinite(value) for value in (
            offset_values + pixel_values + (depth_m,)
        )):
            raise ValueError("recenter values must be finite")
        if depth_m <= 0.0:
            raise ValueError("recenter.depth_m must be positive")
        valid_sides = {"left", "right", "top", "bottom"}
        sides = tuple(str(side).strip().casefold() for side in edge_sides)
        if not sides or any(side not in valid_sides for side in sides):
            raise ValueError("recenter.edge_sides contains an invalid edge")
        return RecenterHint(
            frame_id=frame_id,
            offset_camera_m=offset_values,
            pixel_error=pixel_values,
            depth_m=depth_m,
            edge_sides=sides,
        )

    def _parse_pose(self, data: Any) -> Optional[PoseStamped]:
        if data is None:
            return None
        if not isinstance(data, dict):
            raise ValueError("pose must be an object")
        position = data.get("position", {})
        orientation = data.get("orientation", {})
        pose = PoseStamped()
        pose.header.frame_id = str(data.get("frame_id", "")).strip()
        if not pose.header.frame_id:
            raise ValueError("pose.frame_id is required")
        stamp = data.get("stamp", {})
        pose.header.stamp.sec = int(stamp.get("sec", 0))
        pose.header.stamp.nanosec = int(stamp.get("nanosec", 0))
        pose.pose.position.x = float(position["x"])
        pose.pose.position.y = float(position["y"])
        pose.pose.position.z = float(position["z"])
        pose.pose.orientation.x = float(orientation["x"])
        pose.pose.orientation.y = float(orientation["y"])
        pose.pose.orientation.z = float(orientation["z"])
        pose.pose.orientation.w = float(orientation["w"])
        return pose
