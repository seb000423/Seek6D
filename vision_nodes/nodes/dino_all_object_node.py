#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Grounding DINO 전체 객체 검출 및 robot_base 좌표 DB 저장 ROS 2 노드.

통신 구조
- 서버: `/update_tcp_pose` (`interfaces/srv/UpdateTcpPose`)
  control 노드가 현재 TCP posx [x, y, z, rx, ry, rz]를 전달합니다.
  이 노드는 최신 TCP를 저장하고 수신 성공/실패만 응답합니다.
- 서버: `/set_picked_object` (`vision_nodes/srv/SetPickedObject`)
  픽업 완료된 model_name을 받아 해당 클래스를 전체 객체 검출에서 제외합니다.
  TCP는 이 요청에 포함하지 않고 `/update_tcp_pose`로 받은 최신 값을 사용합니다.
- 클라이언트: `/db_save` (`interfaces/srv/DbSave`)
  각 바운딩 박스 중심 픽셀의 aligned depth를 카메라 3D 점으로 역투영하고,
  T_base_gripper @ T_gripper_camera @ T_camera_point로 robot_base 좌표를 계산합니다.
  DB에는 DbSave.request JSON으로 {"table":"items","rows":[...]} 형식을 전송하며 Any6D는 사용하지 않습니다.

`/set_picked_object` 요청 시점의 RGB/depth/CameraInfo와 최신 TCP를 복사한 뒤
백그라운드에서 전체 검출, 중심점 좌표 변환, DB 저장을 수행합니다.
"""

from __future__ import annotations

import argparse
import json
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

try:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
except ImportError as exc:
    raise SystemExit(
        "ROS 2 Python 패키지를 불러오지 못했습니다. "
        "ROS 2 Humble 환경에서 실행하세요.\n"
        f"원래 오류: {exc}"
    )

try:
    from interfaces.srv import DbSave, UpdateTcpPose
    from vision_nodes.srv import SetPickedObject
except ImportError as exc:
    raise SystemExit(
        "interfaces.srv.DbSave/UpdateTcpPose 또는 "
        "vision_nodes.srv.SetPickedObject를 불러오지 못했습니다.\n"
        f"원래 오류: {exc}"
    )

try:
    from groundingdino.util.inference import Model
except ImportError as exc:
    raise SystemExit(
        "GroundingDINO를 불러오지 못했습니다. "
        "GroundingDINO 환경을 활성화하세요.\n"
        f"원래 오류: {exc}"
    )


CLASS_DEFINITIONS = (
    ("yellow_can", "yellow cylindrical can"),
    ("green_box", "green rectangular box"),
    ("gray_box", "gray rectangular storage box"),
    ("white_bear", "white bear plush toy"),
    ("aircon_remote", "gray air conditioner remote control"),
    ("green_frog", "green frog plush toy"),
    ("otter_in_can", "otter plush toy in a wooden barrel"),
)

CLASS_NAMES = tuple(name for name, _ in CLASS_DEFINITIONS)
CLASS_PROMPTS = tuple(prompt for _, prompt in CLASS_DEFINITIONS)
CLASS_NAME_SET = frozenset(CLASS_NAMES)

COLORS = (
    (0, 220, 255),
    (0, 200, 0),
    (160, 160, 160),
    (255, 180, 0),
    (220, 80, 220),
    (40, 220, 120),
    (255, 120, 60),
)

RESET_NAMES = {"", "none", "clear", "reset"}


@dataclass(frozen=True)
class Candidate:
    box: np.ndarray
    confidence: float
    class_id: int
    class_name: str


@dataclass(frozen=True)
class LocalizedCandidate:
    candidate: Candidate
    center_u: int
    center_v: int
    depth_m: float
    T_base_point_mm: np.ndarray


def box_area(box: np.ndarray) -> float:
    width = max(0.0, float(box[2] - box[0]))
    height = max(0.0, float(box[3] - box[1]))
    return width * height


def intersection_area(a: np.ndarray, b: np.ndarray) -> float:
    width = max(0.0, min(float(a[2]), float(b[2])) - max(float(a[0]), float(b[0])))
    height = max(0.0, min(float(a[3]), float(b[3])) - max(float(a[1]), float(b[1])))
    return width * height


def is_same_object(
    a: np.ndarray,
    b: np.ndarray,
    iou_threshold: float,
    ios_threshold: float,
) -> bool:
    intersection = intersection_area(a, b)
    if intersection <= 0.0:
        return False

    area_a = box_area(a)
    area_b = box_area(b)
    union = area_a + area_b - intersection
    smaller = min(area_a, area_b)

    iou = intersection / union if union > 0.0 else 0.0
    ios = intersection / smaller if smaller > 0.0 else 0.0
    return iou >= iou_threshold or ios >= ios_threshold


def select_unique_candidates(
    candidates: Sequence[Candidate],
    iou_threshold: float,
    ios_threshold: float,
) -> list[Candidate]:
    """클래스별 최대 1개와 같은 물체에 대한 중복 박스를 제거합니다."""
    selected: list[Candidate] = []
    selected_class_ids: set[int] = set()

    for candidate in sorted(candidates, key=lambda item: item.confidence, reverse=True):
        if candidate.class_id in selected_class_ids:
            continue

        if any(
            is_same_object(
                candidate.box,
                accepted.box,
                iou_threshold,
                ios_threshold,
            )
            for accepted in selected
        ):
            continue

        selected.append(candidate)
        selected_class_ids.add(candidate.class_id)

    return selected


def draw_results(
    image: np.ndarray,
    detections: Sequence[Candidate],
    localized: Sequence[LocalizedCandidate],
    excluded_class: Optional[str],
) -> np.ndarray:
    output = image.copy()
    height, width = output.shape[:2]
    localized_by_name = {item.candidate.class_name: item for item in localized}

    for item in detections:
        x1, y1, x2, y2 = np.round(item.box).astype(int)
        x1 = int(np.clip(x1, 0, width - 1))
        x2 = int(np.clip(x2, 0, width - 1))
        y1 = int(np.clip(y1, 0, height - 1))
        y2 = int(np.clip(y2, 0, height - 1))

        color = COLORS[item.class_id % len(COLORS)]
        pose_item = localized_by_name.get(item.class_name)
        if pose_item is None:
            label = f"{item.class_name} {item.confidence:.3f} | depth invalid"
        else:
            x_mm, y_mm, z_mm = pose_item.T_base_point_mm[:3, 3]
            label = (
                f"{item.class_name} {item.confidence:.3f} | "
                f"B({x_mm:.0f},{y_mm:.0f},{z_mm:.0f})mm"
            )
            cv2.drawMarker(
                output,
                (pose_item.center_u, pose_item.center_v),
                color,
                markerType=cv2.MARKER_CROSS,
                markerSize=16,
                thickness=2,
            )

        cv2.rectangle(output, (x1, y1), (x2, y2), color, 3)
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 2
        )
        label_top = max(0, y1 - text_height - baseline - 8)
        label_bottom = min(height - 1, label_top + text_height + baseline + 8)
        label_right = min(width - 1, x1 + text_width + 10)
        cv2.rectangle(output, (x1, label_top), (label_right, label_bottom), color, -1)
        cv2.putText(
            output, label, (x1 + 5, label_bottom - baseline - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 2, cv2.LINE_AA,
        )

    status = f"Detected: {len(detections)} | localized: {len(localized)}"
    if excluded_class:
        status += f" | excluded: {excluded_class}"
    cv2.putText(
        output, status, (15, 30), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (0, 0, 255), 2, cv2.LINE_AA,
    )
    return output


class DinoAllObjectsService(Node):
    def __init__(self, args: argparse.Namespace, T_gripper_camera_mm: np.ndarray) -> None:
        super().__init__("dino_all_object_node")
        self.args = args
        self.T_gripper_camera_mm = np.asarray(
            T_gripper_camera_mm, dtype=np.float64
        ).reshape(4, 4)
        self.bridge = CvBridge()

        self.frame_lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.tcp_lock = threading.Lock()
        self.latest_tcp_posx: Optional[np.ndarray] = None
        self.latest_tcp_received_ns = -1
        self.color_bgr: Optional[np.ndarray] = None
        self.depth_raw: Optional[np.ndarray] = None
        self.K: Optional[np.ndarray] = None
        self.color_stamp_ns = -1
        self.depth_stamp_ns = -1
        self.picked_class: Optional[str] = None

        # OpenCV GUI는 ROS executor callback이 아니라 전용 스레드 한 곳에서만 다룹니다.
        # 평상시에는 창을 만들지 않고, /set_picked_object 요청으로 프레임을 캡처한
        # 순간에만 창을 열어 캡처 프레임과 검출 결과를 표시합니다.
        self.display_lock = threading.Lock()
        self.display_event = threading.Event()
        self.display_stop = threading.Event()
        self.display_frame: Optional[np.ndarray] = None
        self.display_until: Optional[float] = None
        self.display_generation = 0
        self.display_recreate = False
        self.display_thread = threading.Thread(
            target=self._display_loop,
            name="all-object-display",
            daemon=True,
        )
        self.display_thread.start()

        self.get_logger().info("Grounding DINO 모델을 불러오는 중...")
        self.model = Model(
            model_config_path=str(Path(args.config).expanduser()),
            model_checkpoint_path=str(Path(args.weights).expanduser()),
        )
        self.get_logger().info("Grounding DINO 모델 로드 완료")

        camera_group = ReentrantCallbackGroup()
        picked_group = MutuallyExclusiveCallbackGroup()
        tcp_group = ReentrantCallbackGroup()
        db_group = ReentrantCallbackGroup()

        self.color_sub = self.create_subscription(
            Image, args.color_topic, self.color_callback, 10,
            callback_group=camera_group,
        )
        self.depth_sub = self.create_subscription(
            Image, args.depth_topic, self.depth_callback, 10,
            callback_group=camera_group,
        )
        self.info_sub = self.create_subscription(
            CameraInfo, args.info_topic, self.info_callback, 10,
            callback_group=camera_group,
        )

        # control 노드가 현재 TCP pose를 갱신하는 서비스입니다.
        self.tcp_pose_service = self.create_service(
            UpdateTcpPose,
            args.update_tcp_service,
            self.update_tcp_pose_callback,
            callback_group=tcp_group,
        )

        # 픽업 완료 클래스를 받아 전체 검출을 시작하는 서비스입니다.
        self.picked_service = self.create_service(
            SetPickedObject,
            args.picked_service,
            self.picked_callback,
            callback_group=picked_group,
        )

        # 검출 결과를 DB 노드에 보내기 위한 클라이언트입니다.
        self.db_client = self.create_client(
            DbSave,
            args.db_save_service,
            callback_group=db_group,
        )

        self.get_logger().info(f"TCP 갱신 서비스: {args.update_tcp_service}")
        self.get_logger().info(f"작업 트리거 서비스: {args.picked_service}")
        self.get_logger().info(f"DB 저장 클라이언트: {args.db_save_service}")
        self.get_logger().info(f"컬러 토픽: {args.color_topic}")
        self.get_logger().info(f"정렬 depth 토픽: {args.depth_topic}")
        self.get_logger().info(f"CameraInfo 토픽: {args.info_topic}")
        self.get_logger().info(f"hand-eye: {args.gripper_camera}")
        self.get_logger().info(f"confidence 기준: {args.conf_threshold:.2f}")

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    def color_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.frame_lock:
                self.color_bgr = np.asarray(frame).copy()
                self.color_stamp_ns = self._stamp_ns(msg)
        except Exception as exc:
            self.get_logger().error(f"컬러 이미지 변환 실패: {exc}")

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.frame_lock:
                self.depth_raw = np.asarray(depth).copy()
                self.depth_stamp_ns = self._stamp_ns(msg)
        except Exception as exc:
            self.get_logger().error(f"depth 이미지 변환 실패: {exc}")

    def info_callback(self, msg: CameraInfo) -> None:
        if len(msg.k) != 9:
            return
        with self.frame_lock:
            self.K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)

    def update_tcp_pose_callback(self, request, response):
        """control 노드가 전달한 최신 TCP posx를 저장합니다."""
        response.success = False
        response.message = ""

        tcp_posx = np.asarray(request.tcp_pose, dtype=np.float64).reshape(-1)
        if tcp_posx.size != 6:
            response.message = (
                f"tcp_pose 길이는 6이어야 합니다. 받은 길이={tcp_posx.size}"
            )
            self.get_logger().warning(response.message)
            return response

        if not np.all(np.isfinite(tcp_posx)):
            response.message = "tcp_pose에 NaN 또는 infinity가 포함되어 있습니다."
            self.get_logger().warning(response.message)
            return response

        with self.tcp_lock:
            self.latest_tcp_posx = tcp_posx.copy()
            self.latest_tcp_received_ns = self.get_clock().now().nanoseconds

        response.success = True
        response.message = "현재 TCP pose를 저장했습니다."
        self.get_logger().info(
            "TCP pose 갱신: "
            f"x={tcp_posx[0]:.3f}, y={tcp_posx[1]:.3f}, z={tcp_posx[2]:.3f}, "
            f"rx={tcp_posx[3]:.3f}, ry={tcp_posx[4]:.3f}, rz={tcp_posx[5]:.3f}"
        )
        return response

    def get_latest_tcp_pose(self) -> Optional[np.ndarray]:
        """가장 최근에 수신한 TCP pose의 복사본을 반환합니다."""
        with self.tcp_lock:
            if self.latest_tcp_posx is None:
                return None
            return self.latest_tcp_posx.copy()

    def get_latest_rgbd(self):
        with self.frame_lock:
            if self.color_bgr is None or self.depth_raw is None or self.K is None:
                return None
            delta_ms = abs(self.color_stamp_ns - self.depth_stamp_ns) / 1_000_000.0
            if delta_ms > self.args.sync_tolerance_ms:
                return None
            color = self.color_bgr.copy()
            depth_raw = self.depth_raw.copy()
            K = self.K.copy()

        depth_m = depth_raw.astype(np.float32)
        if np.issubdtype(depth_raw.dtype, np.integer):
            depth_m *= self.args.depth_scale

        color_h, color_w = color.shape[:2]
        depth_h, depth_w = depth_m.shape[:2]
        if (depth_h, depth_w) != (color_h, color_w):
            scale_x = color_w / float(depth_w)
            scale_y = color_h / float(depth_h)
            depth_m = cv2.resize(
                depth_m, (color_w, color_h), interpolation=cv2.INTER_NEAREST
            )
            K[0, 0] *= scale_x
            K[0, 2] *= scale_x
            K[1, 1] *= scale_y
            K[1, 2] *= scale_y

        return color, depth_m, K


    def _make_captured_preview(
        self,
        frame: np.ndarray,
        excluded_class: str,
    ) -> np.ndarray:
        """서비스 호출 시점에 고정한 카메라 프레임을 분석 중 화면으로 만듭니다."""
        preview = frame.copy()
        cv2.putText(
            preview,
            "CAPTURED FRAME - DINO ANALYZING...",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            f"excluded: {excluded_class}",
            (15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return preview

    def _show_captured_frame(
        self,
        frame: np.ndarray,
        excluded_class: str,
    ) -> None:
        """기존 창을 닫고 새 캡처 프레임용 창을 다시 열도록 예약합니다."""
        preview = self._make_captured_preview(frame, excluded_class)
        with self.display_lock:
            self.display_generation += 1
            self.display_frame = preview
            self.display_until = None  # DINO 결과가 나올 때까지 유지
            self.display_recreate = True
        self.display_event.set()

    def _show_result_frame(self, frame: np.ndarray) -> None:
        """바운딩 박스 결과를 설정하고 지정 시간 뒤 자동으로 창을 닫습니다."""
        with self.display_lock:
            self.display_frame = frame.copy()
            self.display_until = (
                time.monotonic() + max(0.0, self.args.result_display_sec)
            )
        self.display_event.set()

    def _show_error_frame(
        self,
        frame: np.ndarray,
        excluded_class: str,
        message: str,
    ) -> None:
        preview = frame.copy()
        cv2.putText(
            preview,
            "DINO DETECTION FAILED",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            f"excluded: {excluded_class}",
            (15, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            message[:100],
            (15, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
        self._show_result_frame(preview)

    def _close_display(self) -> None:
        with self.display_lock:
            self.display_generation += 1
            self.display_frame = None
            self.display_until = None
            self.display_recreate = False
        self.display_event.set()

    def _display_loop(self) -> None:
        """OpenCV 창 생성, 갱신, 종료를 전담합니다.

        창은 캡처 요청이 없을 때 존재하지 않습니다. 새 요청이 들어오면 기존 창을
        destroy한 뒤 새 캡처 프레임으로 다시 만들고, 결과는 기본 10초 후 닫습니다.
        """
        title = self.args.window_name
        window_open = False
        active_generation = -1

        while not self.display_stop.is_set():
            now = time.monotonic()
            with self.display_lock:
                generation = self.display_generation
                frame = (
                    None
                    if self.display_frame is None
                    else self.display_frame.copy()
                )
                deadline = self.display_until
                recreate = self.display_recreate

            if frame is not None and deadline is not None and now >= deadline:
                with self.display_lock:
                    if generation == self.display_generation:
                        self.display_frame = None
                        self.display_until = None
                        self.display_recreate = False
                frame = None

            if frame is None:
                if window_open:
                    try:
                        cv2.destroyWindow(title)
                        cv2.waitKey(1)
                    except cv2.error:
                        pass
                    window_open = False
                    active_generation = generation
                self.display_event.wait(timeout=0.05)
                self.display_event.clear()
                continue

            if recreate or generation != active_generation or not window_open:
                if window_open:
                    try:
                        cv2.destroyWindow(title)
                        cv2.waitKey(1)
                    except cv2.error:
                        pass
                cv2.namedWindow(title, cv2.WINDOW_NORMAL)
                window_open = True
                active_generation = generation
                with self.display_lock:
                    if generation == self.display_generation:
                        self.display_recreate = False

            cv2.imshow(title, frame)
            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                self._close_display()
                continue

            # 사용자가 창의 X 버튼을 누른 경우에도 내부 상태를 비웁니다.
            try:
                if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                    self._close_display()
                    window_open = False
            except cv2.error:
                window_open = False

        if window_open:
            try:
                cv2.destroyWindow(title)
                cv2.waitKey(1)
            except cv2.error:
                pass

    def stop_display(self) -> None:
        self.display_stop.set()
        self.display_event.set()
        if self.display_thread.is_alive():
            self.display_thread.join(timeout=2.0)

    def run_detection(
        self,
        image: np.ndarray,
        excluded_class: str,
    ) -> list[Candidate]:
        """전체 고정 클래스를 검출하고 요청받은 클래스는 후보 단계에서 제외합니다."""
        detections = self.model.predict_with_classes(
            image=image,
            classes=CLASS_PROMPTS,
            box_threshold=self.args.conf_threshold,
            text_threshold=self.args.text_threshold,
        )

        candidates: list[Candidate] = []
        for box, confidence, class_id in zip(
            np.asarray(detections.xyxy),
            np.asarray(detections.confidence),
            np.asarray(detections.class_id),
        ):
            if class_id is None:
                continue

            class_id = int(class_id)
            confidence = float(confidence)
            if not 0 <= class_id < len(CLASS_NAMES):
                continue
            if confidence < self.args.conf_threshold:
                continue

            class_name = CLASS_NAMES[class_id]
            if class_name == excluded_class:
                continue

            candidates.append(
                Candidate(
                    box=np.asarray(box, dtype=np.float32),
                    confidence=confidence,
                    class_id=class_id,
                    class_name=class_name,
                )
            )

        return select_unique_candidates(
            candidates,
            self.args.same_object_iou,
            self.args.same_object_ios,
        )

    @staticmethod
    def doosan_posx_to_matrix(posx: Sequence[float]) -> np.ndarray:
        x, y, z, rx, ry, rz = [float(value) for value in posx]
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = Rotation.from_euler(
            "ZYZ", [rx, ry, rz], degrees=True
        ).as_matrix()
        matrix[:3, 3] = [x, y, z]
        return matrix

    @staticmethod
    def camera_point_to_base(
        point_camera_m: np.ndarray,
        T_gripper_camera_mm: np.ndarray,
        T_base_gripper_mm: np.ndarray,
    ) -> np.ndarray:
        """카메라 점을 robot_base 좌표의 3D 점으로 변환합니다.

        바운딩 박스 중심 픽셀과 depth로 위치만 계산합니다.
        반환 행렬의 회전부는 좌표 변환 과정에만 사용하며 DB에는 보내지 않습니다.
        """
        T_camera_point_mm = np.eye(4, dtype=np.float64)
        T_camera_point_mm[:3, 3] = (
            np.asarray(point_camera_m, dtype=np.float64).reshape(3) * 1000.0
        )
        return T_base_gripper_mm @ T_gripper_camera_mm @ T_camera_point_mm

    def _center_depth(
        self, depth_m: np.ndarray, u: int, v: int
    ) -> Optional[float]:
        value = float(depth_m[v, u])
        if np.isfinite(value) and self.args.min_depth_m <= value <= self.args.max_depth_m:
            return value

        radius = max(0, int(self.args.center_depth_radius))
        y1 = max(0, v - radius)
        y2 = min(depth_m.shape[0], v + radius + 1)
        x1 = max(0, u - radius)
        x2 = min(depth_m.shape[1], u + radius + 1)
        patch = depth_m[y1:y2, x1:x2]
        valid = patch[
            np.isfinite(patch)
            & (patch >= self.args.min_depth_m)
            & (patch <= self.args.max_depth_m)
        ]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def localize_candidates(
        self,
        selected: Sequence[Candidate],
        depth_m: np.ndarray,
        K: np.ndarray,
        tcp_posx: np.ndarray,
    ) -> list[LocalizedCandidate]:
        T_base_gripper_mm = self.doosan_posx_to_matrix(tcp_posx)
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("invalid camera intrinsics: fx/fy must be positive")

        height, width = depth_m.shape[:2]
        localized: list[LocalizedCandidate] = []
        for candidate in selected:
            x1, y1, x2, y2 = np.asarray(candidate.box, dtype=np.float64)
            u = int(np.clip(round((x1 + x2) * 0.5), 0, width - 1))
            v = int(np.clip(round((y1 + y2) * 0.5), 0, height - 1))
            depth = self._center_depth(depth_m, u, v)
            if depth is None:
                self.get_logger().warning(
                    f"중심 depth 없음: class={candidate.class_name}, pixel=({u},{v})"
                )
                continue

            point_camera_m = np.array([
                (u - cx) * depth / fx,
                (v - cy) * depth / fy,
                depth,
            ], dtype=np.float64)
            T_base_point_mm = self.camera_point_to_base(
                point_camera_m, self.T_gripper_camera_mm, T_base_gripper_mm
            )
            localized.append(LocalizedCandidate(
                candidate=candidate,
                center_u=u,
                center_v=v,
                depth_m=depth,
                T_base_point_mm=T_base_point_mm,
            ))

            bx, by, bz = T_base_point_mm[:3, 3]
            self.get_logger().info(
                f"{candidate.class_name}: pixel=({u},{v}), depth={depth:.4f}m, "
                f"base=({bx:.1f},{by:.1f},{bz:.1f})mm"
            )
        return localized

    def build_db_request(
        self,
        localized: Sequence[LocalizedCandidate],
    ):
        """DbSave.request에 table/rows 형식의 JSON 문자열을 만듭니다.

        전송 형식:
        {
          "table": "items",
          "rows": [
            {
              "class_name": "...",
              "confidence": 0.0,
              "x": 0.0,
              "y": 0.0,
              "z": 0.0
            }
          ]
        }

        x/y/z는 robot_base 좌표계 기준이며 단위는 mm입니다.
        """
        rows = []
        for item in localized:
            rows.append({
                "class_name": item.candidate.class_name,
                "confidence": float(item.candidate.confidence),
                "x": float(item.T_base_point_mm[0, 3]),
                "y": float(item.T_base_point_mm[1, 3]),
                "z": float(item.T_base_point_mm[2, 3]),
            })

        payload = {
            "table": "items",
            "rows": rows,
        }

        request = DbSave.Request()
        request.request = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        return request

    def call_db_save(self, request) -> tuple[bool, str, str]:
        """DB 노드의 DbSave 서비스를 호출하고 응답 완료까지 기다립니다."""
        if not self.db_client.wait_for_service(timeout_sec=self.args.db_wait_sec):
            return (
                False,
                "",
                f"DB 저장 서비스를 찾을 수 없습니다: {self.args.db_save_service}",
            )

        future = self.db_client.call_async(request)
        completed = threading.Event()
        holder: dict[str, object] = {}

        def done_callback(done_future) -> None:
            try:
                holder["response"] = done_future.result()
            except Exception as exc:  # pragma: no cover - ROS runtime path
                holder["error"] = exc
            finally:
                completed.set()

        future.add_done_callback(done_callback)

        if not completed.wait(timeout=self.args.db_response_timeout_sec):
            return (
                False,
                "",
                f"DB 저장 서비스 응답 시간 초과: {self.args.db_save_service}",
            )

        error = holder.get("error")
        if error is not None:
            return False, "", f"DB 저장 서비스 호출 실패: {error}"

        result = holder.get("response")
        if result is None:
            return False, "", "DB 저장 서비스가 응답을 반환하지 않았습니다."

        return bool(result.success), str(result.response), str(result.message)

    def picked_callback(self, request, response):
        """제외 클래스를 접수하고 전체 검출 작업을 백그라운드로 시작합니다.

        이 서비스의 success는 요청 유효성 및 작업 시작 여부만 뜻합니다.
        DB 저장 성공/실패는 Any6D 노드에 전달하지 않습니다.
        """
        response.success = False
        response.message = ""
        class_name = str(request.model_name).strip()

        if class_name.lower() in RESET_NAMES:
            previous = self.picked_class
            self.picked_class = None
            self._close_display()
            response.success = True
            response.message = f"제외 클래스를 초기화했습니다. 이전 값={previous}."
            self.get_logger().info(response.message)
            return response

        if class_name not in CLASS_NAME_SET:
            response.message = (
                f"알 수 없는 클래스입니다: {class_name}. "
                f"허용값={','.join(CLASS_NAMES)}"
            )
            self.get_logger().warning(response.message)
            return response

        if not self.processing_lock.acquire(blocking=False):
            response.message = "이전 전체 검출 작업이 아직 처리 중입니다."
            self.get_logger().warning(response.message)
            return response

        tcp_posx = self.get_latest_tcp_pose()
        if tcp_posx is None:
            self.processing_lock.release()
            response.message = (
                f"현재 TCP pose가 없습니다. 먼저 {self.args.update_tcp_service} "
                "서비스로 TCP를 전달하세요."
            )
            self.get_logger().warning(response.message)
            return response

        # 서비스 호출 시점의 컬러/depth/K와 최신 TCP를 고정합니다.
        frame_data = self.get_latest_rgbd()
        if frame_data is None:
            self.processing_lock.release()
            response.message = (
                "동기화된 컬러/depth/CameraInfo를 아직 받지 못했습니다."
            )
            self.get_logger().warning(response.message)
            return response
        frame, depth_m, K = frame_data

        self.picked_class = class_name

        # 새 캡처가 들어오면 이전 결과 창을 즉시 닫고, 이번 요청에서 고정한
        # 프레임을 새 창에 표시합니다. DINO가 끝나면 같은 창이 결과로 갱신됩니다.
        self._show_captured_frame(frame, class_name)

        worker = threading.Thread(
            target=self._process_picked_request,
            args=(class_name, frame, depth_m, K, tcp_posx),
            name=f"all-object-{class_name}",
            daemon=True,
        )
        try:
            worker.start()
        except Exception as exc:
            self.processing_lock.release()
            self._show_error_frame(frame, class_name, str(exc))
            response.message = f"전체 검출 작업 시작 실패: {exc}"
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = (
            f"{class_name} 제외 전체 검출 및 base 좌표 계산을 시작했습니다."
        )
        self.get_logger().info(response.message)
        return response

    def _process_picked_request(
        self,
        class_name: str,
        frame: np.ndarray,
        depth_m: np.ndarray,
        K: np.ndarray,
        tcp_posx: np.ndarray,
    ) -> None:
        """DINO 검출/화면 표시/이미지 저장 후 DB 저장을 독립적으로 시도합니다."""
        try:
            self.get_logger().info(
                f"전체 검출 시작: excluded={class_name}; captured_frame=yes"
            )

            try:
                selected = self.run_detection(frame, excluded_class=class_name)
            except Exception as exc:
                self.get_logger().exception(f"Grounding DINO 분석 실패: {exc}")
                self._show_error_frame(frame, class_name, str(exc))
                return

            localized = self.localize_candidates(selected, depth_m, K, tcp_posx)

            # DB 상태와 관계없이 검출 및 좌표 결과를 먼저 표시하고 저장합니다.
            result_image = draw_results(frame, selected, localized, class_name)
            self._show_result_frame(result_image)
            self.save_result_image(result_image)
            self.get_logger().info(
                f"전체 검출 완료: excluded={class_name}, detected={len(selected)}, "
                f"localized={len(localized)}"
            )

            db_request = self.build_db_request(localized)
            self.get_logger().info(
                f"DB 전송 시도: excluded={class_name}, "
                f"items={len(localized)}, request={db_request.request}"
            )

            db_success, db_response, db_message = self.call_db_save(db_request)
            if not db_success:
                detail = f"DB 저장 실패(검출/표시는 정상 완료): {db_message}"
                if db_response:
                    detail += f"; db_response={db_response}"
                self.get_logger().warning(detail)
                return

            detail = (
                f"DB 저장 완료: excluded={class_name}, items={len(localized)}, "
                f"message={db_message}"
            )
            if db_response:
                detail += f"; db_response={db_response}"
            self.get_logger().info(detail)
        finally:
            self.processing_lock.release()

    def save_result_image(self, image: np.ndarray) -> None:
        if not self.args.output:
            return

        output_path = Path(self.args.output).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), image):
            self.get_logger().warning(f"결과 이미지 저장 실패: {output_path}")

    def destroy_node(self) -> bool:
        self.stop_display()
        cv2.destroyAllWindows()
        return super().destroy_node()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Grounding DINO 전체 객체 검출 및 선택적 DbSave 호출 ROS 2 노드"
    )
    dino_home = Path(
        os.getenv("GROUNDINGDINO_HOME", "~/GroundingDINO")
    ).expanduser()
    parser.add_argument(
        "--config",
        default=str(
            dino_home
            / "groundingdino/config/GroundingDINO_SwinT_OGC.py"
        ),
        help="GroundingDINO config 경로",
    )
    parser.add_argument(
        "--weights",
        default=str(
            dino_home
            / "weights/groundingdino_swint_ogc.pth"
        ),
        help="GroundingDINO checkpoint 경로",
    )
    parser.add_argument(
        "--color-topic",
        default="/camera/camera/color/image_raw",
        help="RealSense 컬러 이미지 토픽",
    )
    parser.add_argument(
        "--depth-topic",
        default="/camera/camera/aligned_depth_to_color/image_raw",
        help="컬러에 정렬된 RealSense depth 토픽",
    )
    parser.add_argument(
        "--info-topic",
        default="/camera/camera/aligned_depth_to_color/camera_info",
        help="aligned depth CameraInfo 토픽",
    )
    parser.add_argument(
        "--gripper-camera",
        default="~/Any6D/T_gripper2camera.npy",
        help="T_gripper_camera 4x4 hand-eye 행렬 경로",
    )
    parser.add_argument(
        "--sync-tolerance-ms", type=float, default=80.0,
        help="컬러-depth 최대 timestamp 차이(ms)",
    )
    parser.add_argument(
        "--depth-scale", type=float, default=0.001,
        help="정수 depth를 metre로 바꾸는 배율",
    )
    parser.add_argument(
        "--center-depth-radius", type=int, default=3,
        help="중심 픽셀 depth가 0일 때 사용할 주변 반경(pixel)",
    )
    parser.add_argument("--min-depth-m", type=float, default=0.10)
    parser.add_argument("--max-depth-m", type=float, default=2.00)
    parser.add_argument(
        "--update-tcp-service",
        default="/update_tcp_pose",
        help="control 노드에서 현재 TCP pose를 받는 서비스명",
    )
    parser.add_argument(
        "--picked-service",
        default="/set_picked_object",
        help="픽업 완료된 제외 class ID를 받는 서비스명",
    )
    parser.add_argument(
        "--db-save-service",
        default="/db_save",
        help="검출 결과를 전송할 interfaces/srv/DbSave 서비스명",
    )
    parser.add_argument(
        "--db-wait-sec",
        type=float,
        default=5.0,
        help="DB 서비스 발견 대기 시간(초)",
    )
    parser.add_argument(
        "--db-response-timeout-sec",
        type=float,
        default=10.0,
        help="DB 서비스 응답 대기 시간(초)",
    )
    parser.add_argument(
        "--conf-threshold",
        type=float,
        default=0.40,
        help="검출 및 표시 최소 confidence (기본: 0.40)",
    )
    parser.add_argument(
        "--text-threshold",
        type=float,
        default=0.10,
        help="Grounding DINO text threshold (기본: 0.10)",
    )
    parser.add_argument("--same-object-iou", type=float, default=0.45)
    parser.add_argument("--same-object-ios", type=float, default=0.70)
    parser.add_argument(
        "--result-display-sec",
        type=float,
        default=10.0,
        help="검출 결과 화면 유지 시간 (기본: 10초)",
    )
    parser.add_argument(
        "--window-name",
        default="Grounding DINO All Objects",
    )
    parser.add_argument(
        "--output",
        default="",
        help="결과 이미지 저장 경로. 빈 값이면 저장하지 않음",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    handeye_path = Path(args.gripper_camera).expanduser()
    if not handeye_path.is_file():
        raise SystemExit(f"hand-eye 행렬을 찾을 수 없습니다: {handeye_path}")
    T_gripper_camera_mm = np.load(handeye_path)
    if T_gripper_camera_mm.shape != (4, 4):
        raise SystemExit("T_gripper2camera.npy는 4x4 행렬이어야 합니다.")
    if not np.all(np.isfinite(T_gripper_camera_mm)):
        raise SystemExit("T_gripper2camera.npy에 NaN/inf가 있습니다.")

    rclpy.init()
    node = DinoAllObjectsService(args, T_gripper_camera_mm)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
