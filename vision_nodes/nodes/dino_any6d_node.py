#!/usr/bin/env python3
"""Grounding DINO + Any6D camera-coordinate ROS 2 service server.

Application communication is service-only:
  request.request   : JSON string from DetectionClient
  response.success  : detection + Any6D success
  response.response : JSON string matching DetectionClient parser
  response.message  : human-readable status/error message

Pose output is always T_camera_object. Pose position is expressed in metres.

The RealSense driver is still consumed through its standard ROS image topics.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

import cv2
import numpy as np
import rclpy
import torch
import trimesh
from cv_bridge import CvBridge
from PIL import Image as PILImage
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

import groundingdino.datasets.transforms as T
from groundingdino.util.inference import load_model, predict

from interfaces.srv import DetectObject
from vision_nodes.srv import SetPickedObject

ROBOT_ID = "dsr01"


@dataclass(frozen=True)
class ObjectProfile:
    model_name: str
    display_name: str
    dino_prompt: str
    mesh_path: Path
    height_mm: float
    min_confidence: float


@dataclass
class Detection:
    xyxy: np.ndarray
    confidence: float


class GroundingDinoDetector:
    def __init__(self, config: str, weights: str, device: str, image_size: int,
                 box_threshold: float, text_threshold: float, nms_iou: float) -> None:
        self.device = device
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        self.nms_iou = nms_iou
        self.model = load_model(config, weights, device=device)
        self.transform = T.Compose([
            T.RandomResize([image_size], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])

    def detect(self, bgr: np.ndarray, prompt: str) -> list[Detection]:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        tensor, _ = self.transform(PILImage.fromarray(rgb), None)
        boxes, logits, _ = predict(
            model=self.model,
            image=tensor,
            caption=prompt.rstrip(".") + ".",
            box_threshold=self.box_threshold,
            text_threshold=self.text_threshold,
            device=self.device,
        )
        if len(boxes) == 0:
            return []
        h, w = bgr.shape[:2]
        boxes_np = boxes.detach().cpu().numpy()
        scores = logits.detach().cpu().numpy()
        xyxy = [
            [(cx - bw / 2) * w, (cy - bh / 2) * h,
             (cx + bw / 2) * w, (cy + bh / 2) * h]
            for cx, cy, bw, bh in boxes_np
        ]
        keep = cv2.dnn.NMSBoxes(
            [[x1, y1, x2 - x1, y2 - y1] for x1, y1, x2, y2 in xyxy],
            scores.tolist(), self.box_threshold, self.nms_iou,
        )
        indices = np.asarray(keep).reshape(-1).tolist() if len(keep) else []
        return [Detection(np.asarray(xyxy[i], np.float32), float(scores[i])) for i in indices]


class FindObjectServer(Node):
    def __init__(self, args, profiles: dict[str, ObjectProfile], detector: GroundingDinoDetector,
                 any6d_cls) -> None:
        super().__init__("dino_any6d_node", namespace=ROBOT_ID)
        self.args = args
        self.profiles = profiles
        self.detector = detector
        self.Any6D = any6d_cls
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.color_bgr: np.ndarray | None = None
        self.depth_raw: np.ndarray | None = None
        self.K: np.ndarray | None = None
        self.color_stamp_ns = -1
        self.depth_stamp_ns = -1

        # OpenCV window is owned by one dedicated thread. Service callbacks only
        # update the overlay image, avoiding concurrent imshow/waitKey calls.
        self.display_lock = threading.Lock()
        self.display_overlay: np.ndarray | None = None
        self.display_overlay_until: float | None = None  # None = keep while processing
        self.display_stop = threading.Event()
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self.display_thread.start()

        camera_group = ReentrantCallbackGroup()
        detect_service_group = MutuallyExclusiveCallbackGroup()
        picked_client_group = ReentrantCallbackGroup()
        self.create_subscription(Image, args.color_topic, self._color_cb, 10, callback_group=camera_group)
        self.create_subscription(Image, args.depth_topic, self._depth_cb, 10, callback_group=camera_group)
        self.create_subscription(CameraInfo, args.info_topic, self._info_cb, 10, callback_group=camera_group)
        self.service = self.create_service(
            DetectObject, args.service, self._find_object_cb, callback_group=detect_service_group
        )
        # /set_picked_object 서버는 all_detection 노드가 소유합니다.
        # 이 Any6D 노드는 /find_object_pose로 받은 class_label을 해당 서버로 전달하는 클라이언트입니다.
        self.picked_client = self.create_client(
            SetPickedObject,
            args.picked_service,
            callback_group=picked_client_group,
        )
        self.get_logger().info(f"service ready: {args.service}")
        self.get_logger().info(
            f"picked-object forwarding client: {args.picked_service}"
        )

    @staticmethod
    def _stamp_ns(msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _color_cb(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.frame_lock:
                self.color_bgr = np.asarray(image).copy()
                self.color_stamp_ns = self._stamp_ns(msg)
        except Exception as exc:
            self.get_logger().error(f"color conversion failed: {exc}")

    def _depth_cb(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.frame_lock:
                self.depth_raw = np.asarray(image).copy()
                self.depth_stamp_ns = self._stamp_ns(msg)
        except Exception as exc:
            self.get_logger().error(f"depth conversion failed: {exc}")

    def _info_cb(self, msg: CameraInfo) -> None:
        if len(msg.k) == 9:
            with self.frame_lock:
                self.K = np.asarray(msg.k, dtype=np.float32).reshape(3, 3)

    def _latest_frame(self):
        with self.frame_lock:
            if self.color_bgr is None or self.depth_raw is None or self.K is None:
                return None
            delta_ms = abs(self.color_stamp_ns - self.depth_stamp_ns) / 1_000_000.0
            if delta_ms > self.args.sync_tolerance_ms:
                return None
            return self.color_bgr.copy(), self.depth_raw.copy(), self.K.copy()


    def _resize_color_for_display(self, color_bgr: np.ndarray) -> np.ndarray:
        h, w = color_bgr.shape[:2]
        if self.args.max_width <= 0 or w <= self.args.max_width:
            return color_bgr.copy()
        scale = self.args.max_width / float(w)
        return cv2.resize(
            color_bgr,
            (int(round(w * scale)), int(round(h * scale))),
            interpolation=cv2.INTER_AREA,
        )

    def _set_overlay(self, frame: np.ndarray, hold_seconds: float | None = None) -> None:
        with self.display_lock:
            self.display_overlay = frame.copy()
            self.display_overlay_until = (
                None if hold_seconds is None else time.monotonic() + hold_seconds
            )

    def _finish_overlay(self, hold_seconds: float | None = None) -> None:
        duration = self.args.result_hold_seconds if hold_seconds is None else hold_seconds
        with self.display_lock:
            if self.display_overlay is not None:
                self.display_overlay_until = time.monotonic() + max(0.0, duration)

    def _show_error_overlay(self, message: str) -> None:
        item = self._latest_frame()
        if item is None:
            return
        frame = self._resize_color_for_display(item[0])
        cv2.putText(frame, "REQUEST FAILED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 255), 2)
        cv2.putText(frame, message[:90], (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255), 1)
        self._set_overlay(frame, self.args.result_hold_seconds)

    def _display_loop(self) -> None:
        title = self.args.window_title
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        while not self.display_stop.is_set():
            raw = None
            with self.frame_lock:
                if self.color_bgr is not None:
                    raw = self.color_bgr.copy()

            now = time.monotonic()
            overlay = None
            with self.display_lock:
                if (self.display_overlay is not None and
                        self.display_overlay_until is not None and
                        now >= self.display_overlay_until):
                    self.display_overlay = None
                    self.display_overlay_until = None
                if self.display_overlay is not None:
                    overlay = self.display_overlay.copy()

            if overlay is not None:
                view = overlay
            elif raw is not None:
                view = self._resize_color_for_display(raw)
                cv2.putText(view, "READY - waiting for service request", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2)
            else:
                view = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(view, "Waiting for RealSense image...", (20, 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

            cv2.imshow(title, view)
            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                self.display_stop.set()
                break
        cv2.destroyWindow(title)

    def stop_display(self) -> None:
        self.display_stop.set()
        if self.display_thread.is_alive():
            self.display_thread.join(timeout=1.0)

    @staticmethod
    def _resolve_request_model(model_name: str) -> str | None:
        """제어 요청 JSON에서 받은 class label을 내부 모델 키로 검증합니다."""
        normalized = model_name.strip().lower().replace(" ", "_")
        valid_models = {
            "otter_in_can",
            "white_bear",
            "green_frog",
            "yellow_can",
            "aircon_remote",
            "green_box",
            "gray_box",
        }
        return normalized if normalized in valid_models else None

    @staticmethod
    def _parse_control_request(raw_request: str) -> tuple[dict, str, str]:
        """DetectionClient가 보내는 JSON 문자열을 해석합니다.

        반환값: (원본 payload, request_id, 요청 class label)
        """
        try:
            payload = json.loads(raw_request)
        except json.JSONDecodeError as exc:
            raise ValueError(f"request is not valid JSON: {exc}") from exc
        if not isinstance(payload, dict):
            raise ValueError("request JSON must be an object")

        request_id = str(payload.get("request_id", "")).strip()
        if not request_id:
            raise ValueError("request_id is required")

        object_name = str(
            payload.get("class_label")
            or payload.get("object_name")
            or payload.get("name")
            or ""
        ).strip()
        if not object_name:
            raise ValueError("class_label/object_name/name is required")
        return payload, request_id, object_name

    def _make_detection_json(
        self,
        *,
        request_id: str,
        detected: bool,
        detected_profile: ObjectProfile | None = None,
        T_camera_object_m: np.ndarray | None = None,
    ) -> str:
        """제어 DetectionClient가 기대하는 JSON 형식으로 직렬화합니다.

        position 단위는 metre이고,
        orientation은 quaternion x/y/z/w입니다.
        """
        payload: dict[str, object] = {
            "request_id": request_id,
            "detected": bool(detected),
            "detected_name": "",
            "detected_class_label": "",
            "pose": None,
        }

        if detected:
            if detected_profile is None or T_camera_object_m is None:
                raise ValueError("detected response requires profile and pose")
            matrix = np.asarray(T_camera_object_m, dtype=np.float64).reshape(4, 4)
            quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
            now_msg = self.get_clock().now().to_msg()
            payload.update({
                "detected_name": detected_profile.display_name,
                "detected_class_label": detected_profile.model_name,
                "pose": {
                    "frame_id": "camera_color_optical_frame",
                    "stamp": {
                        "sec": int(now_msg.sec),
                        "nanosec": int(now_msg.nanosec),
                    },
                    "position": {
                        "x": float(matrix[0, 3]),
                        "y": float(matrix[1, 3]),
                        "z": float(matrix[2, 3]),
                    },
                    "orientation": {
                        "x": float(quaternion[0]),
                        "y": float(quaternion[1]),
                        "z": float(quaternion[2]),
                        "w": float(quaternion[3]),
                    },
                },
            })
        return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))

    def _notify_picked_object(self, model_name: str) -> bool:
        """all_object에 선택 클래스를 비동기로 알립니다.

        이 알림의 성공/실패는 제어의 /find_object_pose 응답에 영향을 주지 않습니다.
        all_object 노드가 없더라도 Any6D 검출과 카메라 기준 pose 반환은 계속됩니다.
        """
        if not self.picked_client.service_is_ready():
            self.get_logger().warning(
                f"all_object 알림 생략: 서비스 없음 {self.args.picked_service}; "
                "제어 pose 처리는 정상 계속합니다."
            )
            return False

        request = SetPickedObject.Request()
        request.model_name = model_name
        future = self.picked_client.call_async(request)

        def _done(done_future) -> None:
            try:
                result = done_future.result()
            except Exception as exc:
                self.get_logger().warning(
                    f"all_object 알림 호출 실패: class={model_name}; {exc}"
                )
                return

            if result is None:
                self.get_logger().warning(
                    f"all_object 알림 응답 없음: class={model_name}"
                )
            elif bool(result.success):
                self.get_logger().info(
                    f"all_object 알림 성공: class={model_name}; {result.message}"
                )
            else:
                self.get_logger().warning(
                    f"all_object 알림 거절: class={model_name}; {result.message}"
                )

        future.add_done_callback(_done)
        return True

    def _find_object_cb(self, request, response):
        response.success = False
        response.response = ""
        response.message = ""

        try:
            _, request_id, requested_name = self._parse_control_request(request.request)
        except ValueError as exc:
            response.message = str(exc)
            return response

        requested_model = self._resolve_request_model(requested_name)
        if requested_model is None:
            response.message = f"unsupported class_label: {requested_name!r}"
            return response

        if not self.processing_lock.acquire(blocking=False):
            response.message = "another object request is already being processed"
            return response

        try:
            self.get_logger().info(
                f"request_id={request_id}, class_label={requested_model}"
            )

            selected = self._find_best_detection(
                [requested_model],
                self.args.primary_detect_timeout,
                stage_name="requested-class",
            )

            used_fallback = False
            if selected is None:
                used_fallback = True
                fallback_names = ["green_box", "gray_box"]
                selected = self._find_best_detection(
                    fallback_names,
                    self.args.fallback_detect_timeout,
                    stage_name="fallback-box",
                ) if fallback_names else None

            # 통신 자체는 정상 처리되었으므로 success=True.
            # 물체 미검출 여부는 JSON 내부 detected=false로 구분합니다.
            if selected is None:
                response.success = True
                response.response = self._make_detection_json(
                    request_id=request_id,
                    detected=False,
                )
                response.message = (
                    f"not detected: {requested_model}, green_box, gray_box"
                )
                self.get_logger().warning(response.message)
                self._show_error_overlay("NO TARGET DETECTED")
                return response

            profile, detection, frame_data = selected
            self.get_logger().info(
                f"selected={profile.model_name}, confidence={detection.confidence:.3f}, "
                f"fallback={used_fallback}"
            )

            T_camera_object_m = self._estimate_selected_detection(
                profile, detection, frame_data
            )

            response.success = True
            response.response = self._make_detection_json(
                request_id=request_id,
                detected=True,
                detected_profile=profile,
                T_camera_object_m=T_camera_object_m,
            )
            response.message = (
                f"detected={profile.model_name}; frame=camera_color_optical_frame; "
                f"confidence={detection.confidence:.3f}; fallback_box={used_fallback}"
            )

            # 실제로 pose가 계산된 객체만 all_object에 알립니다.
            # 알림은 비동기이며 실패해도 제어 응답은 그대로 성공입니다.
            self._notify_picked_object(profile.model_name)

            output = Path(self.args.output).expanduser()
            output.parent.mkdir(parents=True, exist_ok=True)
            np.save(output, T_camera_object_m)
            np.savetxt(output.with_suffix(".txt"), T_camera_object_m, fmt="%.9f")
            x, y, z = np.asarray(T_camera_object_m)[:3, 3]
            self.get_logger().info(
                f"{profile.model_name} [camera]: "
                f"X={x:.4f}, Y={y:.4f}, Z={z:.4f} m"
            )
            self._finish_overlay()
        except Exception as exc:
            response.success = False
            response.response = ""
            response.message = str(exc)
            self.get_logger().error(f"request failed: {exc}")
            traceback.print_exc()
            self._show_error_overlay(str(exc))
        finally:
            self.processing_lock.release()
        return response

    @staticmethod
    def _clip_box(xyxy: np.ndarray, image: np.ndarray) -> tuple[int, int, int, int]:
        h, w = image.shape[:2]
        x1, y1, x2, y2 = np.rint(xyxy).astype(int)
        x1, x2 = np.clip([x1, x2], 0, max(0, w - 1))
        y1, y2 = np.clip([y1, y2], 0, max(0, h - 1))
        return int(x1), int(y1), int(x2), int(y2)

    def _find_best_detection(self, profile_names: list[str], timeout: float,
                             stage_name: str):
        """같은 프레임에서 여러 profile을 검사하고 최고 confidence 후보를 반환합니다."""
        if not profile_names:
            return None

        deadline = time.monotonic() + timeout
        last_attempt = 0.0
        while time.monotonic() < deadline:
            item = self._latest_frame()
            if item is None:
                time.sleep(0.01)
                continue
            if time.monotonic() - last_attempt < self.args.detect_interval:
                time.sleep(0.01)
                continue
            last_attempt = time.monotonic()

            color_bgr, depth_raw, K = item
            depth_m = depth_raw.astype(np.float32)
            if np.issubdtype(depth_raw.dtype, np.integer):
                depth_m *= self.args.depth_scale
            color_bgr, depth_m, K = resize_rgbd(
                color_bgr, depth_m, K, self.args.max_width
            )

            candidates = []
            preview = color_bgr.copy()
            for name in profile_names:
                profile = self.profiles[name]
                detections = self.detector.detect(color_bgr, profile.dino_prompt)
                for det in detections:
                    # 클래스별 confidence 제한:
                    # white_bear / otter_in_can / green_frog >= 0.35
                    # 나머지 객체 >= 0.50
                    if det.confidence < profile.min_confidence:
                        continue

                    mask = detection_mask(det, depth_m, self.args.mask_depth_band_m)
                    if int(mask.sum()) < self.args.min_mask_pixels:
                        continue
                    candidates.append((profile, det))
                    x1, y1, x2, y2 = self._clip_box(det.xyxy, preview)
                    cv2.rectangle(preview, (x1, y1), (x2, y2), (120, 120, 120), 2)
                    cv2.putText(
                        preview, f"{name} {det.confidence:.2f}",
                        (x1, max(20, y1 - 7)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, (120, 120, 120), 2,
                    )

            cv2.putText(
                preview, f"DINO stage: {stage_name}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2,
            )

            if candidates:
                profile, det = max(candidates, key=lambda item: item[1].confidence)
                x1, y1, x2, y2 = self._clip_box(det.xyxy, preview)
                cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 255, 255), 3)
                cv2.putText(
                    preview,
                    f"SELECTED: {profile.model_name} {det.confidence:.2f}",
                    (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (0, 255, 255), 2,
                )
                self._set_overlay(preview)
                return profile, det, (color_bgr, depth_m, K)

            self._set_overlay(preview)
        return None

    def _estimate_selected_detection(self, profile: ObjectProfile, det: Detection,
                                     frame_data) -> np.ndarray:
        mesh = load_mesh_in_meters(profile.mesh_path, profile.height_mm)
        debug_dir = (
            Path(self.args.any6d_root).expanduser()
            / "live_obj_debug" / profile.model_name
        )
        debug_dir.mkdir(parents=True, exist_ok=True)
        estimator = self.Any6D(
            symmetry_tfs=None, mesh=mesh, debug_dir=str(debug_dir), debug=0
        )

        color_bgr, depth_m, K = frame_data
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        mask = detection_mask(det, depth_m, self.args.mask_depth_band_m)
        if int(mask.sum()) < self.args.min_mask_pixels:
            raise RuntimeError("selected detection mask is too small")

        preview = color_bgr.copy()
        x1, y1, x2, y2 = self._clip_box(det.xyxy, preview)
        cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 255, 255), 3)
        cv2.putText(
            preview, f"Initializing Any6D: {profile.model_name}", (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2,
        )
        self._set_overlay(preview)

        pose = estimator.register(
            K=K, rgb=color_rgb, depth=depth_m, ob_mask=mask,
            iteration=self.args.register_iterations, name=profile.model_name,
        )

        deadline = time.monotonic() + self.args.pose_timeout
        tracked_frames = 0
        while time.monotonic() < deadline:
            item = self._latest_frame()
            if item is None:
                time.sleep(0.01)
                continue
            color_bgr, depth_raw, K = item
            depth_m = depth_raw.astype(np.float32)
            if np.issubdtype(depth_raw.dtype, np.integer):
                depth_m *= self.args.depth_scale
            color_bgr, depth_m, K = resize_rgbd(
                color_bgr, depth_m, K, self.args.max_width
            )
            color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

            pose = estimator.track_one_any6d(
                rgb=color_rgb, depth=depth_m, K=K,
                iteration=self.args.track_iterations,
            )
            tracked_frames += 1
            preview = draw_pose_box(color_bgr, K, pose, mesh.bounds)
            axis_length = max(0.025, min(0.08, float(np.max(mesh.extents)) * 0.65))
            preview = draw_pose_axes(preview, K, pose, axis_length)
            cv2.putText(
                preview,
                f"Any6D {profile.model_name} {tracked_frames}/{self.args.confirm_track_frames}",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                (0, 255, 255), 2,
            )
            self._set_overlay(preview)
            if tracked_frames >= self.args.confirm_track_frames:
                cv2.putText(preview, "POSE FOUND (CAMERA FRAME)", (10, 58),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0, 255, 0), 2)
                self._set_overlay(preview)
                return np.asarray(pose, dtype=np.float64).reshape(4, 4)

        raise TimeoutError(
            f"{profile.model_name} Any6D pose estimation timed out after "
            f"{self.args.pose_timeout:.1f}s"
        )


def _project_points(K: np.ndarray, pose: np.ndarray, points_obj: np.ndarray):
    points_obj = np.asarray(points_obj, dtype=np.float32).reshape(-1, 3)
    pose = np.asarray(pose, dtype=np.float32).reshape(4, 4)
    points_h = np.concatenate(
        [points_obj, np.ones((len(points_obj), 1), dtype=np.float32)], axis=1
    )
    points_cam = (pose @ points_h.T).T[:, :3]
    valid = points_cam[:, 2] > 1e-6
    pixels = np.full((len(points_obj), 2), np.nan, dtype=np.float32)
    if np.any(valid):
        projected = (K @ points_cam[valid].T).T
        pixels[valid] = projected[:, :2] / projected[:, 2:3]
    return pixels, valid


def draw_pose_box(frame: np.ndarray, K: np.ndarray, pose: np.ndarray,
                  bounds: np.ndarray) -> np.ndarray:
    out = frame.copy()
    lo, hi = np.asarray(bounds, dtype=np.float32).reshape(2, 3)
    corners = np.array([
        [lo[0], lo[1], lo[2]], [hi[0], lo[1], lo[2]],
        [hi[0], hi[1], lo[2]], [lo[0], hi[1], lo[2]],
        [lo[0], lo[1], hi[2]], [hi[0], lo[1], hi[2]],
        [hi[0], hi[1], hi[2]], [lo[0], hi[1], hi[2]],
    ], dtype=np.float32)
    pixels, valid = _project_points(K, pose, corners)
    edges = ((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7))
    for a, b in edges:
        if valid[a] and valid[b] and np.all(np.isfinite(pixels[[a, b]])):
            pa = tuple(np.rint(pixels[a]).astype(int))
            pb = tuple(np.rint(pixels[b]).astype(int))
            cv2.line(out, pa, pb, (0, 255, 255), 2, cv2.LINE_AA)
    return out


def draw_pose_axes(frame: np.ndarray, K: np.ndarray, pose: np.ndarray,
                   axis_length: float) -> np.ndarray:
    out = frame.copy()
    points = np.array([
        [0.0, 0.0, 0.0], [axis_length, 0.0, 0.0],
        [0.0, axis_length, 0.0], [0.0, 0.0, axis_length],
    ], dtype=np.float32)
    pixels, valid = _project_points(K, pose, points)
    if not valid[0] or not np.all(np.isfinite(pixels[0])):
        return out
    origin = tuple(np.rint(pixels[0]).astype(int))
    for idx, (color, label) in enumerate(
        [((0, 0, 255), "X"), ((0, 255, 0), "Y"), ((255, 0, 0), "Z")],
        start=1,
    ):
        if valid[idx] and np.all(np.isfinite(pixels[idx])):
            end = tuple(np.rint(pixels[idx]).astype(int))
            cv2.arrowedLine(out, origin, end, color, 3, cv2.LINE_AA, tipLength=0.18)
            cv2.putText(out, label, end, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return out


def detection_mask(
    det: Detection, depth_m: np.ndarray, depth_band_m: float
) -> np.ndarray:
    """DINO 박스 안에서 대표 깊이 주변 픽셀만 Any6D 마스크로 사용합니다."""
    h, w = depth_m.shape
    x1, y1, x2, y2 = np.rint(det.xyxy).astype(int)
    x1, x2 = np.clip([x1, x2], 0, w)
    y1, y2 = np.clip([y1, y2], 0, h)

    mask = np.zeros((h, w), dtype=bool)
    if x2 <= x1 or y2 <= y1:
        return mask

    roi_depth = depth_m[y1:y2, x1:x2]
    valid = (
        np.isfinite(roi_depth)
        & (roi_depth > 0.10)
        & (roi_depth < 2.0)
    )
    if not np.any(valid):
        return mask

    # 박스 중앙부의 깊이를 우선 사용해 배경이 대표값이 되는 것을 줄입니다.
    roi_h, roi_w = roi_depth.shape
    cx1, cx2 = int(roi_w * 0.25), max(int(roi_w * 0.75), 1)
    cy1, cy2 = int(roi_h * 0.25), max(int(roi_h * 0.75), 1)
    center_depth = roi_depth[cy1:cy2, cx1:cx2]
    center_valid = valid[cy1:cy2, cx1:cx2]
    reference_values = center_depth[center_valid]
    if reference_values.size == 0:
        reference_values = roi_depth[valid]

    reference_depth = float(np.median(reference_values))
    foreground = valid & (np.abs(roi_depth - reference_depth) <= depth_band_m)
    mask[y1:y2, x1:x2] = foreground
    return mask


def load_mesh_in_meters(mesh_path: Path, height_mm: float) -> trimesh.Trimesh:
    """OBJ를 미터 단위로 읽고 FoundationPose가 요구하는 texture+UV를 보장합니다.

    일부 OBJ는 PNG/MTL 텍스처가 없고 vertex color만 있거나 색상 자체가 없습니다.
    현재 FoundationPose Utils.py는 material.image가 None이면 convert()에서 죽으므로,
    그런 경우 중성 2x2 RGB 텍스처와 UV를 자동으로 붙입니다.
    """
    loaded = trimesh.load(mesh_path, force="mesh", process=False)
    if not isinstance(loaded, trimesh.Trimesh) or loaded.is_empty:
        raise ValueError(f"invalid mesh: {mesh_path}")

    mesh = loaded.copy()
    visual = getattr(mesh, "visual", None)
    material = getattr(visual, "material", None)
    texture_image = getattr(material, "image", None)
    uv = getattr(visual, "uv", None)

    valid_uv = (
        uv is not None
        and np.asarray(uv).ndim == 2
        and np.asarray(uv).shape == (len(mesh.vertices), 2)
        and np.all(np.isfinite(np.asarray(uv)))
    )

    if texture_image is not None and valid_uv:
        print(f"[Mesh] PNG/MTL texture 사용: {mesh_path.name}")
    else:
        vertex_colors = getattr(visual, "vertex_colors", None)
        has_vertex_colors = (
            vertex_colors is not None
            and np.asarray(vertex_colors).ndim == 2
            and len(vertex_colors) == len(mesh.vertices)
            and np.asarray(vertex_colors).shape[1] >= 3
        )

        if has_vertex_colors:
            rgb = np.asarray(vertex_colors, dtype=np.uint8)[:, :3]
            # 렌더러 입력 보장을 위한 대표 색상. 포즈 계산의 핵심은 mesh geometry입니다.
            representative = np.median(rgb, axis=0).astype(np.uint8)
            print(
                f"[Mesh] vertex color 감지({len(rgb)} vertices) -> "
                f"대표색 texture 자동 생성: {representative.tolist()}"
            )
        else:
            representative = np.array([180, 180, 180], dtype=np.uint8)
            print("[Mesh] texture/vertex color 없음 -> 중성 texture 자동 생성")

        texture_array = np.tile(representative.reshape(1, 1, 3), (2, 2, 1))
        texture = PILImage.fromarray(texture_array, mode="RGB")

        # 모든 정점을 텍스처 중앙에 매핑합니다. None texture 오류를 완전히 차단합니다.
        generated_uv = np.full((len(mesh.vertices), 2), 0.5, dtype=np.float64)
        generated_material = trimesh.visual.texture.SimpleMaterial(image=texture)
        mesh.visual = trimesh.visual.texture.TextureVisuals(
            uv=generated_uv,
            image=texture,
            material=generated_material,
        )

    z_extent = float(mesh.extents[2])
    if z_extent <= 0 or height_mm <= 0:
        raise ValueError("mesh Z extent and height_mm must be positive")
    mesh.apply_scale((height_mm / 1000.0) / z_extent)
    return mesh


def resize_rgbd(color_bgr, depth_m, K, max_width):
    h, w = color_bgr.shape[:2]
    if max_width <= 0 or w <= max_width:
        return color_bgr, depth_m, K.astype(np.float32).copy()
    scale = max_width / float(w)
    new_size = (int(round(w * scale)), int(round(h * scale)))
    color = cv2.resize(color_bgr, new_size, interpolation=cv2.INTER_AREA)
    depth = cv2.resize(depth_m, new_size, interpolation=cv2.INTER_NEAREST)
    K2 = K.astype(np.float32).copy()
    K2[0, 0] *= scale
    K2[1, 1] *= scale
    K2[0, 2] *= scale
    K2[1, 2] *= scale
    return color, depth, K2


def build_profiles(args) -> dict[str, ObjectProfile]:
    low_confidence = 0.35
    default_confidence = 0.50

    return {
        "yellow_can": ObjectProfile("yellow_can", "노란색 원통형 캔", "yellow cylindrical can",
            Path(args.yellow_can_mesh).expanduser(), args.yellow_can_height_mm, default_confidence),
        "green_box": ObjectProfile("green_box", "초록색 직육면체 상자", "green rectangular box",
            Path(args.green_box_mesh).expanduser(), args.green_box_height_mm, default_confidence),
        "gray_box": ObjectProfile("gray_box", "회색 수납장/회색 박스", "gray storage box",
            Path(args.gray_box_mesh).expanduser(), args.gray_box_height_mm, default_confidence),
        "white_bear": ObjectProfile("white_bear", "흰색 곰 인형", "white bear plush toy",
            Path(args.white_bear_mesh).expanduser(), args.white_bear_height_mm, low_confidence),
        "aircon_remote": ObjectProfile("aircon_remote", "회색 에어컨 리모컨", "gray air conditioner remote control",
            Path(args.aircon_remote_mesh).expanduser(), args.aircon_remote_height_mm, default_confidence),
        "green_frog": ObjectProfile("green_frog", "초록 개구리 인형", "green frog plush toy",
            Path(args.green_frog_mesh).expanduser(), args.green_frog_height_mm, low_confidence),
        "otter_in_can": ObjectProfile("otter_in_can", "통 안에 있는 수달 인형", "otter in a wooden barrel",
            Path(args.otter_in_can_mesh).expanduser(), args.otter_in_can_height_mm, low_confidence),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="DINO + Any6D DetectObject service server")
    parser.add_argument("--service", default="/find_object_pose")
    parser.add_argument(
        "--picked-service",
        default="/set_picked_object",
        help="all_detection 노드가 제공하는 SetPickedObject 서비스명",
    )
    parser.add_argument(
        "--picked-service-wait-sec",
        type=float,
        default=3.0,
        help="호환성 유지용 옵션(현재 비동기 알림에서는 사용하지 않음)",
    )
    parser.add_argument(
        "--picked-response-timeout-sec",
        type=float,
        default=30.0,
        help="호환성 유지용 옵션(현재 비동기 알림에서는 사용하지 않음)",
    )
    parser.add_argument("--any6d-root", default="~/Any6D")
    parser.add_argument("--output", default=None,
                        help="결과 4x4 행렬 저장 경로. 기본: base 모드는 pose_base.npy, camera-only는 pose_camera.npy")
    parser.add_argument("--yellow-can-mesh", default="~/Any6D/anchors/object_can/my_object_can_mesh_raw.obj")
    parser.add_argument("--yellow-can-height-mm", type=float, default=120.0)
    parser.add_argument("--green-box-mesh", default="~/Any6D/green_box.obj")
    parser.add_argument("--green-box-height-mm", type=float, default=70.0)
    parser.add_argument("--gray-box-mesh", default="~/Any6D/anchors/gray_box/gray_box.obj")
    parser.add_argument("--gray-box-height-mm", type=float, default=80.0)
    parser.add_argument("--white-bear-mesh", default="~/Any6D/anchors/files/bear_fixed2.obj")
    parser.add_argument("--white-bear-height-mm", type=float, default=90.0)
    parser.add_argument("--aircon-remote-mesh", default="~/Any6D/anchors/remote/remote.obj")
    parser.add_argument("--aircon-remote-height-mm", type=float, default=30.0)
    parser.add_argument("--green-frog-mesh", default="~/Any6D/anchors/frog/frog_fixed.obj")
    parser.add_argument("--green-frog-height-mm", type=float, default=95.0)
    parser.add_argument("--otter-in-can-mesh", default="~/Any6D/anchors/otter/otter.obj")
    parser.add_argument("--otter-in-can-height-mm", type=float, default=85.0)
    parser.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--info-topic", default="/camera/camera/aligned_depth_to_color/camera_info")
    parser.add_argument("--sync-tolerance-ms", type=float, default=80.0)
    parser.add_argument("--depth-scale", type=float, default=0.001)
    parser.add_argument("--max-width", type=int, default=640)
    parser.add_argument("--primary-detect-timeout", type=float, default=4.0,
                        help="기본 대상 검출 대기 시간(초)")
    parser.add_argument("--fallback-detect-timeout", type=float, default=4.0,
                        help="초록/회색 박스 fallback 검출 대기 시간(초)")
    parser.add_argument("--pose-timeout", type=float, default=12.0,
                        help="선택된 물체의 Any6D 추정 제한 시간(초)")
    parser.add_argument("--detect-interval", type=float, default=0.8)
    parser.add_argument("--min-mask-pixels", type=int, default=100)
    parser.add_argument(
        "--mask-depth-band-m", type=float, default=0.12,
        help="DINO 박스 내부 대표 깊이에서 허용할 거리 범위(m). 기본 0.12",
    )
    parser.add_argument("--register-iterations", type=int, default=3)
    parser.add_argument("--track-iterations", type=int, default=2)
    parser.add_argument("--confirm-track-frames", type=int, default=2)
    parser.add_argument("--result-hold-seconds", type=float, default=2.5,
                        help="완료된 DINO/Any6D 표시를 유지할 시간(초)")
    parser.add_argument("--window-title", default="DINO + Any6D service")
    dino_home = Path(os.getenv("GROUNDINGDINO_HOME", "~/GroundingDINO")).expanduser()
    parser.add_argument("--config", default=str(dino_home / "groundingdino/config/GroundingDINO_SwinT_OGC.py"))
    parser.add_argument("--weights", default=str(dino_home / "weights/groundingdino_swint_ogc.pth"))
    parser.add_argument("--device", default="0")
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.55)
    parser.add_argument("--imgsz", type=int, default=800)
    args, unknown = parser.parse_known_args()

    root = Path(args.any6d_root).expanduser().resolve()
    profiles = build_profiles(args)
    missing = [str(p.mesh_path) for p in profiles.values() if not p.mesh_path.is_file()]
    if missing:
        raise SystemExit("missing mesh files:\n- " + "\n- ".join(missing))
    if args.output is None:
        args.output = "~/Any6D/pose_camera.npy"

    device = "cpu" if args.device.lower() == "cpu" else (
        f"cuda:{args.device}" if args.device.isdigit() else args.device
    )
    if device.startswith("cuda") and not torch.cuda.is_available():
        device = "cpu"
    detector = GroundingDinoDetector(
        args.config, args.weights, device, args.imgsz,
        args.conf, args.text_threshold, args.iou,
    )

    os.chdir(root)
    sys.path.insert(0, str(root))
    from estimater import Any6D

    rclpy.init()
    node = FindObjectServer(args, profiles, detector, Any6D)
    node.get_logger().info("pose output mode: camera frame (robot connection not required)")
    node.get_logger().info(
        "class confidence thresholds: "
        + ", ".join(
            f"{name}>={profile.min_confidence:.2f}"
            for name, profile in profiles.items()
        )
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_display()
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
