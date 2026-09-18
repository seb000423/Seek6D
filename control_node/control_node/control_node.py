"""Top-level ROS 2 control node and task orchestrator."""

from __future__ import annotations

import time
from typing import Any, Optional

import rclpy
from rclpy.node import Node

from .config import AppConfig, DEFAULT_CONFIG
from .db_client import DBClient
from .detection_client import DetectionClient
from .models import (
    DBLookupError,
    GripperError,
    MoveItExecutionError,
    MoveItPlanningError,
    PoseValidationError,
    RobotTask,
    TargetAcquisition,
    TargetPose,
    TaskOutcome,
)
from .motion_executor import MotionExecutor
from .pose_provider import PoseProvider, create_pose_provider
from .search_planner import db_position_mm
from .state_interface import StateInterface


class RobotControlNode(Node):
    def __init__(self, config: AppConfig = DEFAULT_CONFIG) -> None:
        """
        [초기화 함수]
        ROS 2 노드를 생성하고 로봇 제어에 필요한 기본 설정들을 초기화합니다.
        - MoveIt 모션 제어기와 RG2 실행 계층을 생성합니다.
        - Any6D 포즈(자세) 제공자, DB 통신 클라이언트, 그리고 상태(State) 관리 인터페이스를 설정합니다.
        """
        super().__init__(
            "robot_control_any6d",
            namespace=config.robot.robot_id,
        )
        self.config = config
        self.motion: Optional[MotionExecutor] = MotionExecutor(
            self,
            config.robot,
            config.moveit,
            config.motion,
            config.gripper,
            config.search,
            config.recenter,
            config.pose,
        )
        self.pose_provider: PoseProvider = create_pose_provider(
            self,
            config.pose,
            enable_motion=config.robot.enable_motion,
        )
        self.db = DBClient(self, config.interface)
        self.detector = DetectionClient(
            self,
            config.search,
            config.recenter,
        )
        self.state = StateInterface(
            self,
            config.interface,
            supported_object_names=config.search.supported_object_names,
            status_supplier=self._status_payload,
            acceptance_guard=self._acceptance_guard,
        )

    def initialize_hardware(self) -> None:
        """
        [하드웨어 초기화 함수]
        생성된 모션 제어기를 통해 로봇 하드웨어를 설정(configure)하고 초기화(initialize)합니다.
        초기화가 끝나면 로봇 상태를 '준비 완료(Ready)'로 변경하고 관련 토픽 정보를 로그로 남깁니다.
        """
        if self.motion is None:
            raise RuntimeError("MoveIt motion backend is not initialized")

        self.motion.configure()
        self.motion.initialize()
        self.state.mark_ready()
        interface = self.config.interface
        self.get_logger().info(
            f"Control ready: init={interface.control_init_service}, "
            f"request={interface.control_task_service}, "
            f"search_action={interface.control_search_action}, "
            f"result={interface.state_result_service}"
        )

    def _status_payload(self) -> dict[str, Any]:
        """
        [상태 정보 제공 헬퍼 함수]
        현재 로봇의 상태 정보(모션 활성화 여부, 동작 중인지 여부, 물체를 쥐고 있는지 여부 등)를
        딕셔너리 형태로 묶어서 반환합니다. 외부에서 로봇 상태를 요청할 때 사용됩니다.
        """
        return {
            "motion_enabled": self.config.robot.enable_motion,
            "robot_busy": self.motion.busy if self.motion else False,
            "holding_object": (
                self.motion.holding_object if self.motion else False
            ),
            "green_box_lid_restore_pending": bool(
                self.motion and self.motion.green_box_lid_route is not None
            ),
            "gray_box_drawer_restore_pending": bool(
                self.motion and self.motion.gray_box_drawer_route is not None
            ),
            "target_input_mode": self.config.pose.input_mode,
            "db_service": self.config.interface.db_load_service,
            "detection_service": self.config.search.detection_service,
            "motion_backend": "moveit2",
            "planning_group": self.config.moveit.planning_group,
            "planning_frame": self.config.moveit.base_frame,
            "eef_link": self.config.moveit.eef_link,
        }

    def _acceptance_guard(self) -> Optional[str]:
        """
        [작업 수락 검사 헬퍼 함수]
        새로운 작업을 수락해도 되는 상태인지 검사합니다.
        작업을 받을 수 없는 경우(하드웨어 미연결, 상자 복구 미완료)에는 거절 사유(문자열)를 반환하고,
        작업을 받을 수 있다면 None을 반환합니다.
        """
        if self.motion is None:
            return "로봇 하드웨어가 초기화되지 않았습니다"
        if self.motion.green_box_lid_route is not None:
            return "green_box 덮개 복구가 완료되지 않아 새 작업을 받을 수 없습니다"
        if self.motion.gray_box_drawer_route is not None:
            return "gray_box 서랍 닫기가 완료되지 않아 새 작업을 받을 수 없습니다"
        return None

    def execute_task(self, task: RobotTask) -> None:
        """
        [작업 실행 메인 로직 함수]
        단일 로봇 작업(Task)을 전체 흐름에 따라 실행합니다.
        1. DB 조회: 찾고자 하는 물체의 마지막 위치를 DB에서 불러옵니다.
        2. 포즈(Any6D) 대기: 카메라나 비전 시스템으로부터 타겟 물체의 정확한 위치(Pose)를 기다립니다.
        3. 로봇 동작: 전달받은 포즈를 향해 로봇을 움직여 물체를 집고(Pick), 다시 홈 위치로 복귀(Return home)합니다.
        4. 상태 업데이트: 작업 도중 또는 완료 시 성공/실패 여부를 상태 인터페이스를 통해 지속적으로 발행(Publish)합니다.
        """
        db_result = None
        self.state.publish_event(
            task,
            status="running",
            success=True,
            outcome=TaskOutcome.DB_LOOKUP,
            message="DB에서 물체의 마지막 위치를 조회합니다",
        )

        try:
            if self.motion is None:
                raise RuntimeError("로봇 하드웨어가 초기화되지 않았습니다")

            # DB 조회 단계
            db_result = self.db.lookup(task)
            db_payload = db_result.to_payload()
            db_position = db_position_mm(db_result.item)
            acquisition = None
            if db_position is not None:
                acquisition = self._search_db_location(
                    task,
                    db_position,
                    db_payload,
                )
            else:
                self.get_logger().warning(
                    f"Task {task.task_id}: no valid DB XYZ; "
                    "starting the standard search-zone route"
                )
            if acquisition is None:
                acquisition = self._search_unknown_object(task, db_payload)
            not_found_message = (
                "모든 탐색 구역에서 물체를 찾지 못했거나 "
                "유효한 Any6D 자세를 받지 못했습니다"
            )
            if acquisition is None:  # 시간 내에 비전 정보를 받지 못한 경우
                self.state.publish_event(
                    task,
                    status="completed",
                    success=False,
                    outcome=TaskOutcome.NOT_FOUND,
                    message=not_found_message,
                    extra={"db": db_payload},
                )
                return

            # 실제 로봇 구동(Pick) 및 홈 복귀 단계
            success = self.motion.pick_and_return_home(
                acquisition.target,
                object_name=task.class_label or task.name,
            )
            if not success:  # 드라이런(가상 테스트) 모드 등 실제 동작을 안한 경우
                self.state.publish_event(
                    task,
                    status="completed",
                    success=False,
                    outcome=TaskOutcome.DRY_RUN,
                    message="드라이런 모드이므로 로봇 동작을 실행하지 않았습니다",
                    extra={"db": db_payload},
                )
                return

            if acquisition.close_green_box_after_delivery:
                self.state.publish_event(
                    task,
                    status="running",
                    success=True,
                    outcome=TaskOutcome.GREEN_BOX_CLEANUP,
                    message=(
                        "사용자 전달을 완료했습니다. 2초 후 green_box 덮개를 "
                        "원래 위치에 복구합니다"
                    ),
                    extra={"db": db_payload, "source": acquisition.source},
                )
                self._wait_with_spin(
                    self.config.search.green_box_close_delay_sec
                )
                if not self.motion.close_green_box():
                    raise RuntimeError(
                        "사용자 전달은 완료했지만 green_box 덮개 복구에 "
                        "실패했습니다"
                    )
                self.motion.move_home()

            if acquisition.close_gray_box_after_delivery:
                self.state.publish_event(
                    task,
                    status="running",
                    success=True,
                    outcome=TaskOutcome.GRAY_BOX_CLEANUP,
                    message="사용자 전달을 완료했습니다. gray_box 서랍을 닫습니다",
                    extra={"db": db_payload, "source": acquisition.source},
                )
                if not self.motion.close_gray_box():
                    raise RuntimeError(
                        "사용자 전달은 완료했지만 gray_box 서랍 닫기에 실패했습니다"
                    )

            # 성공적으로 작업을 마쳤을 경우
            if acquisition.close_green_box_after_delivery:
                completion_message = "물체 전달, green_box 덮개 원위치 배치 및 홈 복귀를 완료했습니다"
            elif acquisition.close_gray_box_after_delivery:
                completion_message = "물체 전달, gray_box 서랍 닫기 및 홈 복귀를 완료했습니다"
            else:
                completion_message = "물체 파지, 사용자 전달 및 홈 복귀를 완료했습니다"
            self.state.publish_event(
                task,
                status="completed",
                success=True,
                outcome=TaskOutcome.PICK_COMPLETED,
                message=completion_message,
                extra={
                    "db": db_payload,
                    "pose_sequence": acquisition.target.source_sequence,
                    "source": acquisition.source,
                },
            )

        except (
            DBLookupError,
            PoseValidationError,
            GripperError,
            RuntimeError,
        ) as error:
            self.get_logger().error(f"Task {task.task_id} failed: {error}")
            recovered, recovery_message = self._recover_after_failure()
            extra = {
                "recovery_success": recovered,
                "recovery": recovery_message,
            }
            if db_result:
                extra["db"] = db_result.to_payload()
            self.state.publish_event(
                task,
                status="completed",
                success=False,
                outcome=TaskOutcome.FAILED,
                message=f"{error}; {recovery_message}",
                extra=extra,
            )
        except Exception as error:
            self.get_logger().error(
                f"Task {task.task_id} unexpected failure: {error}"
            )
            recovered, recovery_message = self._recover_after_failure()
            self.state.publish_event(
                task,
                status="completed",
                success=False,
                outcome=TaskOutcome.FAILED,
                message=f"예상하지 못한 오류: {error}; {recovery_message}",
                extra={
                    "recovery_success": recovered,
                    "recovery": recovery_message,
                },
            )
        finally:
            # 성공/실패 여부와 상관없이 작업을 종료 상태로 처리
            self.state.finish_task(task)
            if self.motion and (
                self.motion.green_box_lid_route is not None
                or self.motion.gray_box_drawer_route is not None
            ):
                self.get_logger().error(
                    "box restoration is pending; new task execution is blocked"
                )
            else:
                self.get_logger().info("Waiting for the next state-node task")

    def _recover_after_failure(self) -> tuple[bool, str]:
        """Return home after planning, IK, execution, or task failures."""
        if self.motion is None:
            return False, "motion backend unavailable; home recovery skipped"
        recovered, message = self.motion.recover_home()
        log = self.get_logger().info if recovered else self.get_logger().error
        log(message)
        return recovered, message

    def _wait_with_spin(self, duration_sec: float) -> None:
        """Wait without starving ROS callbacks during delayed cleanup."""
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self,
                timeout_sec=min(0.05, max(0.0, remaining)),
            )

    def _search_unknown_object(
        self,
        task: RobotTask,
        db_payload: dict[str, Any],
    ) -> Optional[TargetAcquisition]:
        """Search all configured viewpoints and return the first target."""
        if self.motion is None:
            raise RuntimeError("로봇 하드웨어가 초기화되지 않았습니다")

        green_box_opened = False
        gray_box_opened = False
        zone_count = len(self.config.search.search_zone_joints_deg)
        for zone in range(1, zone_count + 1):
            self.state.publish_event(
                task,
                status="running",
                success=True,
                outcome=TaskOutcome.SEARCHING,
                message=f"탐색 {zone}구역으로 이동합니다",
                extra={"db": db_payload, "search_zone": zone},
            )
            self.motion.move_to_search_zone(zone)
            target = self._request_target_detection(task, zone)
            if target is None:
                self.state.publish_event(
                    task,
                    status="running",
                    success=True,
                    outcome=TaskOutcome.ZONE_NOT_FOUND,
                    message=f"탐색 {zone}구역에서 물체를 찾지 못했습니다",
                    extra={"db": db_payload, "search_zone": zone},
                )
            else:
                self.state.set_task_location(task, f"search_zone_{zone}")
                return TargetAcquisition(
                    target=target,
                    source=f"search_zone_{zone}",
                )

            target_after_open, green_opened_now, gray_opened_now = (
                self._observe_landmark(
                    task,
                    zone,
                    db_payload,
                    search_green=not green_box_opened,
                    search_gray=not gray_box_opened,
                )
            )
            green_box_opened = green_box_opened or green_opened_now
            gray_box_opened = gray_box_opened or gray_opened_now
            if target_after_open is not None:
                box_name = "gray_box" if gray_opened_now else "green_box"
                self.state.set_task_location(task, f"{box_name}_zone_{zone}")
                return target_after_open

        self.motion.move_home()
        return None

    def _search_db_location(
        self,
        task: RobotTask,
        db_position: tuple[float, float, float],
        db_payload: dict[str, Any],
    ) -> Optional[TargetAcquisition]:
        """Try the DB camera viewpoint once, then let zones 1..N take over."""
        if self.motion is None:
            raise RuntimeError("로봇 하드웨어가 초기화되지 않았습니다")

        self.state.publish_event(
            task,
            status="running",
            success=True,
            outcome=TaskOutcome.SEARCHING,
            message="DB 저장 위치의 카메라 관측 자세로 이동합니다",
            extra={"db": db_payload, "search_zone": 0},
        )
        try:
            self.motion.move_to_db_search_location(db_position)
            target = self._request_target_detection(task, 0)
        except (
            MoveItPlanningError,
            MoveItExecutionError,
            PoseValidationError,
            ValueError,
        ) as error:
            self.get_logger().warning(
                f"Task {task.task_id}: DB 위치 우선 탐색 실패: {error}; "
                "1~6구역 탐색으로 전환합니다"
            )
            self.state.publish_event(
                task,
                status="running",
                success=True,
                outcome=TaskOutcome.ZONE_NOT_FOUND,
                message=(
                    "DB 위치로 이동할 수 없어 1~6구역 탐색으로 전환합니다: "
                    f"{error}"
                ),
                extra={"db": db_payload, "search_zone": 0},
            )
            return None

        if target is not None:
            self.state.set_task_location(task, "db_location")
            return TargetAcquisition(target=target, source="db_location")

        self.state.publish_event(
            task,
            status="running",
            success=True,
            outcome=TaskOutcome.ZONE_NOT_FOUND,
            message=(
                "DB 저장 위치에서 물체를 찾지 못해 "
                "1~6구역 탐색을 시작합니다"
            ),
            extra={"db": db_payload, "search_zone": 0},
        )
        return None

    def _request_target_detection(
        self,
        task: RobotTask,
        zone: int,
    ) -> Optional[TargetPose]:
        return self._request_detection_with_recenter(task, zone)

    def _request_detection_with_recenter(
        self,
        task: RobotTask,
        zone: int,
        *,
        request_kind: str = "target",
        candidates: tuple[tuple[str, str], ...] = (),
        object_local_offset_mm: Optional[tuple[float, float, float]] = None,
    ) -> Optional[TargetPose]:
        """Recenter from detector offsets and retry on a fresh RGB-D frame."""
        if self.motion is None:
            raise RuntimeError("Motion executor is not initialized")
        expected_labels = {
            label.strip().casefold()
            for _, label in candidates
            if label.strip()
        }
        if not expected_labels:
            expected_labels = {(task.class_label or task.name).strip().casefold()}

        config = self.config.recenter
        for attempt in range(config.max_attempts + 1):
            base_tcp_posx = self.motion.current_tcp_posx()
            result = self.detector.request_detection(
                task,
                zone,
                self.config.search.detection_timeout_sec,
                request_kind=request_kind,
                candidates=candidates,
                base_tcp_posx=base_tcp_posx,
                attempt=attempt,
            )
            if result.result == "not_detected":
                return None

            detected_label = result.detected_class_label.strip().casefold()
            if detected_label not in expected_labels:
                self.get_logger().warning(
                    f"Ignoring detector class '{detected_label or '<empty>'}' "
                    f"for request_id={result.request_id}; expected one of "
                    f"{sorted(expected_labels)}"
                )
                return None

            if result.result == "detected":
                if result.pose is None:
                    self.get_logger().warning(
                        f"Zone {zone}: detector reported found without a pose"
                    )
                    return None
                self.get_logger().info(
                    "Converting fresh Any6D camera pose with current TCP: "
                    f"{base_tcp_posx}"
                )
                return self.pose_provider.target_from_pose(
                    result.pose,
                    self.detector.sequence,
                    base_tcp_posx,
                    object_local_offset_mm=object_local_offset_mm,
                )

            if result.result != "recenter_required" or result.recenter is None:
                self.get_logger().warning(
                    f"Invalid recenter result for request_id={result.request_id}"
                )
                return None
            if not config.enabled:
                self.get_logger().warning(
                    "Detector requested recentring, but recenter.enabled=false"
                )
                return None
            if attempt >= config.max_attempts:
                self.get_logger().warning(
                    f"Recenter attempts exhausted: request_id={result.request_id}"
                )
                return None

            target_tcp, delta_base_mm = self.pose_provider.recenter_target(
                base_tcp_posx,
                result.recenter.offset_camera_m,
                result.recenter.frame_id,
            )
            self.get_logger().info(
                f"Recenter {attempt + 1}/{config.max_attempts}: "
                f"request_id={result.request_id}, "
                f"camera_offset_m={result.recenter.offset_camera_m}, "
                f"base_delta_mm={delta_base_mm.tolist()}"
            )
            self.motion.recenter_camera(target_tcp)
            time.sleep(config.settle_sec)

        return None

    def _observe_landmark(
        self,
        task: RobotTask,
        zone: int,
        db_payload: dict[str, Any],
        *,
        search_green: bool,
        search_gray: bool,
    ) -> tuple[Optional[TargetAcquisition], bool, bool]:
        """Open a detected green box, retry the target, then check gray box."""
        if self.motion is None:
            raise RuntimeError("Motion executor is not initialized")
        green_box_opened = False
        if search_green:
            self.state.publish_event(
                task,
                status="running",
                success=True,
                outcome=TaskOutcome.LANDMARK_SEARCHING,
                message="green_box를 탐지합니다",
                extra={
                    "db": db_payload,
                    "search_zone": zone,
                    "landmark_candidates": ["green_box"],
                },
            )
            green_target = self._request_detection_with_recenter(
                task,
                zone,
                request_kind="landmark",
                candidates=(("green_box", "green_box"),),
                object_local_offset_mm=(
                    self.config.search.green_box_handle_local_xyz_mm
                ),
            )
            if green_target is not None:
                self.state.publish_event(
                    task,
                    status="running",
                    success=True,
                    outcome=TaskOutcome.LANDMARK_FOUND,
                    message="green_box 덮개를 엽니다",
                    extra={"db": db_payload, "search_zone": zone},
                )
                green_box_opened = self.motion.open_green_box(green_target)
                if green_box_opened:
                    self.state.publish_event(
                        task,
                        status="running",
                        success=True,
                        outcome=TaskOutcome.WAITING_POSE,
                        message="green_box를 연 뒤 요청 물체를 다시 탐지합니다",
                        extra={"db": db_payload, "search_zone": zone},
                    )
                    target = self._request_target_detection(task, zone)
                    if target is not None:
                        return TargetAcquisition(
                            target=target,
                            source=f"green_box_zone_{zone}",
                            close_green_box_after_delivery=True,
                        ), True, False
                    self.state.publish_event(
                        task,
                        status="running",
                        success=True,
                        outcome=TaskOutcome.LANDMARK_NOT_FOUND,
                        message=(
                            "green_box 내부에서 요청 물체를 찾지 못해 "
                            "덮개를 다시 닫습니다"
                        ),
                        extra={"db": db_payload, "search_zone": zone},
                    )
                    if not self.motion.close_green_box():
                        raise RuntimeError("green_box 덮개 닫기에 실패했습니다")
                    green_box_opened = False

        if not search_gray:
            return None, green_box_opened, False

        self.state.publish_event(
            task,
            status="running",
            success=True,
            outcome=TaskOutcome.LANDMARK_SEARCHING,
            message="요청 물체가 없어 gray_box를 탐지합니다",
            extra={
                "db": db_payload,
                "search_zone": zone,
                "landmark_candidates": ["gray_box"],
            },
        )
        gray_target = self._request_detection_with_recenter(
            task,
            zone,
            request_kind="landmark",
            candidates=(("gray_box", "gray_box"),),
        )
        gray_box_opened = False
        gray_box_inspected = False
        if gray_target is not None:
            self.state.publish_event(
                task,
                status="running",
                success=True,
                outcome=TaskOutcome.LANDMARK_FOUND,
                message="gray_box 손잡이를 잡고 상자를 엽니다",
                extra={"db": db_payload, "search_zone": zone},
            )
            gray_box_opened = self.motion.open_gray_box(gray_target)
            if gray_box_opened:
                gray_box_inspected = True
                self.state.publish_event(
                    task,
                    status="running",
                    success=True,
                    outcome=TaskOutcome.WAITING_POSE,
                    message="gray_box를 연 뒤 요청 물체를 다시 탐지합니다",
                    extra={"db": db_payload, "search_zone": zone},
                )
                target = self._request_target_detection(task, zone)
                if target is not None:
                    return TargetAcquisition(
                        target=target,
                        source=f"gray_box_zone_{zone}",
                        close_gray_box_after_delivery=True,
                    ), green_box_opened, True

                self.state.publish_event(
                    task,
                    status="running",
                    success=True,
                    outcome=TaskOutcome.GRAY_BOX_CLEANUP,
                    message="gray_box 내부에서 요청 물체를 찾지 못해 서랍을 닫습니다",
                    extra={"db": db_payload, "search_zone": zone},
                )
                if not self.motion.close_gray_box():
                    raise RuntimeError("gray_box 서랍 닫기에 실패했습니다")
                gray_box_opened = False

        message = (
            "gray_box 내부에서 요청 물체를 찾지 못해 다음 탐색구역으로 이동합니다"
            if gray_box_inspected
            else "gray_box를 찾지 못했거나 유효한 pose가 없습니다"
        )
        self.state.publish_event(
            task,
            status="running",
            success=True,
            outcome=TaskOutcome.LANDMARK_NOT_FOUND,
            message=message,
            extra={"db": db_payload, "search_zone": zone},
        )
        return None, green_box_opened, gray_box_inspected

    def run(self) -> None:
        """
        [메인 실행 루프 함수]
        노드가 살아있는 동안 지속적으로 반복 실행되는 루프입니다.
        1. rclpy.spin_once를 통해 ROS 2 이벤트 및 콜백을 처리합니다.
        2. 상태 인터페이스로부터 대기 중인 다음 작업(Task)이 있는지 확인하고 가져옵니다.
        3. 새 작업이 있다면 execute_task 함수를 호출해 실행합니다.
        """
        interface = self.config.interface
        self.get_logger().info(
            f"Waiting for state-node tasks on {interface.control_task_service} "
            f"or {interface.control_search_action}"
        )
        if self.config.pose.input_mode == "any6d":
            self.get_logger().info(
                f"Any6D detection service: "
                f"{self.config.search.detection_service}"
            )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.motion and (
                self.motion.green_box_lid_route is not None
                or self.motion.gray_box_drawer_route is not None
            ):
                # A failed box restoration must not allow an already queued
                # task to start. New requests are rejected by the guard too.
                continue
            task = self.state.take_next_task()
            if task is not None:
                self.execute_task(task)

    def shutdown_hardware(self) -> None:
        """
        [하드웨어 종료 함수]
        노드가 종료되거나 시스템이 멈출 때 호출되며,
        로봇 하드웨어 및 모션 제어기를 안전하게 종료(Shutdown)합니다.
        """
        if self.motion is not None:
            self.motion.shutdown()
