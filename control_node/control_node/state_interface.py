"""Service/action interface between the state node and robot controller."""

from __future__ import annotations

import json
import threading
from collections import deque
from typing import Any, Callable, Optional

from interfaces.action import Search
from interfaces.srv import ControlTask, NodeInit, RobotResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.task import Future

from .config import InterfaceConfig
from .models import RobotTask, TaskOutcome


StatusSupplier = Callable[[], dict[str, Any]]
AcceptanceGuard = Callable[[], Optional[str]]


class StateInterface:
    """Receive service/action tasks and publish their lifecycle."""

    def __init__(
        self,
        node: Node,
        config: InterfaceConfig,
        *,
        supported_object_names: tuple[str, ...],
        status_supplier: StatusSupplier,
        acceptance_guard: AcceptanceGuard,
    ) -> None:
        self.node = node
        self.config = config
        self.supported_object_names = supported_object_names
        self.status_supplier = status_supplier
        self.acceptance_guard = acceptance_guard
        self._ready = False
        self._lock = threading.Lock()
        self._pending: deque[RobotTask] = deque()
        self._active: Optional[RobotTask] = None
        self._known_task_ids: set[str] = set()
        self._result_futures: set[Any] = set()
        self._action_handles: dict[str, Any] = {}
        self._action_futures: dict[str, Future] = {}
        self._action_locations: dict[str, str] = {}

        self.init_service = node.create_service(
            NodeInit,
            config.control_init_service,
            self._on_init_request,
        )
        self.task_service = node.create_service(
            ControlTask,
            config.control_task_service,
            self._on_task_request,
        )
        self.search_action = ActionServer(
            node,
            Search,
            config.control_search_action,
            execute_callback=self._execute_search,
            goal_callback=self._on_search_goal,
            cancel_callback=self._on_search_cancel,
            handle_accepted_callback=self._on_search_accepted,
        )
        self.result_client = node.create_client(
            RobotResult,
            config.state_result_service,
        )

    @property
    def ready(self) -> bool:
        return self._ready

    def mark_ready(self) -> None:
        self._ready = True

    def _queue_status(self) -> tuple[Optional[str], int]:
        with self._lock:
            active_id = self._active.task_id if self._active else None
            return active_id, len(self._pending)

    def _on_init_request(self, request, response):
        try:
            payload = json.loads(request.request) if request.request.strip() else {}
            if not isinstance(payload, dict):
                raise ValueError("request JSON must be an object")
        except (json.JSONDecodeError, ValueError) as error:
            response.success = False
            response.response = "{}"
            response.message = f"초기화 요청 JSON 오류: {error}"
            return response

        active_id, pending_count = self._queue_status()
        status = self.status_supplier()
        status.update(
            {
                "ready": self._ready,
                "node": self.node.get_fully_qualified_name(),
                "active_task_id": active_id,
                "pending_tasks": pending_count,
            }
        )
        requester = str(payload.get("node", "state_node"))
        response.success = self._ready
        response.response = json.dumps(status, ensure_ascii=False)
        response.message = (
            f"제어 노드 준비 완료 / 요청자={requester}"
            if self._ready
            else f"제어 노드 초기화 중 / 요청자={requester}"
        )
        return response

    def _parse_task(self, raw_json: str) -> RobotTask:
        try:
            payload = json.loads(raw_json)
        except json.JSONDecodeError as error:
            raise ValueError(f"JSON 파싱 실패: {error}") from error
        if not isinstance(payload, dict):
            raise ValueError("요청은 JSON 객체여야 합니다")
        command = str(payload.get("command", "pick")).strip().lower()
        if command not in {"pick", "search", "search_and_pick"}:
            raise ValueError(f"지원하지 않는 command: {command}")
        name = str(payload.get("name") or payload.get("target_name") or "").strip()
        class_label = str(payload.get("class_label") or "").strip()
        if not name and not class_label:
            raise ValueError("name(target_name) 또는 class_label이 필요합니다")
        object_name = class_label or name
        supported = self.supported_object_names
        if object_name not in supported:
            raise ValueError(
                f"지원하지 않는 OBJ 이름: {object_name}; "
                f"지원 목록: {', '.join(supported)}"
            )
        task_id = str(payload.get("task_id") or "").strip()
        if not task_id:
            task_id = f"task-{self.node.get_clock().now().nanoseconds}"
        return RobotTask(
            task_id=task_id,
            name=object_name,
            class_label=object_name,
            requested_by=str(payload.get("requested_by", "state_node")).strip(),
            command=command,
        )

    def _on_task_request(self, request, response):
        try:
            task = self._parse_task(request.request)
        except ValueError as error:
            response.success = False
            response.response = "{}"
            response.message = str(error)
            return response

        rejection = ""
        queue_size = 0
        with self._lock:
            if not self._ready:
                rejection = "제어 노드 초기화가 완료되지 않았습니다"
            elif task.task_id in self._known_task_ids:
                rejection = f"중복 task_id입니다: {task.task_id}"
            elif len(self._pending) >= self.config.max_pending_tasks:
                rejection = "작업 대기열이 가득 찼습니다"
            else:
                rejection = self.acceptance_guard() or ""
            if not rejection:
                self._known_task_ids.add(task.task_id)
                self._pending.append(task)
                queue_size = len(self._pending)

        response.success = not rejection
        response.response = json.dumps(
            {
                "task_id": task.task_id,
                "outcome": (
                    TaskOutcome.REJECTED.value
                    if rejection
                    else TaskOutcome.QUEUED.value
                ),
                "queue_size": queue_size,
            },
            ensure_ascii=False,
        )
        response.message = rejection or "작업 요청을 접수했습니다"
        return response

    def _task_from_search_goal(self, goal_request, task_id: str) -> RobotTask:
        payload = {
            "task_id": task_id,
            "target_name": goal_request.target_name,
            "class_label": goal_request.class_label,
            "command": "search_and_pick",
            "requested_by": "state_node_action",
        }
        return self._parse_task(json.dumps(payload, ensure_ascii=False))

    def _on_search_goal(self, goal_request) -> GoalResponse:
        try:
            self._task_from_search_goal(goal_request, "goal-validation")
        except ValueError as error:
            self.node.get_logger().warning(f"Search goal rejected: {error}")
            return GoalResponse.REJECT
        with self._lock:
            rejection = (
                not self._ready
                or self._active is not None
                or len(self._pending) >= self.config.max_pending_tasks
            )
        if rejection or self.acceptance_guard():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_search_accepted(self, goal_handle) -> None:
        task_id = f"search-{bytes(goal_handle.goal_id.uuid).hex()}"
        task = self._task_from_search_goal(goal_handle.request, task_id)
        result_future = Future()
        with self._lock:
            self._known_task_ids.add(task_id)
            self._pending.append(task)
            self._action_handles[task_id] = goal_handle
            self._action_futures[task_id] = result_future
            self._action_locations[task_id] = ""
        goal_handle.execute()

    async def _execute_search(self, goal_handle):
        task_id = f"search-{bytes(goal_handle.goal_id.uuid).hex()}"
        with self._lock:
            result_future = self._action_futures[task_id]
        result = await result_future
        with self._lock:
            self._action_handles.pop(task_id, None)
            self._action_futures.pop(task_id, None)
            self._action_locations.pop(task_id, None)
        return result

    def _on_search_cancel(self, goal_handle) -> CancelResponse:
        task_id = f"search-{bytes(goal_handle.goal_id.uuid).hex()}"
        with self._lock:
            if self._active is not None and self._active.task_id == task_id:
                # A running robot motion cannot be canceled safely here.
                return CancelResponse.REJECT
            self._pending = deque(
                task for task in self._pending if task.task_id != task_id
            )
        self._finish_action(
            task_id,
            False,
            "",
            "탐색 요청이 취소되었습니다",
            canceled=True,
        )
        return CancelResponse.ACCEPT

    def set_task_location(self, task: RobotTask, location: str) -> None:
        with self._lock:
            if task.task_id in self._action_locations:
                self._action_locations[task.task_id] = location

    def _publish_action_feedback(
        self,
        task: RobotTask,
        step: str,
        progress: float,
    ) -> None:
        with self._lock:
            goal_handle = self._action_handles.get(task.task_id)
        if goal_handle is None or not goal_handle.is_active:
            return
        feedback = Search.Feedback()
        feedback.step = step
        feedback.progress = float(max(0.0, min(1.0, progress)))
        goal_handle.publish_feedback(feedback)

    def _finish_action(
        self,
        task_id: str,
        success: bool,
        location: str,
        message: str,
        *,
        canceled: bool = False,
    ) -> None:
        with self._lock:
            goal_handle = self._action_handles.get(task_id)
            result_future = self._action_futures.get(task_id)
        if goal_handle is None or result_future is None or result_future.done():
            return
        result = Search.Result()
        result.success = bool(success)
        result.location = location if success else ""
        result.message = message
        if canceled:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result_future.set_result(result)

    def publish_event(
        self,
        task: Optional[RobotTask],
        *,
        status: str,
        success: bool,
        outcome: str | TaskOutcome,
        message: str,
        extra: Optional[dict[str, Any]] = None,
    ) -> None:
        outcome_value = outcome.value if isinstance(outcome, TaskOutcome) else outcome
        payload: dict[str, Any] = {
            "task_id": task.task_id if task else None,
            "status": status,
            "success": success,
            "outcome": outcome_value,
            "message": message,
            "stamp_ns": self.node.get_clock().now().nanoseconds,
        }
        if task:
            payload.update(
                {
                    "target": {
                        "name": task.name,
                        "class_label": task.class_label,
                    },
                    "command": task.command,
                    "requested_by": task.requested_by,
                }
            )
        if extra:
            payload.update(extra)

        if task and task.task_id in self._action_handles:
            progress_by_outcome = {
                TaskOutcome.DB_LOOKUP.value: 0.05,
                TaskOutcome.WAITING_POSE.value: 0.20,
                TaskOutcome.SEARCHING.value: 0.20,
                TaskOutcome.ZONE_NOT_FOUND.value: 0.35,
                TaskOutcome.LANDMARK_SEARCHING.value: 0.45,
                TaskOutcome.LANDMARK_FOUND.value: 0.55,
                TaskOutcome.LANDMARK_NOT_FOUND.value: 0.55,
                TaskOutcome.GREEN_BOX_CLEANUP.value: 0.90,
                TaskOutcome.PICK_COMPLETED.value: 1.0,
            }
            self._publish_action_feedback(
                task,
                message,
                progress_by_outcome.get(outcome_value, 0.75),
            )
            if status == "completed":
                with self._lock:
                    location = self._action_locations.get(task.task_id, "")
                self._finish_action(
                    task.task_id,
                    success,
                    location,
                    message,
                )

        if not self.result_client.service_is_ready():
            self.result_client.wait_for_service(timeout_sec=0.2)
        if not self.result_client.service_is_ready():
            self.node.get_logger().warning(
                f"State result service unavailable: {self.config.state_result_service}"
            )
            return
        request = RobotResult.Request()
        request.request = json.dumps(payload, ensure_ascii=False)
        future = self.result_client.call_async(request)
        self._result_futures.add(future)
        future.add_done_callback(self._on_result_response)

    def _on_result_response(self, future) -> None:
        self._result_futures.discard(future)
        try:
            response = future.result()
            if response is None or not response.success:
                message = response.message if response else "empty response"
                self.node.get_logger().warning(
                    f"State rejected robot result: {message}"
                )
        except Exception as error:
            self.node.get_logger().warning(
                f"State result service call failed: {error}"
            )

    def take_next_task(self) -> Optional[RobotTask]:
        with self._lock:
            if self._active is not None or not self._pending:
                return None
            self._active = self._pending.popleft()
            return self._active

    def finish_task(self, task: RobotTask) -> None:
        with self._lock:
            if self._active == task:
                self._active = None
