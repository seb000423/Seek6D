"""현재 상태 스냅샷을 들고 있는 스레드 세이프 저장소.

GitHub ui 브랜치의 StateStore를 기준으로 유지하면서, 어제 테스트한
`/ui/task_state` 갱신 기능만 추가했다.
"""

import copy
import threading
import time

_ROBOT_STALE_AFTER = 2.0


def _base_task() -> dict:
    return {
        "task_id": "-",
        "voice_command": "-",
        "target_id": None,
        "target_name": "-",
        "status": "WAITING",
        "stage": "idle",
        "elapsed_sec": 0,
        "current_zone": "-",
        "action": "작업 명령을 기다리고 있습니다.",
        "action_reason": "현재 실행 중인 작업이 없습니다.",
        "detections": [],
    }


def _base_snapshot() -> dict:
    return {
        "frame_id": 0,
        "system": {
            "state": "IDLE",
            "nodes": {
                "image": False,
                "main": False,
                "db": False,
                "voice": False,
                "state": False,
            },
            "robot_connected": False,
            "camera_connected": False,
            "gripper_state": "open",
            "object_count_total": 0,
            "object_count_confirmed": 0,
            "object_count_unknown": 0,
        },
        "task": _base_task(),
        "objects": [],
        "zones": [],
        "recent_tasks": [],
    }


class StateStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._snapshot = _base_snapshot()
        self._robot_links: list[dict] = []
        self._last_joint_states_time: float | None = None

    def update_robot_links(self, links: list[dict]):
        """GitHub 기존 기능: 관절 상태로 계산된 로봇 링크를 저장한다."""
        with self._lock:
            self._robot_links = copy.deepcopy(links)
            self._last_joint_states_time = time.time()

    def update_task(self, **values):
        """추가 기능: `/ui/task_state`에서 받은 task 필드만 갱신한다."""
        with self._lock:
            task = self._snapshot["task"]
            for key, value in values.items():
                if key in task and value is not None:
                    task[key] = copy.deepcopy(value)

    def update_system(self, **values):
        """향후 카메라/그리퍼 콜백에서 system 필드를 갱신할 때 사용한다."""
        with self._lock:
            system = self._snapshot["system"]
            for key, value in values.items():
                if key in system and value is not None:
                    system[key] = copy.deepcopy(value)

    def update_frame_id(self, frame_id: int):
        with self._lock:
            self._snapshot["frame_id"] = int(frame_id)

    def update_recent_tasks(self, recent_tasks: list[dict]):
        """추가 기능: db의 tasks 폴링 결과로 recent_tasks 전체를 갈아끼운다.
        (update_objects와 같은 이유로 병합하지 않고 통째로 교체한다.)"""
        with self._lock:
            self._snapshot["recent_tasks"] = copy.deepcopy(recent_tasks)

    def update_objects(self, objects: list[dict]):
        """추가 기능: db의 items 폴링 결과로 objects 전체를 갈아끼운다.

        델타가 아니라 매번 "지금 db에 있는 전체 목록"으로 통째로 교체한다
        — db 쪽 items가 사라진 물건은 그대로 없어져야(사라졌다는 걸 UI가
        알아야) 하므로 이전 값과 병합하지 않는다.
        """
        with self._lock:
            self._snapshot["objects"] = copy.deepcopy(objects)

    def get_snapshot(self) -> dict:
        """HTTP `/state`가 사용할 현재 상태 복사본을 반환한다."""
        with self._lock:
            snap = copy.deepcopy(self._snapshot)
            robot_links = copy.deepcopy(self._robot_links)
            last_joint_states_time = self._last_joint_states_time

        now = time.time()
        snap["ts"] = now
        snap["robot"] = {"links": robot_links}
        snap["system"]["robot_connected"] = (
            last_joint_states_time is not None
            and (now - last_joint_states_time) < _ROBOT_STALE_AFTER
        )
        return snap
