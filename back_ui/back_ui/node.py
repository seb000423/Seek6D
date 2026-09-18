"""back_ui 진입점. `ros2 run back_ui node`로 실행한다.

GitHub ui 브랜치의 `/dsr01/joint_states` + FK + HTTP 구조를 유지하고,
`/ui/task_state` JSON 구독과 RealSense 컬러 영상 구독을 추가한다.

`objects`(물체 목록)는 토픽 구독이 아니라 `db` 노드의 `/db/load` 서비스를
타이머로 계속 폴링해서 채운다(2026-08-06) — db는 이벤트를 안 쏘고 "지금
상태를 물어보면 답해주는" 서비스라서, 최신 상태를 계속 보려면 이쪽에서
주기적으로 물어보는 수밖에 없다.
"""

import json
import time
from datetime import datetime

import cv2
import rclpy

from cv_bridge import CvBridge
from interfaces.srv import DbLoad
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

from back_ui.frame_store import FrameStore
from back_ui.http_server import start_server
from back_ui.robot_fk import compute_links
from back_ui.state_store import StateStore


_JOINT_UPDATE_MIN_INTERVAL = 0.1
_DB_POLL_INTERVAL = 1.0  # db/load를 이 주기로 계속 물어본다


def _items_to_objects(items_rows: list) -> list:
    """db `items` 행(class_name/confidence/x/y/z) -> front_ui `objects` 스키마.

    x/y/z는 비전이 mm 단위로 주는 것으로 보여서(값 크기가 수백대라 m로
    보기엔 너무 큼) 1000으로 나눠 m로 바꾼다 — front_ui의 pos는 base_link
    기준 미터다(README 참고). **좌표축 방향이 base_link(REP-103: x전방/
    y좌/z상)와 실제로 맞는지는 아직 검증 못 했다** — 카메라-로봇 캘리브레이션
    쪽에서 한 번 확인이 필요하다.
    """
    objects = []
    for row in items_rows:
        class_name = row.get("class_name")
        if not class_name:
            continue

        x, y, z = row.get("x"), row.get("y"), row.get("z")
        pos = None
        if x is not None and y is not None and z is not None:
            pos = [x / 1000.0, y / 1000.0, z / 1000.0]

        objects.append(
            {
                "id": class_name,
                "name": class_name,
                "category": None,
                "pos": pos,
                # db에 있다는 것 자체가 위치를 안다는 뜻이라 confirmed로 둔다.
                "status": "confirmed" if pos is not None else "unknown",
                "confidence": row.get("confidence"),
                "last_seen": row.get("last_seen"),
            }
        )
    return objects


# db의 tasks.status(SUCCEEDED/FAILED/ABORTED, db 저장 로그 명세 2026-08-06)를
# front_ui가 이미 알고 있는 코드값(labels.TASK_STATUS: RUNNING/SUCCESS/
# FAILED/CANCELED/ERROR)으로 바꾼다 — front_ui 쪽 코드는 안 건드리는 게
# 원칙이라(README) 여기서 맞춘다.
_TASK_STATUS_MAP = {
    "SUCCEEDED": "SUCCESS",
    "FAILED": "FAILED",
    "ABORTED": "CANCELED",
}


def _tasks_to_recent(task_rows: list) -> list:
    """db `tasks` 행 -> front_ui `recent_tasks` 스키마(task_id/target_name/
    result/ended_at/duration_sec). started_at/ended_at 차이로 duration_sec을
    계산한다 — db에는 그 필드가 없다(초 단위로 미리 안 구해둠)."""
    recent = []
    for row in task_rows:
        duration_sec = None
        started_at, ended_at = row.get("started_at"), row.get("ended_at")
        if started_at and ended_at:
            try:
                duration_sec = (
                    datetime.fromisoformat(ended_at) - datetime.fromisoformat(started_at)
                ).total_seconds()
            except ValueError:
                pass  # 시각 형식이 이상해도 나머지 필드는 그냥 보여준다

        recent.append(
            {
                "task_id": str(row.get("id")),
                "target_name": row.get("target_name"),
                "result": _TASK_STATUS_MAP.get(row.get("status"), row.get("status")),
                "ended_at": ended_at,
                "duration_sec": duration_sec,
            }
        )
    # 최신순(끝난 시각 내림차순)으로 — front_ui는 앞에서부터 5개만 쓴다(README).
    recent.sort(key=lambda r: r["ended_at"] or "", reverse=True)
    return recent


# state 노드의 targets 이름 -> StateStore.system.nodes 키.
# targets 에 노드를 추가하면 여기도 같이 늘린다.
# 'control' -> 'main' : front_ui 스키마의 노드 키가 'main'이라 제어 노드를 거기 매핑한다.
_NODE_KEY_MAP = {
    "db": "db",
    "control": "main",
    "vision": "image",
    "voice": "voice",
}


class BackUiNode(Node):
    def __init__(self):
        super().__init__("back_ui")

        # 상태 저장소와 이미지 저장소
        self.state_store = StateStore()
        self.frame_store = FrameStore()

        # HTTP 서버 시작
        self._httpd = start_server(
            self.state_store,
            self.frame_store,
        )

        addr, port = self._httpd.server_address
        self.get_logger().info(
            f"back_ui HTTP 서버 시작: http://{addr}:{port}"
        )

        # RealSense 컬러 영상 구독
        self._bridge = CvBridge()

        self._image_subscription = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._on_image,
            10,
        )

        self.get_logger().info(
            "카메라 구독 시작: "
            "/camera/camera/color/image_raw"
        )

        # 로봇 joint_states 구독
        self.declare_parameter(
            "robot_name",
            "dsr01",
        )

        robot_name = (
            self.get_parameter("robot_name")
            .get_parameter_value()
            .string_value
        )

        joint_states_topic = (
            f"/{robot_name}/joint_states"
        )

        self._joint_subscription = (
            self.create_subscription(
                JointState,
                joint_states_topic,
                self._on_joint_states,
                10,
            )
        )

        self.get_logger().info(
            f"구독 시작: {joint_states_topic}"
        )

        self._last_joint_update = 0.0

        # UI 작업 상태 토픽 구독
        self._task_subscription = (
            self.create_subscription(
                String,
                "/ui/task_state",
                self._on_task_state,
                10,
            )
        )

        self.get_logger().info(
            "구독 시작: /ui/task_state"
        )

        # 상태 노드의 현재 상태(LOAD/IDLE/RUN)와 노드별 준비 여부.
        # state 노드가 TRANSIENT_LOCAL 로 발행하므로 늦게 떠도 마지막 값을 받는다
        self._state_subscription = self.create_subscription(
            String,
            "/state/current",
            self._on_state_current,
            QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.get_logger().info(
            "구독 시작: /state/current"
        )

        # db 노드 폴링 (구독 아니고 서비스라 타이머로 계속 물어봄) —
        # items(홈/작업 3D 맵의 objects), tasks(로그 화면의 recent_tasks)
        self._db_load_cli = self.create_client(
            DbLoad,
            "db/load",
        )

        self._db_poll_timer = self.create_timer(
            _DB_POLL_INTERVAL,
            self._poll_db,
        )

        self.get_logger().info(
            f"db/load 폴링 시작 ({_DB_POLL_INTERVAL}초 간격, items+tasks)"
        )

    def _poll_db(self):
        """타이머 콜백. call_async만 쓴다 — 응답 기다리다가 다음 틱을
        막으면 안 된다(다른 콜백도 있는 노드라 데드락/지연 위험). 한 틱에
        items/tasks 둘 다 물어본다."""
        if not self._db_load_cli.service_is_ready():
            # db 노드가 아직 안 떴거나 잠깐 끊긴 것 — 다음 틱에 다시 시도.
            return

        for table, callback in (
            ("items", self._on_items_loaded),
            ("tasks", self._on_tasks_loaded),
        ):
            req = DbLoad.Request()
            req.request = json.dumps({"table": table}, ensure_ascii=False)
            future = self._db_load_cli.call_async(req)
            future.add_done_callback(callback)

    def _on_items_loaded(self, future):
        try:
            res = future.result()
        except Exception as error:      # noqa: BLE001
            self.get_logger().error(f"db/load(items) 호출 실패: {error}")
            return

        if not res.success:
            self.get_logger().warning(f"db/load(items) 실패 응답: {res.message}")
            return

        try:
            data = json.loads(res.response)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"db/load(items) 응답 JSON 파싱 실패: {error}")
            return

        objects = _items_to_objects(data.get("items", []))
        self.state_store.update_objects(objects)

    def _on_tasks_loaded(self, future):
        try:
            res = future.result()
        except Exception as error:      # noqa: BLE001
            self.get_logger().error(f"db/load(tasks) 호출 실패: {error}")
            return

        if not res.success:
            self.get_logger().warning(f"db/load(tasks) 실패 응답: {res.message}")
            return

        try:
            data = json.loads(res.response)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"db/load(tasks) 응답 JSON 파싱 실패: {error}")
            return

        recent_tasks = _tasks_to_recent(data.get("rows", []))
        self.state_store.update_recent_tasks(recent_tasks)

    def _on_image(self, msg: Image):
        """ROS Image를 JPEG로 변환해 HTTP 프레임 저장소에 넣는다."""
        try:
            # sensor_msgs/Image → OpenCV BGR 이미지
            frame = self._bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            # OpenCV 이미지 → JPEG
            success, jpeg = cv2.imencode(
                ".jpg",
                frame,
                [
                    cv2.IMWRITE_JPEG_QUALITY,
                    80,
                ],
            )

            if not success:
                self.get_logger().warning(
                    "JPEG 인코딩에 실패했습니다."
                )
                return

            # /frame.jpg에서 사용할 최신 이미지 저장
            self.frame_store.set_latest(
                jpeg.tobytes()
            )

            # FrameStore의 현재 frame_id 읽기
            frame_id, _ = (
                self.frame_store.get_latest()
            )

            # /state의 frame_id 갱신
            self.state_store.update_frame_id(
                frame_id
            )

            # UI가 카메라 영상을 표시하도록 상태 갱신
            self.state_store.update_system(
                camera_connected=True
            )

        except Exception as error:
            self.get_logger().error(
                f"카메라 이미지 처리 오류: {error}"
            )

    def _on_joint_states(
        self,
        msg: JointState,
    ):
        """관절 상태를 이용해 3D 로봇 링크 좌표를 갱신한다."""
        now = time.monotonic()

        if (
            now - self._last_joint_update
            < _JOINT_UPDATE_MIN_INTERVAL
        ):
            return

        self._last_joint_update = now

        angles = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        links = compute_links(angles)

        self.state_store.update_robot_links(
            links
        )

    def _on_task_state(
        self,
        msg: String,
    ):
        """String 안의 JSON을 UI task 상태로 반영한다."""
        try:
            data = json.loads(msg.data)

        except json.JSONDecodeError as error:
            self.get_logger().error(
                "/ui/task_state JSON 형식 오류: "
                f"{error}"
            )
            return

        if not isinstance(data, dict):
            self.get_logger().error(
                "/ui/task_state 최상위 JSON은 "
                "객체여야 합니다."
            )
            return

        self.state_store.update_task(
            task_id=data.get("task_id"),
            target_id=data.get("target_id"),
            target_name=data.get("target_name"),
            voice_command=data.get(
                "voice_command"
            ),
            status=data.get("status"),
            stage=data.get("stage"),
            action=data.get("action"),
            action_reason=data.get(
                "action_reason"
            ),
            elapsed_sec=data.get(
                "elapsed_sec"
            ),
            current_zone=data.get(
                "current_zone"
            ),
            detections=data.get(
                "detections"
            ),
        )

        self.get_logger().info(
            "UI 상태 갱신: "
            f"action={data.get('action')}, "
            f"reason={data.get('action_reason')}"
        )

    def _on_state_current(self, msg: String):
        """상태 노드가 발행하는 {"state","ready","reason"}을 system에 반영한다."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f"/state/current JSON 형식 오류: {error}")
            return

        nodes = dict(self.state_store.get_snapshot()["system"]["nodes"])
        nodes["state"] = True  # 이 메시지가 왔다는 것 자체가 상태 노드가 떠 있다는 증거
        for name, ready in (data.get("ready") or {}).items():
            key = _NODE_KEY_MAP.get(name)
            if key:
                nodes[key] = bool(ready)

        self.state_store.update_system(state=data.get("state"), nodes=nodes)

    def destroy_node(self):
        """ROS 노드 종료 시 HTTP 서버도 함께 종료한다."""
        self._httpd.shutdown()
        self._httpd.server_close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = BackUiNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
