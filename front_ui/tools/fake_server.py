"""개발용 가짜 /state 서버.

back_ui가 실제로 내보낼 것과 완전히 같은 HTTP 인터페이스(주소·경로·JSON 스키마)를
흉내낸다.

back_ui가 준비되면 이 파일 대신 `ros2 run back_ui node`를 띄우기만 하면 되고,
front_ui 쪽(state_client.py, views/*)은 손대지 않는다. state_client는 이게
fake_server인지 진짜 back_ui인지 구분할 방법이 없다.

실행:
    conda activate front_ui
    cd front_ui
    python tools/fake_server.py
"""

import json
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import config as cfg  # noqa: E402
import labels as L  # noqa: E402

START = time.time()

STAGE_CODES = L.STAGE_ORDER  # idle ... done 순서 그대로
STAGE_DURATION = 4.0  # 각 단계에 머무는 시간(초). 데모 속도 조절용.

# 로봇 팔(M0609) 링크별 위치/자세. 실제 관절 각도가 없어서(back_ui가 아직
# 없다) 전부 0도일 때의 자세를 URDF 조인트 원점들을 이어붙여 미리 계산해
# 넣었다 — 다 펴진 채로 위를 향하는 자세라 실제 동작 모습은 아니지만,
# tools/mesh_to_points.py로 만든 점군이 base_link 기준으로 잘 배치되는지
# 확인하기엔 충분하다. 값 그대로 쓰지 말고 back_ui가 실제 FK로 갈아끼운다.
ROBOT_LINKS_ZERO_POSE = [
    {"name": "base_link", "pos": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
    {"name": "link_1", "pos": [0.0, 0.0, 0.1345], "rpy": [0.0, 0.0, 0.0]},
    {"name": "link_2", "pos": [0.0, 0.006, 0.1345], "rpy": [3.1416, -1.5706, 1.5706]},
    {"name": "link_3", "pos": [0.0, 0.0061, 0.5455], "rpy": [-1.571, 0.0002, -0.0002]},
    {"name": "link_4", "pos": [0.0001, 0.0062, 0.9135], "rpy": [0.0, 0.0002, -0.0002]},
    {"name": "link_5", "pos": [0.0001, 0.0062, 0.9135], "rpy": [-1.571, 0.0002, -0.0002]},
    {"name": "link_6", "pos": [0.0001, 0.0062, 1.0345], "rpy": [0.0, 0.0002, -0.0002]},
]


def _stage_progress(elapsed: float) -> tuple[str, float]:
    """경과 시간을 STAGES 순환으로 환산한다.

    (현재 stage 코드, 그 stage 안에서 흐른 시간) 을 돌려준다.
    """
    cycle = STAGE_DURATION * len(STAGE_CODES)
    pos = elapsed % cycle
    idx = int(pos // STAGE_DURATION)
    return STAGE_CODES[idx], pos - idx * STAGE_DURATION


def build_snapshot() -> dict:
    elapsed = time.time() - START
    stage, stage_elapsed = _stage_progress(elapsed)

    return {
        "ts": time.time(),
        "frame_id": int(elapsed * 10),
        "system": {
            "state": "RUN",
            "nodes": {
                "image": True, "main": True, "db": True, "voice": True, "state": True,
            },
            "robot_connected": True,
            "camera_connected": True,
            "gripper_state": "open",
            "object_count_total": 12,
            "object_count_confirmed": 9,
            "object_count_unknown": 3,
        },
        "task": {
            "task_id": "task_demo_001",
            "voice_command": "빨간 컵 좀 찾아줘",
            "target_id": "cup_red_01",
            "target_name": "빨간 컵",
            "status": "RUNNING",
            "stage": stage,
            "action": "Drawer_A_2 내부를 관측하고 있습니다.",
            "action_reason": "초기 관측에서 대상이 확인되지 않았습니다.",
            "elapsed_sec": round(elapsed, 1),
            "current_zone": "drawer_a_2",
            "detections": [
                {"label": "빨간 컵 후보", "confidence": 0.91},
            ],
        },
        "objects": [
            {
                "id": "cup_red_01", "name": "빨간 컵", "category": "cup",
                "pos": [0.42, 0.18, 0.31], "zone": "drawer_a_2",
                "status": "confirmed", "confidence": 0.92,
                "last_seen": "2026-08-04T12:00:00",
            },
            {
                "id": "box_blue_01", "name": "파란 상자", "category": "box",
                "pos": None, "zone": None,
                "status": "unknown", "confidence": None,
                "last_seen": "2026-08-04T11:40:00",
            },
            # 3D 지도에 마커 여러 개 찍히는 모양 보려고 대충 선반 위에 흩어놓은
            # 더미 물체들. 정확한 실측 위치 아님 — scene_config.json의 shelf
            # 박스(pos=[-0.48,-0.32,0.12], size=[0.30,1.00,0.24]) 윗면(z=0.24)
            # 근처에 대충 얹었다.
            {
                "id": "mug_01", "name": "머그컵", "category": "cup",
                "pos": [-0.48, -0.10, 0.27], "zone": None,
                "status": "confirmed", "confidence": 0.88,
                "last_seen": "2026-08-05T09:00:00",
            },
            {
                "id": "tape_01", "name": "테이프", "category": "misc",
                "pos": [-0.55, 0.05, 0.26], "zone": None,
                "status": "searching", "confidence": None,
                "last_seen": "2026-08-05T09:01:00",
            },
            {
                "id": "cable_01", "name": "케이블", "category": "misc",
                "pos": [-0.40, -0.35, 0.26], "zone": None,
                "status": "warning", "confidence": 0.4,
                "last_seen": "2026-08-05T09:02:00",
            },
        ],
        # zones는 아직 실측 위치가 없다 (수납장/박스처럼 실제 치수를 받기 전까지는
        # 임의로 안 채운다). 실측값이 오면 여기에 넣는다.
        # MapView의 zones 렌더링 코드(render3d/map_view.py)는
        # 그대로 둔다 — 데이터가 비어있으면 그냥 아무것도 안 그린다.
        "zones": [],
        "robot": {"links": ROBOT_LINKS_ZERO_POSE},
        "recent_tasks": [
            {
                "task_id": "task_20260804_001", "target_name": "빨간 컵",
                "result": "SUCCESS", "ended_at": "2026-08-04T12:03:38", "duration_sec": 38,
            },
            {
                "task_id": "task_20260804_000", "target_name": "파란 상자",
                "result": "FAILED", "ended_at": "2026-08-04T11:40:52", "duration_sec": 52,
            },
        ],
    }


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, payload: dict, status: int = 200):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == cfg.STATE_PATH:
            self._send_json(build_snapshot())
        elif path == cfg.HEALTH_PATH:
            self._send_json({"ok": True})
        elif path == cfg.FRAME_PATH:
            # TODO: 작업(monitor) 화면에서 카메라 패널을 붙일 때 실제 JPEG로 구현한다.
            self._send_json({"error": "not_implemented"}, status=501)
        else:
            self._send_json({"error": "not_found"}, status=404)

    def log_message(self, fmt, *args):
        pass  # 폴링마다 콘솔에 찍히면 개발 중 로그가 묻힌다. 필요하면 지운다.


def main():
    parsed = urlparse(cfg.BASE_URL)
    addr = (parsed.hostname, parsed.port)
    httpd = ThreadingHTTPServer(addr, Handler)
    print(f"fake_server: http://{addr[0]}:{addr[1]}{cfg.STATE_PATH}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
