"""카메라 프레임 저장소. `/frame.jpg`용.

front_ui는 아직 카메라 패널을 안 붙였다(README "아직 미사용" 참고) — 그래도
주소는 이미 예약돼 있어서, 이미지 토픽 구독이 나중에 붙을 자리를 미리
잡아둔다. 지금은 아무도 `set_latest()`를 안 불러서 항상 비어 있고,
http_server.py가 그 경우 501을 돌려준다(fake_server.py와 동일).
"""

import threading


class FrameStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame_id = 0
        self._jpeg_bytes: bytes | None = None

    def get_latest(self) -> tuple[int, bytes | None]:
        with self._lock:
            return self._frame_id, self._jpeg_bytes

    def set_latest(self, jpeg_bytes: bytes):
        """이미지 토픽 콜백이 나중에 이걸 부르게 된다."""
        with self._lock:
            self._frame_id += 1
            self._jpeg_bytes = jpeg_bytes
