"""back_ui(또는 개발 중엔 fake_server)의 /state 를 폴링하는 클라이언트.

HTTP 스키마만 맞으면 되고, 반대편이 fake_server인지 진짜 back_ui인지
구분하지 않는다. back_ui가 완성돼도 이 파일은 고치지 않는다.
"""

import threading
import time
from typing import Callable, Optional

import requests

import config as cfg

Snapshot = dict

STATUS_CONNECTED = "connected"
STATUS_STALE = "stale"
STATUS_DISCONNECTED = "disconnected"


class StateClient:
    """백그라운드 스레드에서 주기적으로 /state를 가져와 콜백으로 넘긴다.

    콜백은 폴링 스레드에서 그대로 호출된다. Flet 컨트롤 값을 바꾼 뒤
    page.update()를 부르는 것은 콜백을 넘긴 쪽(main.py)의 책임이다.
    (theme.py 규칙 참고: 핸들러 밖에서 화면을 바꿀 때는 update()를 직접 부른다)
    """

    def __init__(
        self,
        interval: float = cfg.POLL_HOME,
        base_url: str = cfg.BASE_URL,
        on_snapshot: Optional[Callable[[Snapshot], None]] = None,
        on_status: Optional[Callable[[str, Optional[float]], None]] = None,
    ):
        self._interval = interval
        self._url = base_url + cfg.STATE_PATH
        self._on_snapshot = on_snapshot
        self._on_status = on_status
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _loop(self):
        while not self._stop.is_set():
            self._poll_once()
            self._stop.wait(self._interval)

    def _poll_once(self):
        try:
            resp = requests.get(self._url, timeout=cfg.REQUEST_TIMEOUT)
            resp.raise_for_status()
            snapshot = resp.json()
        except (requests.RequestException, ValueError):
            # ValueError: 응답은 왔는데 JSON이 아님 (back_ui가 반쯤 떠 있는 경우 등)
            self._report_status(STATUS_DISCONNECTED, None)
            return

        if self._on_snapshot:
            self._on_snapshot(snapshot)

        age = time.time() - snapshot.get("ts", 0)
        if age > cfg.STALE_AFTER:
            self._report_status(STATUS_STALE, age)
        else:
            self._report_status(STATUS_CONNECTED, age)

    def _report_status(self, status: str, age: Optional[float]):
        if self._on_status:
            self._on_status(status, age)
