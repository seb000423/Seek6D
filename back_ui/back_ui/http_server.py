"""front_ui가 폴링하는 HTTP 서버.

GitHub ui 브랜치의 start_server 기반 구조를 그대로 사용한다.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

HOST = "127.0.0.1"
PORT = 8765

STATE_PATH = "/state"
HEALTH_PATH = "/health"
FRAME_PATH = "/frame.jpg"


def _make_handler(state_store, frame_store):
    class Handler(BaseHTTPRequestHandler):
        def _send_json(self, payload: dict, status: int = 200):
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_jpeg(self, jpeg_bytes: bytes):
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg_bytes)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(jpeg_bytes)

        def do_GET(self):
            path = urlparse(self.path).path

            if path == STATE_PATH:
                self._send_json(state_store.get_snapshot())
            elif path == HEALTH_PATH:
                self._send_json({"ok": True})
            elif path == FRAME_PATH:
                _frame_id, jpeg_bytes = frame_store.get_latest()
                if jpeg_bytes is None:
                    self._send_json(
                        {"error": "not_implemented"},
                        status=501,
                    )
                else:
                    self._send_jpeg(jpeg_bytes)
            else:
                self._send_json({"error": "not_found"}, status=404)

        def log_message(self, fmt, *args):
            pass

    return Handler


def start_server(
    state_store,
    frame_store,
    host: str = HOST,
    port: int = PORT,
) -> ThreadingHTTPServer:
    handler_cls = _make_handler(state_store, frame_store)
    httpd = ThreadingHTTPServer((host, port), handler_cls)
    thread = threading.Thread(
        target=httpd.serve_forever,
        daemon=True,
    )
    thread.start()
    return httpd
