"""드래그로 돌려보는 3D 물체 뷰어.

원래는 "런타임 OBJ 로딩"과 "자유 회전 인터랙티브 3D 뷰어"를 금지했으나,
2026-08-05 사용자 결정으로 홈 화면 "찾고 있는 3d 모델" 패널에 한해 예외를 둔다.

단, front_ui 의존성 규칙(flet/numpy/requests 셋뿐)은 그대로 지킨다.
pyrender·trimesh·open3d 같은 3D 라이브러리는 쓰지 않는다. 대신:

  1. OBJ는 앱 시작 시 한 번만 순수 파이썬으로 읽는다 (면 중심점 + 면 법선만
     추출하는 정도라 무겁지 않다).
  2. 매 드래그 프레임마다 numpy로 회전·투영·셰이딩을 다시 계산해
     면 중심점들을 작은 점으로 스플랫(splat)한다 — 폴리곤 래스터라이즈보다
     훨씬 가볍고, 이 정도 밀도(면 수천 개)면 뭉쳐서 꽤 꽉 찬 표면처럼 보인다.
  3. 결과를 BMP로 인코딩해 ft.Image에 raw bytes로 먹인다. BMP는 압축이 없어
     PNG 인코더 없이 numpy 배열 → 바이트 변환만으로 끝난다 (Flet은 BMP를
     지원 포맷으로 명시하고 있다).
"""

import struct
from pathlib import Path

import flet as ft
import numpy as np

import theme as t

# 광원 2개(주광+보조광)로 완전히 어두운 면이 없게 한다.
# 조명 때문에 형태가 잘 안 보인다는 피드백 반영 — 앰비언트 바닥값을 높게 잡는다.
_KEY_LIGHT = np.array([0.35, 0.55, 0.77])
_KEY_LIGHT = _KEY_LIGHT / np.linalg.norm(_KEY_LIGHT)
_FILL_LIGHT = np.array([-0.55, 0.15, 0.35])
_FILL_LIGHT = _FILL_LIGHT / np.linalg.norm(_FILL_LIGHT)
_AMBIENT_FLOOR = 0.45

_STAMP = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]  # 3x3

_DRAG_SENSITIVITY = 0.35  # 픽셀 이동 -> 각도(도)
_PITCH_LIMIT = 85.0
_MAX_POINTS = 6000  # 이보다 많으면 골고루 솎아낸다. 원래 병목(전체 화면 재생성)을
# 고쳤으니 이제 여유가 있다 — 너무 낮추면 점이 성겨 보인다.


def _hex_to_rgb01(hex_color: str) -> np.ndarray:
    hex_color = hex_color.lstrip("#")
    return np.array([int(hex_color[i : i + 2], 16) / 255 for i in (0, 2, 4)])


def _rotation_matrix(yaw_deg: float, pitch_deg: float) -> np.ndarray:
    y, p = np.radians(yaw_deg), np.radians(pitch_deg)
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, np.cos(p), -np.sin(p)], [0, np.sin(p), np.cos(p)]])
    return rx @ rz


def _load_face_centroids_and_normals(obj_path: Path) -> tuple[np.ndarray, np.ndarray]:
    """v / f 라인만 읽는 최소 OBJ 파서. vt/vn/mtl은 무시한다.

    render_object.py 의 파서와 같은 역할이지만, 그건 개발 중 1회용 오프라인
    도구고 이건 앱이 매번 시작할 때 도는 런타임 코드라 여기 따로 둔다.
    """
    vertices = []
    faces = []
    with open(obj_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                vertices.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
                for i in range(1, len(idx) - 1):  # fan triangulation
                    faces.append((idx[0], idx[i], idx[i + 1]))

    v = np.array(vertices, dtype=float)
    center = (v.max(axis=0) + v.min(axis=0)) / 2
    scale = np.abs(v - center).max()
    v = (v - center) / scale

    tris = v[np.array(faces)]  # (F, 3, 3)
    centroids = tris.mean(axis=1)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths == 0] = 1
    normals = normals / lengths

    # 점이 너무 빽빽해 보인다는 피드백: 면이 _MAX_POINTS보다 많으면 골고루 솎아낸다.
    # (일부만 버려도 어차피 표면 전체에 고르게 퍼진 면들이라 형태는 그대로 남는다)
    if len(centroids) > _MAX_POINTS:
        stride = -(-len(centroids) // _MAX_POINTS)  # 올림 나눗셈
        centroids = centroids[::stride]
        normals = normals[::stride]

    return centroids, normals


def _encode_bmp(rgb: np.ndarray) -> bytes:
    """(H,W,3) uint8 배열을 24bit BMP로. 압축 없어서 외부 인코더가 필요 없다."""
    h, w, _ = rgb.shape
    row_bytes = w * 3
    pad = (-row_bytes) % 4
    bgr = rgb[::-1, :, ::-1]  # BMP는 아래->위, BGR 순서
    if pad:
        packed = np.concatenate(
            [bgr.reshape(h, row_bytes), np.zeros((h, pad), dtype=np.uint8)], axis=1
        )
    else:
        packed = bgr.reshape(h, row_bytes)
    pixel_data = packed.tobytes()

    file_header = struct.pack("<2sIHHI", b"BM", 14 + 40 + len(pixel_data), 0, 0, 54)
    info_header = struct.pack(
        "<IiiHHIIiiII", 40, w, h, 1, 24, 0, len(pixel_data), 2835, 2835, 0, 0
    )
    return file_header + info_header + pixel_data


class ObjectViewer:
    """`.control`을 패널 content에 그대로 넣으면 된다. 드래그하면 돈다."""

    def __init__(
        self,
        obj_path: Path,
        base_color_hex: str = "#5AA86A",
        size: int = 200,
        render_size: int = 140,
        initial_yaw: float = -60.0,
        initial_pitch: float = 18.0,
    ):
        self._centroids, self._normals = _load_face_centroids_and_normals(obj_path)
        self._base_color = _hex_to_rgb01(base_color_hex)
        self._bg_color = self._panel_bg_rgb()
        self._size = size  # 화면에 보이는 크기
        self._render_size = render_size  # 실제로 그리는(=전송하는) 해상도
        self._yaw = initial_yaw
        self._pitch = initial_pitch
        self._yaw_at_drag_start = initial_yaw
        self._pitch_at_drag_start = initial_pitch

        self._image = ft.Image(
            src=self._render(),
            width=size,
            height=size,
            fit=ft.BoxFit.CONTAIN,
            border_radius=t.RADIUS,
            gapless_playback=True,  # 새 프레임 올 때까지 이전 프레임 유지 (깜빡임 방지)
        )
        self.control = ft.GestureDetector(
            content=self._image,
            drag_interval=35,  # 초당 프레임 수를 제한해서 웹소켓 트래픽을 줄인다
            mouse_cursor=ft.MouseCursor.MOVE,
            on_pan_start=self._on_pan_start,
            on_pan_update=self._on_pan_update,
        )

    @staticmethod
    def _panel_bg_rgb() -> np.ndarray:
        return _hex_to_rgb01(t.SURFACE)

    def _render(self) -> bytes:
        r = _rotation_matrix(self._yaw, self._pitch)
        rc = self._centroids @ r.T
        rn = self._normals @ r.T

        shade = 0.7 * np.clip(rn @ _KEY_LIGHT, 0, 1) + 0.3 * np.clip(rn @ _FILL_LIGHT, 0, 1)
        shade = np.clip(shade, _AMBIENT_FLOOR, 1.0)
        colors = np.clip(self._base_color[None, :] * shade[:, None] * 255, 0, 255).astype(
            np.uint8
        )

        size = self._render_size
        # 회전할 때마다 화면에 보이는 실루엣의 2D 바운딩박스를 기준으로 다시
        # 가운데를 맞추고 배율도 다시 잡는다. 3D bbox 중심으로 고정해두면
        # 보는 각도에 따라 화면 안에서 한쪽으로 쏠려 보인다.
        proj_x, proj_z = rc[:, 0], rc[:, 2]
        mid_x = (proj_x.max() + proj_x.min()) / 2
        mid_z = (proj_z.max() + proj_z.min()) / 2
        extent = max(proj_x.max() - proj_x.min(), proj_z.max() - proj_z.min(), 1e-6)
        scale = (size * 0.88) / extent

        cx = cy = size / 2
        x2d = (cx + (proj_x - mid_x) * scale).astype(np.int32)
        y2d = (cy - (proj_z - mid_z) * scale).astype(np.int32)
        depth = rc[:, 1]

        order = np.argsort(depth)  # 먼 것부터 그려서 가까운 점이 나중에 덮어쓴다
        x2d, y2d, colors = x2d[order], y2d[order], colors[order]

        canvas = np.tile(
            (self._bg_color * 255).astype(np.uint8), (size, size, 1)
        )
        for dx, dy in _STAMP:
            xs, ys = x2d + dx, y2d + dy
            mask = (xs >= 0) & (xs < size) & (ys >= 0) & (ys < size)
            canvas[ys[mask], xs[mask]] = colors[mask]

        return _encode_bmp(canvas)

    def _on_pan_start(self, e: ft.DragStartEvent):
        self._yaw_at_drag_start = self._yaw
        self._pitch_at_drag_start = self._pitch

    def _on_pan_update(self, e: ft.DragUpdateEvent):
        delta = e.global_delta
        if delta is None:
            return
        self._yaw = self._yaw_at_drag_start - delta.x * _DRAG_SENSITIVITY
        pitch = self._pitch_at_drag_start + delta.y * _DRAG_SENSITIVITY
        self._pitch = max(-_PITCH_LIMIT, min(_PITCH_LIMIT, pitch))
        self._image.src = self._render()
        self._image.update()
