"""3D 지도 Flet 컨트롤. 마우스로 돌리고 확대한다.

구현 순서 1~7(투영 성능 측정 -> 그리드/축 -> 마우스 회전·줌 -> 책상/선반
박스 -> zones+서랍 열림 -> objects 마커 -> 로봇 팔 점군)까지 됐다.
`build_map()`(snapshot을 받는 최종 형태 하나로 통합하는 함수)만 아직이다 —
지금은 HomeView/MonitorView가 각자 update()를 필요한 인자로 부른다.
"""

import time

import flet as ft
import flet.canvas as cv
import numpy as np

import labels as L
import theme as t
from render3d.projection import Camera
from render3d.robot_points import build_robot_points, has_data as has_robot_data
from render3d.scene import load_scene_config
from render3d.shapes import (
    draw_axes,
    draw_box_wire,
    draw_grid,
    draw_label,
    draw_marker,
    draw_points,
)

# 로봇 팔 점군(assets/robot/*.npy)이 아직 없을 때(tools/mesh_to_points.py를
# 안 돌렸을 때) 대신 그리는 자리 표시 박스 크기. 실측/URDF 값 아니다.
_ROBOT_BOX_SIZE = (0.20, 0.20, 0.35)  # x,y,z (m)

_DEFAULT_YAW = -0.6
_DEFAULT_PITCH = 0.5
_DEFAULT_SCALE = 300.0
_SCALE_MIN, _SCALE_MAX = 50.0, 1200.0
_PITCH_LIMIT = 1.4  # 라디안. 이 이상 돌면 화면이 뒤집힌다

_DRAG_SENSITIVITY = 0.01  # 드래그 픽셀 -> 라디안
_SCROLL_SENSITIVITY = 0.0015  # 휠 델타 -> 배율 변화


def _scene_pivot(floor_cfg: dict) -> tuple[float, float, float]:
    """카메라 회전축을 바닥판 중앙쯤으로 잡는다.

    원점(로봇)은 장면 한쪽 구석이라 그걸 축으로 돌리면 선반/수납장/박스가
    화면 밖으로 크게 튕겨 나갔다 들어온다. 바닥판 중심(x,y)에, 높이는 선반/
    수납장이 있는 대략 중간 높이(z)를 줘서 돌릴 때 장면이 화면 안에 그대로
    있는 것처럼 보이게 한다.
    """
    pos = floor_cfg.get("pos", [0.0, 0.0])
    return (pos[0], pos[1], 0.15)


def _zone_box_params(zone: dict):
    """open_ratio를 반영한 현재 중심/회전을 계산한다.

    캐시하면 안 된다 — 열리는 동안 매 프레임 다시 계산해야 서랍이 스르륵
    나오는 게 보인다. 호출하는 쪽(_redraw)이 매번 새로 부른다.
    """
    pos = np.array(zone.get("pos", [0.0, 0.0, 0.0]), dtype=float)
    size = zone.get("size", [0.1, 0.1, 0.1])
    open_ratio = zone.get("open_ratio") or 0.0

    if zone.get("type") == "door":
        # TODO: 진짜 힌지 스윙(한쪽 변 고정)이 아니라 상자 중심 기준 회전이다.
        # 스키마에 힌지 방향이 없어서 정확히 흉내내려면 draw_box_wire가 중심과
        # 다른 회전축을 받아야 한다. 문 zone이 실제로 생기면 그때 손본다.
        angle = open_ratio * (np.pi / 2)
        return pos, size, (0.0, 0.0, angle)

    # 서랍(기본값): open_axis 방향으로 이동. 회전은 없다.
    open_axis = np.array(zone.get("open_axis", [1.0, 0.0, 0.0]), dtype=float)
    axis_idx = int(np.argmax(np.abs(open_axis))) if np.any(open_axis) else 0
    center = pos + open_axis * open_ratio * size[axis_idx]
    return center, size, None


class MapView:
    """`.control`을 패널 content에 넣는다. 드래그=회전, 휠=확대, 더블클릭=리셋."""

    def __init__(
        self,
        width: int = 480,
        height: int = 320,
        dummy_points: int = 0,
        show_robot: bool = True,
        scale: float = _DEFAULT_SCALE,
    ):
        self._width = width
        self._height = height
        self._default_scale = scale  # 더블클릭 리셋 때 이 인스턴스의 초기값으로 되돌아가게
        # 홈/작업 화면 3D 지도가 같은 내용을 보여주기로 해서 지금은 둘 다
        # True로 쓴다. 나중에 다시 화면별로 다르게 할 수도 있어서 플래그
        # 자체는 남겨둔다 (MapView는 재사용되는 컴포넌트).
        self._show_robot = show_robot

        # 바닥판 120x64cm, 로봇 위치는 왼쪽 변에서 28cm(y) / 뒤쪽 변에서 33cm(x).
        # base_link(로봇=원점) 기준으로 계산한 값이 assets/scene_config.json에 있다.
        scene = load_scene_config()
        self._floor_cfg = scene.get("floor", {"size": [2.0, 2.0], "grid_step": 0.2})
        self._boxes = scene.get("boxes", [])

        self._cam = Camera(
            yaw=_DEFAULT_YAW,
            pitch=_DEFAULT_PITCH,
            scale=self._default_scale,
            center=(width / 2, height / 2),
            pivot=_scene_pivot(self._floor_cfg),
        )
        self._yaw0 = self._cam.yaw
        self._pitch0 = self._cam.pitch
        self._pivot0 = self._cam.pivot  # 더블클릭 리셋용 원래 피벗
        self._right_pan_last: tuple[float, float] | None = None  # 직전 오른쪽 드래그 위치(px)

        # 드래그 중에 폴링(update())이 끼어들면 안 된다 — 로봇 팔이 실제로
        # 움직이기 시작하면서(back_ui가 진짜 관절값을 보내기 시작한 뒤,
        # 2026-08-05) 폴링마다 오는 값이 매번 달라져 _redraw()가 드래그
        # 프레임 사이사이에 끼어드는 게 체감될 정도가 됐다. 드래그 중엔
        # update()가 데이터만 받아두고 그리는 건 미뤘다가, 드래그가 끝나면
        # 한 번에 반영한다.
        self._dragging = False
        self._pending_redraw = False

        # 구현 순서 1~3 확인용 더미 점. zones/objects가 실제로 붙으면(4~6단계) 없앤다.
        if dummy_points:
            rng = np.random.default_rng(0)
            self._dummy_points = (rng.random((dummy_points, 3)) - 0.5) * 1.6
        else:
            self._dummy_points = None

        # /state의 zones/objects. update()가 폴링마다 갈아끼운다 — 그래도
        # 카메라 조작 상태(yaw/pitch/scale)는 이 인스턴스 자체가 안 바뀌니 유지된다.
        self._zones: list = []
        self._current_zone_id: str | None = None
        self._objects: list = []
        self._robot_links: list = []

        self._frame_label = ft.Text(
            "", size=t.SIZE_LABEL, color=t.TEXT_FAINT, font_family=t.MONO
        )
        self._canvas = cv.Canvas(width=width, height=height, shapes=[])
        self._redraw()

        self.control = ft.GestureDetector(
            content=ft.Stack(
                width=width,
                height=height,
                controls=[
                    self._canvas,
                    ft.Container(top=4, left=6, content=self._frame_label),
                ],
            ),
            drag_interval=16,  # ~60fps. 조작 중 지연이 느껴지면 안 된다
            mouse_cursor=ft.MouseCursor.MOVE,
            on_pan_start=self._on_pan_start,
            on_pan_update=self._on_pan_update,
            on_pan_end=self._on_drag_end,
            on_scroll=self._on_scroll,
            on_double_tap=self._on_double_tap,
            # 가운데 휠 버튼 드래그는 Flet GestureDetector에 이벤트 자체가 없다
            # (왼쪽=on_pan_*, 오른쪽=on_right_pan_* 만 있음). 그래서 오른쪽 버튼
            # 드래그를 피벗 이동(패닝)으로 쓴다 — 왼쪽=회전, 오른쪽=이동, 휠=확대.
            on_right_pan_start=self._on_right_pan_start,
            on_right_pan_update=self._on_right_pan_update,
            on_right_pan_end=self._on_drag_end,
        )

    def _redraw(self):
        """shapes를 다시 계산해 Canvas에 반영하고, 걸린 시간을 라벨에 찍는다.

        측정 대상은 "numpy 투영 + 도형 리스트 구성"까지다 — 진짜 병목(Canvas에
        도형 수천 개를 넘기는 비용)은 이 함수가 끝난 뒤 클라이언트가 실제로
        그리는 단계라 여기선 못 잰다. 드래그해보면서 눈으로 끊기는지 확인해야
        하는 이유다.
        """
        # 그리는 순서(뒤->앞): 그리드 -> 고정 배경 상자 -> zones -> 로봇 팔
        # -> objects 마커 -> 좌표축 -> 라벨. 라벨은 점에 가리지 않게 맨 끝에
        # 한 번 더 돌면서 그린다.
        t0 = time.perf_counter()
        shapes: list = []
        draw_grid(shapes, self._floor_cfg, self._cam)
        for box in self._boxes:
            draw_box_wire(shapes, box["pos"], box["size"], self._cam, t.BORDER, width=1.5)
        for zone in self._zones:
            center, size, rpy = _zone_box_params(zone)
            is_current = zone.get("id") == self._current_zone_id
            if is_current:
                color, width = t.ACCENT, 2.5  # "지금 진행 중인 것"에만 붙이는 강조색
            else:
                key = L.ZONE_SEARCH_COLOR.get(zone.get("search_state"), "unknown")
                color, width = t.STATUS[key], 1.5
            draw_box_wire(shapes, center, size, self._cam, color, rpy=rpy, width=width)
        if self._dummy_points is not None:
            draw_points(shapes, self._dummy_points, self._cam, t.STATUS["searching"], radius=1.5)

        # 로봇 팔. robot.links 데이터가 없으면 그냥 아무것도
        # 안 그린다(에러 아님) — 데이터는 있는데 점군 파일(assets/robot/*.npy)이
        # 없을 때만(아직 tools/mesh_to_points.py를 안 돌렸을 때) 자리 표시
        # 박스로 대체한다.
        if self._show_robot and self._robot_links:
            robot_pts = build_robot_points(self._robot_links)
            if len(robot_pts):
                draw_points(shapes, robot_pts, self._cam, t.ROBOT, radius=1.3)
            elif not has_robot_data():
                robot_center = (0.0, 0.0, _ROBOT_BOX_SIZE[2] / 2)
                draw_box_wire(shapes, robot_center, _ROBOT_BOX_SIZE, self._cam, t.ROBOT, width=2.0)

        # objects 마커. pos가 없는 물체(아직 위치 불명)는 3D 공간에 놓을 좌표가
        # 없으니 건너뛴다 — "위치 불명"은 시스템 상태 패널의 숫자로만 보여준다.
        located = [obj for obj in self._objects if obj.get("pos")]
        for obj in located:
            color = t.STATUS.get(obj.get("status"), t.STATUS["unknown"])
            draw_marker(shapes, obj["pos"], self._cam, color)

        draw_axes(shapes, self._cam)

        for obj in located:
            color = t.STATUS.get(obj.get("status"), t.STATUS["unknown"])
            draw_label(shapes, obj["pos"], self._cam, obj.get("name") or obj.get("id", ""), color)

        elapsed_ms = (time.perf_counter() - t0) * 1000

        self._canvas.shapes = shapes
        self._frame_label.value = f"{elapsed_ms:.2f} ms · 도형 {len(shapes)}개"

    def update(
        self,
        zones: list | None,
        current_zone_id: str | None,
        objects: list | None = None,
        robot_links: list | None = None,
    ):
        """폴링마다 호출된다 (open_ratio가 계속 바뀌니 캐시하지 않는다).

        `.control`(GestureDetector/Canvas 객체)은 그대로 두고 shapes 내용만
        바꾼다 — 카메라 회전 상태와 위젯 트리를 유지해야 드래그 중에 다른
        패널이 갱신돼도 안 끊긴다 (ObjectViewer와 같은 이유).
        """
        self._zones = zones or []
        self._current_zone_id = current_zone_id
        self._objects = objects or []
        self._robot_links = robot_links or []

        if self._dragging:
            # 드래그 도중 폴링이 오면 여기서 안 그리고 미룬다 — 값은 이미
            # 위에서 최신으로 받아뒀으니 드래그가 끝나는 순간(_on_drag_end)
            # 한 번만 반영해도 데이터가 밀리지 않는다.
            self._pending_redraw = True
            return

        self._redraw()
        try:
            self.control.update()
        except RuntimeError:
            # HomeView 생성자가 초기값(None)으로 한 번 부르는데, 그 시점엔
            # 아직 page에 안 붙어있다(body.content로 넣기 전이라 .page 접근 자체가
            # 예외를 던짐 — Flet엔 "붙어있나"를 물어보는 안 던지는 API가 없다).
            # 실제로 화면에 붙은 뒤(폴링/드래그)에만 갱신하면 되니 무시한다.
            pass

    def _on_drag_end(self, e):
        """왼쪽/오른쪽 드래그 둘 다 여기로 온다 — 끝나고 나서 미뤄둔 폴링
        갱신이 있으면 그제서야 한 번 반영한다."""
        self._dragging = False
        if self._pending_redraw:
            self._pending_redraw = False
            self._redraw()
            try:
                self.control.update()
            except RuntimeError:
                pass

    def _on_pan_start(self, e: ft.DragStartEvent):
        self._dragging = True
        self._yaw0 = self._cam.yaw
        self._pitch0 = self._cam.pitch

    def _on_pan_update(self, e: ft.DragUpdateEvent):
        delta = e.global_delta
        if delta is None:
            return
        self._cam.yaw = self._yaw0 + delta.x * _DRAG_SENSITIVITY
        pitch = self._pitch0 - delta.y * _DRAG_SENSITIVITY
        self._cam.pitch = max(-_PITCH_LIMIT, min(_PITCH_LIMIT, pitch))
        self._redraw()
        self.control.update()

    def _on_scroll(self, e: ft.ScrollEvent):
        factor = 1.0 - e.scroll_delta.y * _SCROLL_SENSITIVITY
        new_scale = self._cam.scale * factor
        self._cam.scale = max(_SCALE_MIN, min(_SCALE_MAX, new_scale))
        self._redraw()
        self.control.update()

    def _on_double_tap(self, e):
        self._cam.yaw = _DEFAULT_YAW
        self._cam.pitch = _DEFAULT_PITCH
        self._cam.scale = self._default_scale
        self._cam.pivot = self._pivot0
        self._redraw()
        self.control.update()

    def _on_right_pan_start(self, e: ft.PointerEvent):
        self._dragging = True
        self._right_pan_last = (e.global_position.x, e.global_position.y)

    def _on_right_pan_update(self, e: ft.PointerEvent):
        """오른쪽 버튼 드래그 = 피벗 이동(패닝).

        `on_right_pan_update`는 (`on_pan_update`와 달리) 누적 delta를 안 주고
        절대 좌표만 준다 — 그래서 직전 위치를 직접 들고 있다가 매번 뺀다.

        화면 픽셀 이동량(dx_px, dy_px)을 그대로 피벗의 x,y(m)로 바꾸면 안 된다
        — 지금 회전각(yaw/pitch)에 따라 화면의 가로/세로가 세계 좌표의 x/y와
        비스듬히 물려있기 때문이다. `project()`가 pivot을 어떻게 화면에 놓는지
        (`rotated = R@P + (I-R)@pivot`, `projection.py` 참고) 역으로 풀어서,
        "마우스가 옮긴 만큼 화면도 정확히 그만큼 움직인다"를 만족하는 (dx,dy)를
        매번 2x2 연립방정식으로 구한다.
        """
        if self._right_pan_last is None:
            return
        gx, gy = e.global_position.x, e.global_position.y
        last_x, last_y = self._right_pan_last
        dx_px, dy_px = gx - last_x, gy - last_y
        self._right_pan_last = (gx, gy)
        if dx_px == 0 and dy_px == 0:
            return

        r = self._cam.matrix()
        m = np.eye(3) - r
        scale = self._cam.scale
        # x_px = center.x + scale*rotated.x, y_px = center.y - scale*rotated.z
        # rotated = R@P + M@pivot 이므로 pivot을 dx,dy(z는 고정) 만큼 바꿀 때
        # 화면이 움직이는 양은 아래 2x2 행렬로 정리된다.
        a = np.array(
            [
                [scale * m[0, 0], scale * m[0, 1]],
                [-scale * m[2, 0], -scale * m[2, 1]],
            ]
        )
        try:
            dx, dy = np.linalg.solve(a, np.array([dx_px, dy_px]))
        except np.linalg.LinAlgError:
            return  # 화면과 딱 평행하게 보는 각도라 못 풀 때(드묾) — 그냥 무시

        px, py, pz = self._cam.pivot
        self._cam.pivot = (px + dx, py + dy, pz)
        self._redraw()
        self.control.update()
