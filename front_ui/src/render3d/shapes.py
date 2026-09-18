"""3D 지도의 도형 그리기 함수들.

전부 `projection.project()`가 계산한 화면 좌표를 `flet.canvas` 도형으로
바꾸는 역할만 한다 — 좌표 계산 자체는 여기 없다.

4단계: draw_box_wire (책상/벽/선반처럼 고정된 상자). draw_marker(물체 라벨)는
6단계에서 추가한다.
"""

import flet as ft
import flet.canvas as cv
import numpy as np

import theme as t
from render3d.projection import Camera, project

# 큐브 8개 꼭짓점의 부호 조합. sx>>2, sy>>1, sz 순서 비트로 인덱스가 매겨진다
# (아래 _BOX_EDGES가 이 순서를 전제로 한다).
_BOX_SIGNS = np.array(
    [[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float
)
# 꼭짓점 인덱스가 1비트만 다른 쌍 = 큐브의 모서리 12개.
_BOX_EDGES = [
    (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
    (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
]


def rpy_matrix(rpy) -> np.ndarray:
    """roll(x)-pitch(y)-yaw(z) 오일러각 -> 3x3 회전행렬. XYZ 순서로 곱한다."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def draw_grid(shapes: list, floor_cfg: dict, cam: Camera) -> None:
    """바닥 그리드. 로봇(원점)을 지나는 두 선만 조금 밝게 해서 축 위치를 알려준다.

    floor_cfg["pos"]로 바닥판을 원점에서 벗어난 곳에 둘 수 있다 — 로봇이
    책상 한가운데가 아니라 구석에 놓인 경우 등. 없으면 원점 중심(기존 방식).
    """
    size = floor_cfg.get("size", [2.0, 2.0])
    pos = floor_cfg.get("pos", [0.0, 0.0])
    step = floor_cfg.get("grid_step", 0.2)
    half_x, half_y = size[0] / 2, size[1] / 2
    cx, cy = pos[0], pos[1]

    xs = np.arange(cx - half_x, cx + half_x + 1e-6, step)
    ys = np.arange(cy - half_y, cy + half_y + 1e-6, step)

    normal_paint = ft.Paint(color=t.BORDER, stroke_width=1)
    axis_paint = ft.Paint(color=t.TEXT_FAINT, stroke_width=1.5)

    # x 방향으로 뻗는 선들 (y를 고정하고 x 양 끝을 잇는다)
    for y in ys:
        p1 = project(np.array([[cx - half_x, y, 0.0]]), cam)[0]
        p2 = project(np.array([[cx + half_x, y, 0.0]]), cam)[0]
        shapes.append(cv.Line(p1[0], p1[1], p2[0], p2[1], paint=normal_paint))

    # y 방향으로 뻗는 선들
    for x in xs:
        p1 = project(np.array([[x, cy - half_y, 0.0]]), cam)[0]
        p2 = project(np.array([[x, cy + half_y, 0.0]]), cam)[0]
        shapes.append(cv.Line(p1[0], p1[1], p2[0], p2[1], paint=normal_paint))

    # 로봇이 지나는 x=0/y=0 선은 grid_step에 안 맞아떨어질 수 있어서
    # (바닥판이 원점 중심이 아니면 흔하다) 격자와 별개로 직접 그린다.
    if cx - half_x <= 0.0 <= cx + half_x:
        p1 = project(np.array([[0.0, cy - half_y, 0.0]]), cam)[0]
        p2 = project(np.array([[0.0, cy + half_y, 0.0]]), cam)[0]
        shapes.append(cv.Line(p1[0], p1[1], p2[0], p2[1], paint=axis_paint))
    if cy - half_y <= 0.0 <= cy + half_y:
        p1 = project(np.array([[cx - half_x, 0.0, 0.0]]), cam)[0]
        p2 = project(np.array([[cx + half_x, 0.0, 0.0]]), cam)[0]
        shapes.append(cv.Line(p1[0], p1[1], p2[0], p2[1], paint=axis_paint))


def draw_axes(shapes: list, cam: Camera, length: float = 0.10) -> None:
    """원점에 x(빨강)/y(초록)/z(파랑) 축. 로봇(base_link) 위치 표시 역할."""
    origin = project(np.array([[0.0, 0.0, 0.0]]), cam)[0]
    axes = [
        ((length, 0, 0), "#D9564A"),  # x - 빨강
        ((0, length, 0), "#3FB27F"),  # y - 초록
        ((0, 0, length), "#4A90D9"),  # z - 파랑
    ]
    for tip_3d, color in axes:
        tip = project(np.array([tip_3d]), cam)[0]
        shapes.append(
            cv.Line(origin[0], origin[1], tip[0], tip[1], paint=ft.Paint(color=color, stroke_width=2))
        )


def draw_box_wire(
    shapes: list,
    center,
    size,
    cam: Camera,
    color: str,
    rpy=None,
    width: float = 1.0,
) -> None:
    """직육면체를 와이어프레임으로. 꼭짓점 8개를 투영하고 모서리 12개를 잇는다.

    채운 면 대신 와이어프레임을 쓰는 이유: 채우면 서랍 안 물체나
    로봇 팔이 가려지고, painter's algorithm 특성상 팔이 상자를 관통할 때
    앞뒤가 틀리게 나온다. 와이어프레임은 두 문제가 동시에 없어진다.
    """
    center = np.asarray(center, dtype=float)
    half = np.asarray(size, dtype=float) / 2
    local = _BOX_SIGNS * half  # (8,3)
    if rpy is not None:
        local = local @ rpy_matrix(rpy).T

    verts = local + center
    proj = project(verts, cam)  # (8,3) [x,y,depth]

    paint = ft.Paint(color=color, stroke_width=width)
    for a, b in _BOX_EDGES:
        shapes.append(
            cv.Line(proj[a, 0], proj[a, 1], proj[b, 0], proj[b, 1], paint=paint)
        )


def draw_robot_marker(shapes: list, cam: Camera, color: str = t.ROBOT, radius: float = 7.0) -> None:
    """로봇 위치(=원점, base_link) 표시. 링크 포인트 클라우드가 붙기 전까지는
    이 큰 점 하나가 로봇 자리다."""
    origin = project(np.array([[0.0, 0.0, 0.0]]), cam)[0]
    shapes.append(cv.Circle(float(origin[0]), float(origin[1]), radius, paint=ft.Paint(color=color)))


_MARKER_MIN_R, _MARKER_MAX_R = 3.0, 6.0
# 이 정도 깊이 차이(m)면 반지름이 최소~최대로 다 벌어지게. 씬 크기(바닥판
# 대각선 ~1.3m)에 맞춘 임의 값이라 정확한 눈금은 아니고, "가까울수록 커
# 보인다" 정도의 원근감 힌트만 준다.
_MARKER_DEPTH_SPAN = 1.0


def draw_marker(shapes: list, pos, cam: Camera, color: str) -> None:
    """물체 위치 점 하나. 라벨은 별도(draw_label) — 라벨은 맨 마지막에
    한 번 더 돌면서 그린다(점에 가려지지 않게)."""
    proj = project(np.array([pos], dtype=float), cam)[0]
    depth = proj[2]
    t_norm = np.clip(0.5 - depth / _MARKER_DEPTH_SPAN, 0.0, 1.0)  # 가까울수록(depth 작을수록) 1에 가깝게
    radius = _MARKER_MIN_R + t_norm * (_MARKER_MAX_R - _MARKER_MIN_R)
    shapes.append(cv.Circle(float(proj[0]), float(proj[1]), radius, paint=ft.Paint(color=color)))


def draw_label(shapes: list, pos, cam: Camera, text: str, color: str) -> None:
    """물체 이름 라벨. 점 옆에 살짝 띄워서 겹치지 않게."""
    proj = project(np.array([pos], dtype=float), cam)[0]
    shapes.append(
        cv.Text(
            float(proj[0]) + 8,
            float(proj[1]) - 6,
            text,
            style=ft.TextStyle(size=11, color=color),
        )
    )


def draw_points(shapes: list, points_3d: np.ndarray, cam: Camera, color: str, radius: float = 1.2) -> None:
    """포인트 클라우드. 점 하나당 cv.Circle 하나씩 만들면(수천 개) 서버 쪽 객체
    생성 비용도 크고, 클라이언트가 그걸 하나하나 따로 그려야 해서 훨씬 느리다.
    실제로 드래그 중 앱이 멈추는 원인이었다 — cv.Points 하나에 좌표를 전부
    묶어서 한 번에 넘긴다 (Flutter의 batched drawPoints 호출 하나가 된다).

    같은 Paint(같은 색·크기)로 한 번에 그리는 거라 depth 정렬은 의미가 없어서
    (겹쳐도 안 보이는 차이) 생략한다 — 그만큼 더 가볍다.
    """
    points_3d = np.asarray(points_3d, dtype=float).reshape(-1, 3)
    if len(points_3d) == 0:
        return
    proj = project(points_3d, cam)
    offsets = list(zip(proj[:, 0].tolist(), proj[:, 1].tolist()))
    paint = ft.Paint(color=color, stroke_width=radius * 2, stroke_cap=ft.StrokeCap.ROUND)
    shapes.append(cv.Points(points=offsets, point_mode=cv.PointMode.POINTS, paint=paint))
