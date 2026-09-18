"""3D 좌표 -> 2D 화면 좌표 투영.

표현 방식(와이어프레임이냐 점이냐)이 바뀌어도 이 파일은 변하지 않는다 —
여긴 순수 좌표 계산만 한다.

base_link 축(x 전방, y 좌측, z 상방)을 화면에 그리려면, 화면 시점에 맞는
회전을 씌운 뒤 화면 평면(x_px, y_px)으로 정사영한다. 원근투영은 안 쓴다
(작업 공간이 1~2m라 차이가 없고, 직교가 좌표 읽기에 더 유리하다).
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Camera:
    """시점. yaw/pitch/scale을 바꾸면 화면이 회전·확대된다."""

    yaw: float = -0.6  # 라디안. z축 기준 좌우 회전
    pitch: float = 0.5  # 라디안. 위에서 내려다보는 각도. 0이면 정면, 커질수록 위에서
    scale: float = 300.0  # 픽셀 / 미터
    center: tuple[float, float] = (0.0, 0.0)  # 화면 중심 픽셀 좌표
    pivot: tuple[float, float, float] = (0.0, 0.0, 0.0)  # 회전축(base_link 좌표)
    # 원점(로봇)은 장면 한쪽 구석이라, 원점 기준으로 돌리면 선반/수납장/박스가
    # 화면 밖으로 휙 나갔다 들어온다. 장면 중앙 쯤을 pivot으로 주면 그 자리에
    # 고정된 채로 도는 것처럼 보여서 보기 편하다.

    def matrix(self) -> np.ndarray:
        """yaw(z축 회전) 다음 pitch(x축 회전) 순서로 곱한 3x3 회전행렬."""
        cy, sy = np.cos(self.yaw), np.sin(self.yaw)
        cp, sp = np.cos(self.pitch), np.sin(self.pitch)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cp, -sp], [0.0, sp, cp]])
        return rx @ rz


def project(points_3d: np.ndarray, cam: Camera) -> np.ndarray:
    """(N,3) base_link 좌표 -> (N,3) [x_px, y_px, depth]

    depth는 정렬용이며 값이 클수록 뒤쪽이다 (painter's algorithm: depth 내림차순
    정렬 후 뒤에서부터 그리면 앞이 뒤를 가린다).

    반드시 벡터화되어 있다 — `points @ R.T` 행렬곱 한 번으로 N개를 한꺼번에
    처리한다. 점 하나씩 도는 반복문은 링크 점 수천 개에서 느려진다.
    """
    pts = np.asarray(points_3d, dtype=float).reshape(-1, 3)
    pivot = np.asarray(cam.pivot, dtype=float)
    r = cam.matrix()
    # pivot을 원점으로 옮겨서 돌린 뒤 도로 더한다 — pivot 자기 자신은
    # (pts-pivot)이 0이라 회전과 무관하게 항상 같은 자리에 투영된다.
    rotated = (pts - pivot) @ r.T + pivot  # (N,3)

    x_px = cam.center[0] + rotated[:, 0] * cam.scale
    y_px = cam.center[1] - rotated[:, 2] * cam.scale  # 화면 y는 아래로 증가 -> z 뒤집기
    depth = rotated[:, 1]

    return np.stack([x_px, y_px, depth], axis=1)
