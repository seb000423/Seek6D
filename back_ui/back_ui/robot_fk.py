"""관절 각도(joint_1~6) -> 각 링크의 base_link 기준 위치/자세.

`doosan-robot2/dsr_description2/urdf/m0609.white.urdf`의 조인트 원점을 그대로
옮겨 적었다(모델이 바뀌면 여기 `_JOINT_CHAIN`도 같이 바꿔야 한다 —
front_ui/tools/mesh_to_points.py의 LINKS와 짝이 맞아야 함). 6개 조인트 전부
회전축이 자기 로컬 z축(URDF의 `<axis xyz="0 0 1"/>`)이라, 각 관절의 변환은
"고정 원점 변환 다음에 z축으로 각도만큼 회전"으로 계산된다 — 진짜 로봇마다
다른 축을 쓰면 이 가정이 깨지니 그때는 축도 같이 저장해야 한다.

front_ui는 이 모듈을 안 가져온다(패키지가 다르고 conda/시스템 파이썬
환경도 다르다) — 회전 표현(R = Rz·Ry·Rx)만
`render3d/shapes.py`의 `rpy_matrix()`와 맞춰서, 여기서 만든 rpy를 front_ui가
그대로 다시 그릴 수 있게 했다.
"""

import numpy as np

# (링크 이름, 조인트 원점 rpy, 조인트 원점 xyz) — URDF에 나온 순서 그대로.
_JOINT_CHAIN = [
    ("link_1", (0.0, 0.0, 0.0), (0.0, 0.0, 0.1345)),
    ("link_2", (0.0, -1.571, -1.571), (0.0, 0.006, 0.0)),
    ("link_3", (0.0, 0.0, 1.571), (0.411, 0.0, 0.0)),
    ("link_4", (1.571, 0.0, 0.0), (0.0, -0.368, 0.0)),
    ("link_5", (-1.571, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("link_6", (1.571, 0.0, 0.0), (0.0, -0.121, 0.0)),
]


def _rpy_matrix(rpy) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _rz(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _extract_rpy(r: np.ndarray) -> tuple[float, float, float]:
    """`_rpy_matrix`의 역연산. R = Rz(yaw)·Ry(pitch)·Rx(roll) 가정."""
    pitch = float(np.arcsin(np.clip(-r[2, 0], -1.0, 1.0)))
    roll = float(np.arctan2(r[2, 1], r[2, 2]))
    yaw = float(np.arctan2(r[1, 0], r[0, 0]))
    return roll, pitch, yaw


def compute_links(joint_angles: dict) -> list[dict]:
    """`{"joint_1": 0.3, ...}` (라디안) -> front_ui `robot.links` 스키마.

    없는 관절은 0도로 취급한다 — JointState가 관절 일부만 담고 있어도
    (그리퍼 관절이 섞여 있거나 순서가 달라도) 죽지 않는다.
    """
    links = [{"name": "base_link", "pos": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]}]

    cum = np.eye(4)
    for link_name, origin_rpy, origin_xyz in _JOINT_CHAIN:
        joint_name = "joint_" + link_name.split("_")[1]
        angle = joint_angles.get(joint_name, 0.0)

        origin = np.eye(4)
        origin[:3, :3] = _rpy_matrix(origin_rpy)
        origin[:3, 3] = origin_xyz

        rot = np.eye(4)
        rot[:3, :3] = _rz(angle)

        cum = cum @ origin @ rot

        links.append(
            {
                "name": link_name,
                "pos": cum[:3, 3].tolist(),
                "rpy": list(_extract_rpy(cum[:3, :3])),
            }
        )

    return links
