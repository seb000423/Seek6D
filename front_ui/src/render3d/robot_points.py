"""로봇 팔 점군 로딩 + `/state`의 robot.links로 배치.

mesh 파일 자체는 여기서 안 읽는다
(런타임 3D 라이브러리 금지) — `tools/mesh_to_points.py`가 미리 만들어둔
`assets/robot/<link_name>.npy`(링크 로컬 좌표, 미터)만 numpy로 읽는다.

각 링크의 실제 위치(pos)·자세(rpy)는 이 모듈이 모른다 — back_ui가 TF/FK로
계산해서 `/state`의 `robot.links`로 이미 base_link 기준 값을 보내준다
(front_ui는 TF를 다루지 않는다). 여기선 로컬 점군을 그 값으로
회전·이동시켜 하나로 합치기만 한다.
"""

from pathlib import Path

import numpy as np

from render3d.shapes import rpy_matrix

_ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets" / "robot"

_cache: dict[str, np.ndarray] | None = None

# base_link의 "정면" 축이 URDF/mesh 좌표계와 우리 프로젝트 축 규약(x 전방)
# 사이에서 90도 어긋나 있어서(관절 각도와 무관한 고정 오차 — base_link 자체가
# 어느 쪽을 보는지의 문제라 팔이 움직여도 안 변한다) 매번 이 값만큼 보정한다.
# 위에서 내려다봤을 때 시계방향 90도 = z축 기준 -90도 회전(오른손 법칙:
# +z 회전은 위에서 보면 반시계).
_ORIENTATION_FIX = rpy_matrix((0.0, 0.0, -np.pi / 2))


def _load_all() -> dict[str, np.ndarray]:
    """링크별 .npy를 처음 쓸 때 한 번만 읽어서 캐시한다 (폴링마다 디스크
    안 읽으려고 — object_viewer.py가 OBJ를 한 번만 읽는 것과 같은 이유)."""
    global _cache
    if _cache is None:
        _cache = {}
        if _ASSETS_DIR.exists():
            for path in _ASSETS_DIR.glob("*.npy"):
                _cache[path.stem] = np.load(path)
    return _cache


def has_data() -> bool:
    """`tools/mesh_to_points.py`를 아직 안 돌려서 .npy가 하나도 없는 경우
    map_view.py가 자리 표시 박스로 대체할지 판단하는 데 쓴다."""
    return bool(_load_all())


def build_robot_points(links: list[dict] | None) -> np.ndarray:
    """robot.links를 받아 링크별 로컬 점군을 pos/rpy로 옮긴 뒤 전부 합친다.

    링크 하나마다 따로 그리지 않고 (N,3) 배열 하나로 합쳐서 돌려주는 이유:
    draw_points()가 cv.Points 하나로 통째로 넘겨야 빠르다 — 링크 6개를
    따로 그리면 도형이 6배로 늘어 그만큼 렉의 원인이 된다.

    `links`가 없거나 비어 있으면 빈 배열을 돌려준다 — robot이 없거나
    비어 있으면 팔을 그리지 않는다. 오류가 아니다.
    """
    cache = _load_all()
    if not links or not cache:
        return np.empty((0, 3))

    parts = []
    for link in links:
        local = cache.get(link.get("name"))
        if local is None:
            continue
        pos = np.array(link.get("pos", [0.0, 0.0, 0.0]), dtype=float)
        rpy = link.get("rpy")
        world = local @ rpy_matrix(rpy).T + pos if rpy else local + pos
        parts.append(world)

    if not parts:
        return np.empty((0, 3))

    combined = np.vstack(parts)
    # base_link(원점) 기준 고정 보정이라 링크별이 아니라 다 합친 뒤 한 번만
    # 돌린다 — pos/rpy가 나중에 실제 FK 값으로 바뀌어도(팔이 움직여도) 이
    # 보정 자체는 그대로 유효하다.
    return combined @ _ORIENTATION_FIX.T
