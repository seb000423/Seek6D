"""로봇 팔 mesh(.dae) -> 링크별 점군(.npy) 오프라인 변환.

front_ui 런타임은 mesh를 직접 읽지
않는다(런타임 3D 라이브러리 의존성 제약). 이 스크립트를
미리 한 번 돌려서 front_ui/src/assets/robot/<link_name>.npy 를 만들어두면,
런타임은 그 .npy만 numpy로 읽는다.

원본 mesh: doosan-robot2/dsr_description2/meshes/<MODEL>_white/*.dae
(COLLADA). 사용 모델은 M0609. 다른 모델로 바꾸려면 MODEL/LINKS만
고치면 된다 — URDF(xacro/../urdf/<model>.white.urdf)에서 링크별 mesh 파일
목록과 순서를 그대로 옮겨적은 것이다.

vertex(꼭짓점) 좌표를 그대로 점군으로 쓴다. 삼각형 인덱스(<triangles>)는 안
읽는다 — "면을 채워서 그린다"가 아니라 "표면 위에 점을 뿌린다"가 목적이라
꼭짓점 자체가 이미 표면 위의 샘플이고, 그걸로 충분하다 (object_viewer.py가
OBJ face-centroid를 점으로 쓴 것과 같은 발상).

실행 (시스템 파이썬, ROS/conda 무관 — numpy만 있으면 된다):
    python tools/mesh_to_points.py
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

MODEL = "m0609"
MESH_DIR = (
    Path.home()
    / "cobot_ws" / "src" / "doosan-robot2" / "dsr_description2" / "meshes" / f"{MODEL}_white"
)
OUT_DIR = Path(__file__).resolve().parent.parent / "src" / "assets" / "robot"

# URDF(<model>.white.urdf)의 <link>/<visual>/<mesh filename=...> 순서 그대로.
# 링크 하나가 sub-mesh 여러 개로 나뉘어 있으면(link_2, link_4) 합쳐서 하나의
# 점군으로 만든다. base_link는 로봇 받침대 mesh다.
LINKS = {
    "base_link": ["MF0609_0_0"],
    "link_1": ["MF0609_1_0"],
    "link_2": ["MF0609_2_0", "MF0609_2_1", "MF0609_2_2"],
    "link_3": ["MF0609_3_0"],
    "link_4": ["MF0609_4_0", "MF0609_4_1"],
    "link_5": ["MF0609_5_0"],
    "link_6": ["MF0609_6_0"],
}

SCALE = 0.001  # URDF의 <mesh scale="0.001 0.001 0.001"/> — mesh 파일 자체는 mm 단위
# 5000으로 처음 만들었는데, 화면 전환(다른 화면 -> 홈)할 때 이 점군 전체를
# 새로 붙이는 비용이 묵직하게 느껴진다는 보고가 있어서(2026-08-05) 줄였다.
# draw_points()가 한 도형으로 묶어서 보내긴 하지만, 좌표 개수 자체(x,y 5000쌍)는
# 그대로 전송/파싱 비용이라 화면이 작은 미리보기 패널 용도로는 이 정도로도
# 로봇 팔 형태를 알아보는 데 충분하다.
TOTAL_POINT_BUDGET = 1500
MIN_POINTS_PER_LINK = 60  # 작은 링크도 형태는 알아볼 수 있게 최소치를 둔다

_COLLADA_NS = "http://www.collada.org/2005/11/COLLADASchema"


def _tag(name: str) -> str:
    return f"{{{_COLLADA_NS}}}{name}"


def _parse_dae(path: Path) -> np.ndarray:
    """geometry의 꼭짓점 좌표를, 그 mesh를 배치하는 node의 변환행렬까지
    적용해서 돌려준다 (파일 자체 좌표는 임의 원점일 수 있어서 node의
    <matrix>로 그 mesh 인스턴스가 실제로 놓이는 자리를 맞춰야 한다).
    """
    root = ET.parse(path).getroot()

    geometry = root.find(f".//{_tag('library_geometries')}/{_tag('geometry')}")
    positions_src = geometry.find(f".//{_tag('source')}")
    # source가 여러 개(POSITION/NORMAL)일 수 있어 id에 "-positions"가 붙은 것만 쓴다.
    for src in geometry.findall(f".//{_tag('source')}"):
        if src.get("id", "").endswith("-positions"):
            positions_src = src
            break
    float_array = positions_src.find(_tag("float_array"))
    values = np.array(float_array.text.split(), dtype=float)
    positions = values.reshape(-1, 3)

    geom_id = geometry.get("id")
    matrix = np.eye(4)
    for node in root.findall(f".//{_tag('library_visual_scenes')}//{_tag('node')}"):
        instance = node.find(_tag("instance_geometry"))
        if instance is not None and instance.get("url") == f"#{geom_id}":
            m_el = node.find(_tag("matrix"))
            if m_el is not None:
                matrix = np.array(m_el.text.split(), dtype=float).reshape(4, 4)
            break

    homo = np.hstack([positions, np.ones((len(positions), 1))])
    transformed = (matrix @ homo.T).T[:, :3]
    return transformed * SCALE


def _subsample(points: np.ndarray, target: int) -> np.ndarray:
    if len(points) <= target:
        return points
    rng = np.random.default_rng(0)  # 매번 같은 결과 나오게 고정 시드
    idx = rng.choice(len(points), size=target, replace=False)
    return points[idx]


def main():
    if not MESH_DIR.exists():
        raise SystemExit(f"mesh 폴더를 못 찾음: {MESH_DIR}")

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    raw_per_link = {}
    for link_name, mesh_names in LINKS.items():
        parts = [_parse_dae(MESH_DIR / f"{name}.dae") for name in mesh_names]
        raw_per_link[link_name] = np.vstack(parts)

    total_raw = sum(len(p) for p in raw_per_link.values())
    print(f"원본 꼭짓점 합계: {total_raw}개 -> 목표 {TOTAL_POINT_BUDGET}개로 축소")

    for link_name, points in raw_per_link.items():
        # 링크 크기(꼭짓점 수)에 비례 배분하되 최소치는 보장한다.
        share = int(TOTAL_POINT_BUDGET * len(points) / total_raw)
        target = max(MIN_POINTS_PER_LINK, share)
        sampled = _subsample(points, target).astype(np.float32)
        out_path = OUT_DIR / f"{link_name}.npy"
        np.save(out_path, sampled)
        print(f"  {link_name}: {len(points)} -> {len(sampled)}점  ({out_path.name})")


if __name__ == "__main__":
    main()
