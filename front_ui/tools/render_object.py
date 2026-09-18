"""OBJ -> PNG 오프라인 사전 렌더 도구.

"찾고 있는 3d 모델" 패널의 물체 이미지 표현용. **front_ui 런타임에는 포함되지 않는다.**
front_ui 의존성(flet/numpy/requests)과 무관하게, 이 스크립트를 돌릴 때만
matplotlib·Pillow가 있으면 된다 (시스템 파이썬 등 아무 환경에서나 1회 실행).

결과물 `src/assets/renders/{object_id}.png` 만 front_ui가 읽는다.

실행:
    python3 tools/render_object.py src/assets/models/frog_fixed.obj cup_red_01
    (obj 경로, 저장할 object_id 순서. object_id는 fake_server/back_ui가
     주는 task.target_id 와 같아야 홈 화면 "찾고 있는 3d 모델" 패널이 찾는다.)

참고: 2026-08-05 결정으로 "찾고 있는 3d 모델" 패널은 이제 정적 PNG 대신
components/object_viewer.py 의 실시간 드래그 회전 뷰어를 쓴다. 이 스크립트는
회전 뷰어를 못 쓰는 물체(OBJ가 없거나 너무 무거운 경우)용 폴백 경로로 남겨둔다.
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from PIL import Image  # noqa: E402

# mpl_toolkits.mplot3d(Axes3D)는 이 개발 환경에 깔린 두 matplotlib 버전이
# 충돌해 import가 깨진다. render3d/projection.py 에 갈 것과 같은 방식으로
# numpy로 직접 투영 + painter's algorithm을 써서 그 의존성을 피한다.
RENDERS_DIR = Path(__file__).resolve().parent.parent / "src" / "assets" / "renders"
OUTPUT_SIZE = 256
BASE_COLOR = np.array([0.36, 0.66, 0.38])  # 텍스처가 없는 mesh용 기본 색
LIGHT_DIR = np.array([0.4, 0.55, 0.75])
LIGHT_DIR = LIGHT_DIR / np.linalg.norm(LIGHT_DIR)
ELEV_DEG = 18
AZIM_DEG = -60


def _rotation_matrix(elev_deg: float, azim_deg: float) -> np.ndarray:
    e, a = np.radians(elev_deg), np.radians(azim_deg)
    rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]])
    return rx @ rz


def load_obj(path: Path):
    """v / f 라인만 읽는 최소 OBJ 파서. vt/vn, mtl은 무시한다."""
    vertices = []
    faces = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if line.startswith("v "):
                vertices.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("f "):
                idx = []
                for token in line.split()[1:]:
                    vi = int(token.split("/")[0])
                    idx.append(vi - 1 if vi > 0 else vi)  # OBJ는 1-based
                for i in range(1, len(idx) - 1):  # fan triangulation
                    faces.append((idx[0], idx[i], idx[i + 1]))
    return np.array(vertices, dtype=float), faces


def render(obj_path: Path, out_path: Path):
    vertices, faces = load_obj(obj_path)
    center = (vertices.max(axis=0) + vertices.min(axis=0)) / 2
    vertices = vertices - center
    scale = np.abs(vertices).max()
    vertices = vertices / scale

    tris = vertices[np.array(faces)]  # (F, 3, 3) object-space

    # 셰이딩은 회전 전 object-space 노멀 vs 고정 광원으로 계산한다.
    # (내적은 회전에 불변이라 광원도 같이 돌릴 필요가 없다)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths == 0] = 1
    normals = normals / lengths
    shade = np.clip(normals @ LIGHT_DIR, 0.2, 1.0)
    colors = BASE_COLOR[None, :] * shade[:, None]

    # 3D 지도와 같은 투영 방식(고정 회전행렬 R + 직교투영)을 그대로 적용한다.
    r = _rotation_matrix(ELEV_DEG, AZIM_DEG)
    rotated = tris @ r.T  # (F, 3, 3)
    xy = rotated[:, :, [0, 2]]  # x2d, y2d
    depth = rotated[:, :, 1].mean(axis=1)  # painter's algorithm 정렬 기준

    order = np.argsort(depth)  # 먼 것부터 그려서 가까운 삼각형이 위를 덮는다
    xy, colors = xy[order], colors[order]

    fig = plt.figure(figsize=(4, 4), dpi=OUTPUT_SIZE // 4)
    fig.patch.set_alpha(0)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_facecolor((0, 0, 0, 0))
    ax.add_collection(PolyCollection(xy, facecolors=colors, edgecolors="none"))
    lim = 1.05
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.set_axis_off()

    fig.canvas.draw()
    raw = np.asarray(fig.canvas.buffer_rgba())
    plt.close(fig)

    img = Image.fromarray(raw, mode="RGBA")
    alpha = img.getchannel("A")
    bbox = alpha.getbbox()
    if bbox:
        img = img.crop(bbox)

    canvas = Image.new("RGBA", (OUTPUT_SIZE, OUTPUT_SIZE), (0, 0, 0, 0))
    img.thumbnail((int(OUTPUT_SIZE * 0.9), int(OUTPUT_SIZE * 0.9)))
    paste_at = ((OUTPUT_SIZE - img.width) // 2, (OUTPUT_SIZE - img.height) // 2)
    canvas.paste(img, paste_at, img)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)
    print(f"saved {out_path} ({canvas.width}x{canvas.height})")


def main():
    if len(sys.argv) != 3:
        print("usage: render_object.py <obj_path> <object_id>")
        raise SystemExit(1)
    obj_path = Path(sys.argv[1])
    object_id = sys.argv[2]
    render(obj_path, RENDERS_DIR / f"{object_id}.png")


if __name__ == "__main__":
    main()
