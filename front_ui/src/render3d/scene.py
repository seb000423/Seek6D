"""고정 배경(scene_config.json) 로딩.

바닥·책상·벽처럼 로봇도 모르고 /state로도 안 오는, UI에서만 쓰는 고정 배경이다.
"""

import json
from pathlib import Path

_CONFIG_PATH = Path(__file__).resolve().parent.parent / "assets" / "scene_config.json"

_FALLBACK = {
    "floor": {"size": [2.0, 2.0], "pos": [0.0, 0.0], "grid_step": 0.2},
    "boxes": [],
}


def load_scene_config() -> dict:
    """파일이 없거나 깨져 있어도 앱이 죽지 않게 기본 배경으로 대체한다."""
    try:
        with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return _FALLBACK
