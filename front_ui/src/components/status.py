"""상태를 색 점 + 글자로 보여 주는 작은 컴포넌트들.

노드 생존, 물체 상태, 구역 상태, 시스템 상태, 연결 여부가 전부
같은 모양이라 여기 모아 둔다.
"""

import flet as ft

import theme as t
import labels as L


def dot(color: str, size: int = 7):
    """색 점 하나."""
    return ft.Container(
        width=size,
        height=size,
        bgcolor=color,
        border_radius=size / 2,
    )


def status_text(text: str, color: str, size: int | None = None, bold: bool = False):
    return ft.Text(
        text,
        size=size or t.SIZE_BODY,
        color=color,
        weight=ft.FontWeight.W_600 if bold else ft.FontWeight.NORMAL,
    )


def badge(text: str, color: str, size: int | None = None, bold: bool = False):
    """색 점 + 글자. 이 파일의 다른 함수들이 전부 이걸 쓴다."""
    return ft.Row(
        spacing=6,
        tight=True,
        controls=[dot(color), status_text(text, color, size, bold)],
    )


# ── 용도별 배지 ──────────────────────────────────────────────


def object_status(status: str | None):
    """물체 상태. unknown / searching / confirmed / held / warning / error"""
    color = t.STATUS.get(status, t.TEXT_FAINT)
    return badge(L.get(L.OBJECT_STATUS, status), color)


def zone_search_status(search_state: str | None):
    """구역 탐색 상태. untouched / observing / done / found / failed"""
    key = L.ZONE_SEARCH_COLOR.get(search_state, "unknown")
    return badge(L.get(L.ZONE_SEARCH_STATE, search_state), t.STATUS[key])


def task_status(status: str | None):
    """작업 결과. RUNNING / SUCCESS / FAILED / CANCELED / ERROR"""
    key = L.TASK_STATUS_COLOR.get(status, "unknown")
    return badge(L.get(L.TASK_STATUS, status), t.STATUS[key])


def system_state(state: str | None):
    """시스템 상태. LOAD / IDLE / RUN. 홈 화면에서 크게 쓴다."""
    color = t.SYSTEM_STATE.get(state, t.TEXT_FAINT)
    return badge(L.get(L.SYSTEM_STATE, state), color, size=t.SIZE_VALUE, bold=True)


def online(ok: bool | None, ok_text: str = "정상", ng_text: str = "끊김"):
    """연결·생존 여부처럼 참/거짓만 있는 값.

    ok 가 None 이면 아직 값을 못 받은 상태로 본다.
    """
    if ok is None:
        return badge("확인 중", t.TEXT_FAINT)
    if ok:
        return badge(ok_text, t.STATUS["confirmed"])
    return badge(ng_text, t.STATUS["error"])


def node_list(nodes: dict | None):
    """노드별 생존 표시를 한 줄에 늘어놓는다.

    nodes 예: {"image": True, "main": True, "db": False, ...}
    """
    nodes = nodes or {}
    items = []
    for key, name in L.NODE.items():
        ok = nodes.get(key)
        color = (
            t.TEXT_FAINT if ok is None
            else t.STATUS["confirmed"] if ok
            else t.STATUS["error"]
        )
        items.append(
            ft.Row(
                spacing=5,
                tight=True,
                controls=[dot(color, 6), ft.Text(name, size=t.SIZE_LABEL, color=t.TEXT_DIM)],
            )
        )
    return ft.Row(items, spacing=14, wrap=True)


# ── 상단 바 연결 배지 ────────────────────────────────────────


class ConnectionBadge:
    """상단 바에 붙는 연결 상태 표시.

    state_client 가 polling 결과에 따라 set_* 을 부른다.
    컨트롤을 새로 만들지 않고 기존 것의 색·글자만 바꾼다.
    """

    def __init__(self):
        self._dot = dot(t.STATUS["error"])
        self._text = ft.Text("연결 끊김", size=t.SIZE_LABEL, color=t.TEXT_DIM)
        self.control = ft.Row([self._dot, self._text], spacing=6, tight=True)

    def _set(self, color: str, text: str):
        self._dot.bgcolor = color
        self._text.value = text

    def set_connected(self):
        self._set(t.STATUS["confirmed"], "연결됨")

    def set_stale(self, seconds: float):
        self._set(t.STATUS["warning"], f"데이터 정지 {seconds:.0f}초")

    def set_disconnected(self):
        self._set(t.STATUS["error"], "연결 끊김")