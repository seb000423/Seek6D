"""front_ui 진입점.

좌측 네비게이션 + 상단 바 + 화면 전환 + state_client 폴링 연결을 담당한다.
"""

import flet as ft

import theme as t
from components.status import ConnectionBadge
from client.state_client import (
    StateClient,
    STATUS_CONNECTED,
    STATUS_STALE,
    STATUS_DISCONNECTED,
)
from views.home_view import HomeView
from views.monitor_view import MonitorView
from views.log_view import LogView


PAGES = [
    ("홈", ft.Icons.HOME_OUTLINED, ft.Icons.HOME, HomeView),
    ("작업", ft.Icons.VISIBILITY_OUTLINED, ft.Icons.VISIBILITY, MonitorView),
    ("로그", ft.Icons.LIST_ALT_OUTLINED, ft.Icons.LIST_ALT, LogView),
]


def main(page: ft.Page):
    page.title = "숨은 물체 탐색 모니터"
    page.bgcolor = t.BG
    page.padding = 0
    page.spacing = 0
    page.theme_mode = ft.ThemeMode.DARK
    page.window.width = 1440
    page.window.height = 900
    page.window.min_width = 1100
    page.window.min_height = 700

    title = ft.Text(PAGES[0][0], size=t.SIZE_BODY, color=t.TEXT_DIM)

    # 연결 상태 배지. state_client 폴링 결과에 따라 아래 on_status 가 갱신한다.
    conn = ConnectionBadge()

    topbar = ft.Container(
        height=44,
        padding=ft.Padding.symmetric(horizontal=16),
        bgcolor=t.RAIL,
        border=ft.Border.only(bottom=ft.BorderSide(1, t.BORDER)),
        content=ft.Row(
            alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
            controls=[
                title,
                conn.control,
            ],
        ),
    )

    body = ft.Container(expand=True, padding=t.GAP)

    # 화면 인스턴스는 최초 방문할 때 한 번만 만들고 계속 재사용한다.
    # (매 폴링마다 새로 만들면 홈 화면의 드래그 3D 뷰어까지 통째로 다시
    #  생성돼서 드래그 도중 다른 패널이 갱신될 때마다 뷰어가 끊긴다)
    current_index = 0
    latest_snapshot: dict | None = None
    view_instances: dict[int, object] = {}

    def get_view(index: int):
        view = view_instances.get(index)
        if view is None:
            _, _, _, factory = PAGES[index]
            view = factory()
            view_instances[index] = view
        return view

    def on_nav_change(e):
        nonlocal current_index
        current_index = e.control.selected_index
        title.value = PAGES[current_index][0]
        view = get_view(current_index)
        view.update(latest_snapshot)  # 그동안 못 본 최신값으로 맞춘다
        body.content = view.control

    def on_snapshot(snapshot):
        nonlocal latest_snapshot
        latest_snapshot = snapshot

    async def _apply_status(status: str, age: float | None):
        if status == STATUS_CONNECTED:
            conn.set_connected()
        elif status == STATUS_STALE:
            conn.set_stale(age or 0.0)
        else:
            conn.set_disconnected()
        # 화면 전체를 다시 만들지 않는다 — 지금 보이는 화면의 값만 갱신한다.
        get_view(current_index).update(latest_snapshot)
        page.update()

    def on_status(status: str, age: float | None):
        # state_client의 폴링 스레드에서 직접 호출된다 — page 이벤트 루프
        # 소유 스레드가 아니다. page.update()를 여기서 바로 부르면 스레드
        # 안전하지 않아서(큐에만 쌓이고 안 나감) 클릭 등 진짜 클라이언트
        # 이벤트가 와야만 밀린 게 한꺼번에 반영되는 것처럼 보인다.
        # run_task()가 run_coroutine_threadsafe로 이벤트 루프에 안전하게 넘겨준다.
        page.run_task(_apply_status, status, age)

    state_client = StateClient(on_snapshot=on_snapshot, on_status=on_status)
    state_client.start()
    page.on_close = lambda e: state_client.stop()

    rail = ft.NavigationRail(
        selected_index=0,
        bgcolor=t.RAIL,
        min_width=76,
        label_type=ft.NavigationRailLabelType.ALL,
        indicator_color=t.SURFACE_ALT,
        on_change=on_nav_change,
        destinations=[
            ft.NavigationRailDestination(icon=icon, selected_icon=sel, label=label)
            for label, icon, sel, _ in PAGES
        ],
    )

    page.controls.append(
        ft.Row(
            spacing=0,
            expand=True,
            controls=[
                rail,
                ft.Container(width=1, bgcolor=t.BORDER),
                ft.Column(spacing=0, expand=True, controls=[topbar, body]),
            ],
        )
    )

    body.content = get_view(0).control


ft.run(main)
