"""화면을 구성하는 사각 패널.

모든 화면이 이 함수로 박스를 만든다. 테두리·여백·제목 위치를 한 곳에서 관리한다.
"""

import flet as ft

import theme as t


def panel(title: str, content: ft.Control | None = None, accent: str | None = None):
    """제목이 달린 패널 하나.

    title   왼쪽 위 작은 라벨
    content 패널 안에 들어갈 컨트롤
    accent  라벨 앞 점 색. 상태를 나타낼 때 쓴다. None 이면 점 없음
    """
    label_row = [
        ft.Text(
            title,
            size=t.SIZE_LABEL,
            color=t.TEXT_DIM,
            weight=ft.FontWeight.W_500,
        )
    ]
    if accent:
        label_row.insert(
            0,
            ft.Container(
                width=6,
                height=6,
                bgcolor=accent,
                border_radius=3,
            ),
        )

    return ft.Container(
        expand=True,
        bgcolor=t.SURFACE,
        border=ft.Border.all(1, t.BORDER),
        border_radius=t.RADIUS,
        padding=t.PAD,
        content=ft.Column(
            spacing=10,
            controls=[
                ft.Row(label_row, spacing=6),
                ft.Container(
                    expand=True,
                    content=content or placeholder(title),
                ),
            ],
        ),
    )


def placeholder(name: str):
    """아직 구현하지 않은 패널에 넣는 안내."""
    return ft.Container(
        alignment=ft.Alignment.CENTER,
        content=ft.Text(name, size=t.SIZE_BODY, color=t.TEXT_FAINT),
    )


def kv(key: str, value: str, value_color: str | None = None, mono: bool = False):
    """라벨 + 값 한 줄. 상태 요약 패널에서 반복해서 쓴다."""
    return ft.Row(
        alignment=ft.MainAxisAlignment.SPACE_BETWEEN,
        controls=[
            ft.Text(key, size=t.SIZE_BODY, color=t.TEXT_DIM),
            ft.Text(
                value,
                size=t.SIZE_BODY,
                color=value_color or t.TEXT,
                font_family=t.MONO if mono else None,
            ),
        ],
    )