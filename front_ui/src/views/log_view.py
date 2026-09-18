"""로그 화면.

┌──────────┬──────────┐
│ 작업 기록 │ 물건 위치 │        ← 탭 (직접 만든 pill 버튼. ft.Tabs 대신
├──────────┴──────────┴──────┐    쓴 이유는 API 변경 위험을 피하기 위함)
│                            │
│  선택한 탭 내용               │
│                            │
└────────────────────────────┘

두 탭 다 db 기반이다:
  작업 기록 = recent_tasks (db tasks 테이블, back_ui가 폴링해서 보내줌)
  물건 위치 = objects (db items 테이블, back_ui가 폴링해서 보내줌)

"탐색 구역 수"/"파지 결과"(작업 기록), "위치 라벨"(물건 위치), 이벤트
타임라인, 작업별 이미지는 아직 뺐다 — 지금 스키마·db에 그 데이터 자체가
없다(back_ui/README.md "back_ui가 줘야 하는 것" 참고). 나중에 db/back_ui
쪽에 그 데이터가 생기면 여기 표에 칸만 추가하면 된다.
"""

import flet as ft

import labels as L
import theme as t
from components import status as st
from components.panel import panel

TABS = ["작업 기록", "물건 위치"]


def _format_datetime(iso_str: str | None) -> str:
    """ISO 8601 -> "YYYY-MM-DD HH:MM". 홈의 "최근 실행 경과"(시:분만)보다
    로그 화면은 자리가 넉넉해서 날짜까지 보여준다."""
    if not iso_str:
        return L.EMPTY
    try:
        date_part, time_part = iso_str.split("T")
        return f"{date_part} {time_part[:5]}"
    except (ValueError, AttributeError):
        return iso_str


def _format_duration(seconds) -> str:
    if seconds is None:
        return L.EMPTY
    seconds = int(seconds)
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def _format_confidence(value) -> str:
    return f"{value:.2f}" if isinstance(value, (int, float)) else L.EMPTY


def _format_pos(pos) -> str:
    if not pos:
        return L.EMPTY
    return ", ".join(f"{v:.2f}" for v in pos)


def _task_header() -> ft.Control:
    return ft.Row(
        controls=[
            ft.Text("실행 시각", size=t.SIZE_LABEL, color=t.TEXT_FAINT, width=140),
            ft.Text("대상 물체", size=t.SIZE_LABEL, color=t.TEXT_FAINT, expand=True),
            ft.Text("결과", size=t.SIZE_LABEL, color=t.TEXT_FAINT, width=60),
            ft.Text("소요 시간", size=t.SIZE_LABEL, color=t.TEXT_FAINT, width=70),
        ],
    )


def _task_row(task: dict) -> ft.Control:
    return ft.Row(
        controls=[
            ft.Text(_format_datetime(task.get("ended_at")), size=t.SIZE_BODY,
                     color=t.TEXT_DIM, font_family=t.MONO, width=140),
            ft.Text(task.get("target_name") or L.EMPTY, size=t.SIZE_BODY,
                     color=t.TEXT, expand=True),
            st.task_status(task.get("result")),
            ft.Text(_format_duration(task.get("duration_sec")), size=t.SIZE_BODY,
                     color=t.TEXT_DIM, font_family=t.MONO, width=70),
        ],
    )


def _object_header() -> ft.Control:
    return ft.Row(
        controls=[
            ft.Text("물체 이름", size=t.SIZE_LABEL, color=t.TEXT_FAINT, expand=True),
            ft.Text("좌표 (m, base_link)", size=t.SIZE_LABEL, color=t.TEXT_FAINT, width=170),
            ft.Text("상태", size=t.SIZE_LABEL, color=t.TEXT_FAINT, width=80),
            ft.Text("마지막 확인", size=t.SIZE_LABEL, color=t.TEXT_FAINT, width=140),
            ft.Text("신뢰도", size=t.SIZE_LABEL, color=t.TEXT_FAINT, width=55),
        ],
    )


def _object_row(obj: dict) -> ft.Control:
    return ft.Row(
        controls=[
            ft.Text(obj.get("name") or obj.get("id") or L.EMPTY, size=t.SIZE_BODY,
                     color=t.TEXT, expand=True),
            ft.Text(_format_pos(obj.get("pos")), size=t.SIZE_BODY,
                     color=t.TEXT_DIM, font_family=t.MONO, width=170),
            st.object_status(obj.get("status")),
            ft.Text(_format_datetime(obj.get("last_seen")), size=t.SIZE_BODY,
                     color=t.TEXT_DIM, font_family=t.MONO, width=140),
            ft.Text(_format_confidence(obj.get("confidence")), size=t.SIZE_BODY,
                     color=t.TEXT_DIM, font_family=t.MONO, width=55),
        ],
    )


class LogView:
    """`.control`은 한 번만 만들고 `.update(snapshot)`으로 표 내용만 갱신한다.

    탭 전환은 body.content를 바꾸는 정도라 가벼워서(3D 맵과 달리 카메라
    상태를 안 들고 있음) HomeView처럼 서브트리 보존에 신경 쓸 필요는 없다
    — 그래도 탭 버튼을 누를 때마다 표를 다시 만들지 않도록 컨테이너는
    한 번만 만들고 내용(Column.controls)만 매번 갈아끼운다.
    """

    def __init__(self):
        self._current_tab = 0
        self._recent_tasks: list = []
        self._objects: list = []

        self._tasks_body = ft.Column(spacing=6, expand=True, scroll=ft.ScrollMode.AUTO, controls=[])
        self._objects_body = ft.Column(spacing=6, expand=True, scroll=ft.ScrollMode.AUTO, controls=[])

        tasks_panel = panel(
            "작업 기록",
            content=ft.Column(
                spacing=8,
                controls=[_task_header(), ft.Divider(height=1, color=t.BORDER), self._tasks_body],
            ),
        )
        objects_panel = panel(
            "물건 위치",
            content=ft.Column(
                spacing=8,
                controls=[_object_header(), ft.Divider(height=1, color=t.BORDER), self._objects_body],
            ),
        )
        self._tab_panels = [tasks_panel, objects_panel]

        self._body = ft.Container(expand=True, content=self._tab_panels[0])
        self._buttons = [self._make_tab_button(i, label) for i, label in enumerate(TABS)]

        self.control = ft.Column(
            spacing=t.GAP,
            controls=[
                ft.Row(self._buttons, spacing=t.GAP),
                self._body,
            ],
        )
        self.update(None)

    def _make_tab_button(self, index: int, label: str) -> ft.Control:
        def on_click(e):
            self._current_tab = index
            for i, btn in enumerate(self._buttons):
                selected = i == index
                btn.bgcolor = t.ACCENT if selected else t.SURFACE
                btn.content.color = t.BG if selected else t.TEXT_DIM
            self._body.content = self._tab_panels[index]
            e.page.update()

        return ft.Container(
            width=150,
            height=34,
            border_radius=17,
            alignment=ft.Alignment.CENTER,
            bgcolor=t.ACCENT if index == 0 else t.SURFACE,
            border=ft.Border.all(1, t.BORDER),
            on_click=on_click,
            content=ft.Text(
                label,
                size=t.SIZE_BODY,
                weight=ft.FontWeight.W_600,
                color=t.BG if index == 0 else t.TEXT_DIM,
            ),
        )

    def update(self, snapshot: dict | None):
        recent_tasks = (snapshot or {}).get("recent_tasks") or []
        objects = (snapshot or {}).get("objects") or []

        if recent_tasks:
            self._tasks_body.controls = [_task_row(task) for task in recent_tasks]
        else:
            self._tasks_body.controls = [
                ft.Text("작업 기록 없음", size=t.SIZE_BODY, color=t.TEXT_FAINT)
            ]

        if objects:
            self._objects_body.controls = [_object_row(obj) for obj in objects]
        else:
            self._objects_body.controls = [
                ft.Text("등록된 물체 없음", size=t.SIZE_BODY, color=t.TEXT_FAINT)
            ]

        try:
            self.control.update()
        except RuntimeError:
            # HomeView와 같은 이유 — 생성자의 초기 update(None) 호출 시점엔
            # 아직 page에 안 붙어있다.
            pass
