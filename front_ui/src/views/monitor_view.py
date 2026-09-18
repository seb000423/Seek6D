"""작업 화면."""

import time

import flet as ft

import config as cfg
import theme as t
from components.panel import panel
from render3d.map_view import MapView


STAGES = [
    ("idle", "대기"),
    ("initial_observe", "초기 관측"),
    ("select_zone", "탐색 위치 선정"),
    ("open_container", "서랍/문 열기"),
    ("internal_reobserve", "내부 재관측"),
    ("verify_candidate", "후보 확인"),
    ("approach_grasp", "파지 접근"),
    ("verify_grasp", "파지 검증"),
    ("transport", "지정 위치로 이송"),
    ("place", "물체 내려놓기"),
    ("return_home", "홈 복귀"),
    ("done", "완료"),
]


def stage_row(label: str, state: str):
    if state == "current":
        dot = t.ACCENT
        color = t.TEXT
        weight = ft.FontWeight.W_600

    elif state == "done":
        dot = t.STATUS["confirmed"]
        color = t.TEXT_DIM
        weight = ft.FontWeight.NORMAL

    else:
        dot = t.BORDER
        color = t.TEXT_FAINT
        weight = ft.FontWeight.NORMAL

    return ft.Row(
        spacing=8,
        controls=[
            ft.Container(
                width=6,
                height=6,
                bgcolor=dot,
                border_radius=3,
            ),
            ft.Text(
                label,
                size=t.SIZE_BODY,
                color=color,
                weight=weight,
            ),
        ],
    )


class MonitorView:
    def __init__(self):
        # 마지막으로 표시한 카메라 프레임 ID
        self.last_frame_id = None

        # 마지막으로 화면을 갱신한 시각
        self.last_frame_update = 0.0

        # 최대 30FPS로 화면 갱신
        self.frame_update_interval = 1.0 / 30.0

        # RealSense 컬러 영상
        self.camera_image = ft.Image(
            src="",
            fit=ft.BoxFit.FILL,
            gapless_playback=True,
            visible=False,
        )

        # 카메라 연결 전 표시 문구
        self.camera_message = ft.Text(
            "카메라 연결 없음",
            size=t.SIZE_BODY,
            color=t.TEXT_FAINT,
        )

        # 검출 결과 표시
        self.detection_text = ft.Text(
            "검출 결과 없음",
            size=t.SIZE_BODY,
            color=t.TEXT_DIM,
        )

        # 현재 행동
        self.action_text = ft.Text(
            "-",
            size=t.SIZE_VALUE,
            color=t.TEXT,
        )

        # 행동 이유
        self.reason_text = ft.Text(
            "-",
            size=t.SIZE_BODY,
            color=t.TEXT_DIM,
        )

        # 작업 단계
        self.stage_column = ft.Column(
            spacing=7,
            scroll=ft.ScrollMode.AUTO,
            controls=[
                stage_row(label, "todo")
                for _, label in STAGES
            ],
        )

        # 작업 정보
        self.task_id_text = ft.Text("-", color=t.TEXT)
        self.target_text = ft.Text("-", color=t.TEXT)
        self.voice_text = ft.Text("-", color=t.TEXT)
        self.status_text = ft.Text("-", color=t.TEXT)
        self.elapsed_text = ft.Text("--:--", color=t.TEXT)
        self.zone_text = ft.Text("-", color=t.TEXT)

        # GitHub 기존 3D 지도 기능 유지
        self._map_view = MapView(
            width=480,
            height=280,
            show_robot=True,
        )

        self.control = self._build()

    def _build(self):
        # 640×480 영상과 동일한 4:3 비율
        camera_view = ft.Container(
            aspect_ratio=4 / 3,
            bgcolor="#0A0D12",
            border_radius=4,
            clip_behavior=ft.ClipBehavior.HARD_EDGE,
            content=ft.Stack(
                fit=ft.StackFit.EXPAND,
                controls=[
                    self.camera_image,

                    # 카메라가 연결되지 않았을 때만 표시
                    ft.Container(
                        alignment=ft.Alignment.CENTER,
                        content=self.camera_message,
                    ),
                ],
            ),
        )

        camera = panel(
            "현재 카메라 화면",
            content=ft.Column(
                spacing=8,
                horizontal_alignment=(
                    ft.CrossAxisAlignment.CENTER
                ),
                controls=[
                    camera_view,
                    self.detection_text,
                ],
            ),
        )

        action = panel(
            "현재 판단과 행동",
            accent=t.ACCENT,
            content=ft.Column(
                spacing=14,
                controls=[
                    ft.Column(
                        spacing=4,
                        controls=[
                            ft.Text(
                                "현재 행동",
                                size=t.SIZE_LABEL,
                                color=t.TEXT_DIM,
                            ),
                            self.action_text,
                        ],
                    ),
                    ft.Divider(
                        height=1,
                        color=t.BORDER,
                    ),
                    ft.Column(
                        spacing=4,
                        controls=[
                            ft.Text(
                                "행동 이유",
                                size=t.SIZE_LABEL,
                                color=t.TEXT_DIM,
                            ),
                            self.reason_text,
                        ],
                    ),
                ],
            ),
        )

        map3d = panel(
            "3D 지도",
            content=ft.Container(
                expand=True,
                alignment=ft.Alignment.CENTER,
                content=self._map_view.control,
            ),
        )

        stages = panel(
            "현재 실행 단계",
            content=self.stage_column,
        )

        info = panel(
            "작업 정보",
            content=ft.Row(
                spacing=t.GAP * 3,
                controls=[
                    ft.Column(
                        spacing=6,
                        expand=1,
                        controls=[
                            ft.Row(
                                controls=[
                                    ft.Text(
                                        "작업 ID",
                                        color=t.TEXT_DIM,
                                    ),
                                    self.task_id_text,
                                ],
                            ),
                            ft.Row(
                                controls=[
                                    ft.Text(
                                        "대상 물체",
                                        color=t.TEXT_DIM,
                                    ),
                                    self.target_text,
                                ],
                            ),
                            ft.Row(
                                controls=[
                                    ft.Text(
                                        "음성 명령",
                                        color=t.TEXT_DIM,
                                    ),
                                    self.voice_text,
                                ],
                            ),
                        ],
                    ),
                    ft.Column(
                        spacing=6,
                        expand=1,
                        controls=[
                            ft.Row(
                                controls=[
                                    ft.Text(
                                        "작업 상태",
                                        color=t.TEXT_DIM,
                                    ),
                                    self.status_text,
                                ],
                            ),
                            ft.Row(
                                controls=[
                                    ft.Text(
                                        "경과 시간",
                                        color=t.TEXT_DIM,
                                    ),
                                    self.elapsed_text,
                                ],
                            ),
                            ft.Row(
                                controls=[
                                    ft.Text(
                                        "현재 구역",
                                        color=t.TEXT_DIM,
                                    ),
                                    self.zone_text,
                                ],
                            ),
                        ],
                    ),
                    ft.Container(
                        expand=2,
                        content=ft.Text(
                            "탐색 영역 상태 표",
                            size=t.SIZE_BODY,
                            color=t.TEXT_FAINT,
                        ),
                    ),
                ],
            ),
        )

        return ft.Column(
            spacing=t.GAP,
            controls=[
                # 1. 현재 카메라 화면 / 3. 3D 지도
                ft.Row(
                    spacing=t.GAP,
                    expand=6,
                    vertical_alignment=(
                        ft.CrossAxisAlignment.STRETCH
                    ),
                    controls=[
                        ft.Container(
                            content=camera,
                            expand=3,
                        ),
                        ft.Container(
                            content=map3d,
                            expand=4,
                        ),
                    ],
                ),

                # 2. 현재 판단과 행동 / 4. 현재 실행 단계
                ft.Row(
                    spacing=t.GAP,
                    expand=2,
                    vertical_alignment=(
                        ft.CrossAxisAlignment.STRETCH
                    ),
                    controls=[
                        ft.Container(
                            content=action,
                            expand=3,
                        ),
                        ft.Container(
                            content=stages,
                            expand=4,
                        ),
                    ],
                ),

                # 5. 작업 정보
                ft.Row(
                    spacing=t.GAP,
                    expand=2,
                    controls=[
                        info,
                    ],
                ),
            ],
        )

    def update(self, snapshot):
        if not snapshot:
            return

        system = snapshot.get("system", {})
        task = snapshot.get("task", {})

        # GitHub 기존 MapView 데이터 갱신 유지
        zones = snapshot.get("zones", [])
        objects = snapshot.get("objects", [])
        robot = snapshot.get("robot") or {}

        self._map_view.update(
            zones,
            task.get("current_zone"),
            objects,
            robot.get("links"),
        )

        # 카메라 상태
        camera_connected = system.get(
            "camera_connected",
            False,
        )

        frame_id = snapshot.get("frame_id")

        if camera_connected and frame_id is not None:
            now = time.monotonic()

            should_update = (
                frame_id != self.last_frame_id
                and (
                    now - self.last_frame_update
                    >= self.frame_update_interval
                )
            )

            if should_update:
                # frame_id를 URL에 추가해 캐시 방지
                self.camera_image.src = (
                    f"{cfg.BASE_URL}"
                    f"{cfg.FRAME_PATH}"
                    f"?id={frame_id}"
                )

                self.last_frame_id = frame_id
                self.last_frame_update = now

            self.camera_image.visible = True
            self.camera_message.visible = False

        else:
            self.camera_image.visible = False
            self.camera_message.visible = True
            self.camera_message.value = (
                "카메라 연결 없음"
            )

        # 검출 결과 표시
        detections = task.get("detections", [])

        if detections:
            detection_labels = []

            for item in detections:
                label = item.get("label", "-")
                confidence = item.get("confidence")

                if confidence is None:
                    detection_labels.append(label)

                else:
                    detection_labels.append(
                        f"{label}  "
                        f"confidence {confidence:.2f}"
                    )

            self.detection_text.value = " / ".join(
                detection_labels
            )

        else:
            self.detection_text.value = (
                "현재 시야에서 대상 물체를 "
                "찾지 못했습니다."
            )

        # 행동과 이유
        self.action_text.value = (
            task.get("action") or "-"
        )

        self.reason_text.value = (
            task.get("action_reason") or "-"
        )

        # 현재 작업 단계
        current_stage = task.get(
            "stage",
            "idle",
        )

        stage_codes = [
            code
            for code, _ in STAGES
        ]

        try:
            current_index = stage_codes.index(
                current_stage
            )

        except ValueError:
            current_index = 0

        self.stage_column.controls = []

        for index, (_, label) in enumerate(STAGES):
            if index < current_index:
                row_state = "done"

            elif index == current_index:
                row_state = "current"

            else:
                row_state = "todo"

            self.stage_column.controls.append(
                stage_row(
                    label,
                    row_state,
                )
            )

        # 작업 정보
        self.task_id_text.value = (
            task.get("task_id") or "-"
        )

        self.target_text.value = (
            task.get("target_name") or "-"
        )

        self.voice_text.value = (
            task.get("voice_command") or "-"
        )

        self.status_text.value = (
            task.get("status") or "-"
        )

        elapsed_sec = int(
            task.get("elapsed_sec", 0)
        )

        minutes, seconds = divmod(
            elapsed_sec,
            60,
        )

        self.elapsed_text.value = (
            f"{minutes:02d}:{seconds:02d}"
        )

        self.zone_text.value = (
            task.get("current_zone") or "-"
        )


def build_monitor(snapshot=None):
    view = MonitorView()

    if snapshot:
        view.update(snapshot)

    return view.control