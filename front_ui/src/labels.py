"""코드값 → 화면 표시 문자열 매핑.

back_ui 는 영문 코드값만 보낸다. 한글 라벨은 전부 여기서 붙인다.
문구를 고칠 일이 생기면 이 파일만 열면 된다.
"""

# ── 시스템 상태 (state 노드) ─────────────────────────────────
SYSTEM_STATE = {
    "LOAD": "초기화 중",
    "IDLE": "대기",
    "RUN": "작업 중",
}

# ── 노드 이름 ───────────────────────────────────────────────
NODE = {
    "image": "이미지",
    "main": "메인",
    "db": "DB",
    "voice": "음성",
    "state": "상태",
}

# ── 물체 상태 ───────────────────────────────────────────────
OBJECT_STATUS = {
    "unknown": "위치 불명",
    "searching": "탐색 중",
    "confirmed": "확인됨",
    "held": "손에 있음",
    "warning": "경고",
    "error": "오류",
}

# ── 작업 상태 ───────────────────────────────────────────────
TASK_STATUS = {
    "RUNNING": "진행 중",
    "SUCCESS": "성공",
    "FAILED": "실패",
    "CANCELED": "취소",
    "ERROR": "오류",
}

# 작업 상태 → theme.STATUS 키
TASK_STATUS_COLOR = {
    "RUNNING": "searching",
    "SUCCESS": "confirmed",
    "FAILED": "error",
    "CANCELED": "unknown",
    "ERROR": "error",
}

# ── 실행 단계 ───────────────────────────────────────────────
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

STAGE = dict(STAGES)
STAGE["failed"] = "실패"

STAGE_ORDER = [code for code, _ in STAGES]

# ── 탐색 구역 ───────────────────────────────────────────────
ZONE_TYPE = {
    "shelf": "선반",
    "drawer": "서랍",
    "door": "문",
    "divider": "칸막이",
}

ZONE_STATE = {
    "closed": "닫힘",
    "opening": "여는 중",
    "open": "열림",
    "closing": "닫는 중",
}

ZONE_SEARCH_STATE = {
    "untouched": "미탐색",
    "observing": "관측 중",
    "done": "탐색 완료",
    "found": "후보 발견",
    "failed": "탐색 실패",
}

# 구역 탐색 상태 → theme.STATUS 키
ZONE_SEARCH_COLOR = {
    "untouched": "unknown",
    "observing": "searching",
    "done": "unknown",
    "found": "confirmed",
    "failed": "error",
}

# ── 그리퍼 ──────────────────────────────────────────────────
GRIPPER = {
    "open": "열림",
    "closed": "닫힘",
    "holding": "물체 파지 중",
}

EMPTY = "-"


def get(table: dict, key, default: str = EMPTY) -> str:
    """매핑에 없는 코드값이 와도 앱이 죽지 않게 한다.

    back_ui 가 아직 안 정한 값을 보내는 경우가 개발 중에 자주 생긴다.
    모르는 코드는 그대로 보여 줘서 무슨 값이 왔는지 알 수 있게 한다.
    """
    if key is None:
        return default
    return table.get(key, str(key))