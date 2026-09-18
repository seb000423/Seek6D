"""back_ui 접속 설정과 폴링 주기.

여러 파일에 흩어지면 나중에 주소 바꿀 때 귀찮으므로 여기 모아둔다.
"""

BASE_URL = "http://127.0.0.1:8765"

STATE_PATH = "/state"
FRAME_PATH = "/frame.jpg"
HEALTH_PATH = "/health"

# 화면별 폴링 주기 (초)
POLL_MONITOR = 0.1   # 작업 화면
POLL_HOME = 0.5      # 홈
POLL_LOG = 2.0       # 로그

REQUEST_TIMEOUT = 1.0

# ts 가 이 시간 이상 갱신되지 않으면 "데이터 정지"로 본다 (초)
STALE_AFTER = 3.0