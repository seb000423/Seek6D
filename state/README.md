# state

프로젝트 전체 상태를 관리하는 상태머신 노드. 음성 명령을 접수해 탐색
액션으로 넘기고, 제어 노드의 진행 상황을 UI로 중계하며, 작업이 끝나면
DB에 기록을 남긴다.

## 상태

| 상태 | 의미 |
|---|---|
| `LOAD` | 부팅 중, 다른 노드(`targets` 파라미터)의 준비를 기다리는 중 |
| `IDLE` | 대기 — 이 상태에서만 새 명령을 받는다 |
| `RUN` | 작업 중 — 새 명령은 거절한다 |

부팅 시 `targets`에 나열된 각 노드의 `<노드>/init` 서비스(`interfaces/NodeInit`)를
전부 확인해야 `IDLE`로 넘어간다.

## 빌드 & 실행

```bash
colcon build --packages-select interfaces db back_ui state
source install/setup.bash
ros2 run state state_node
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `targets` | `['db', 'control']` | 기동 시 준비 확인을 요청할 노드 이름 목록 |
| `search_action` | `/control/search` | 탐색 실행을 지시할 액션 이름 |
| `wait_timeout` | `5.0` | 노드별 서비스 대기 시간(초) |
| `retry_period` | `2.0` | 재시도 간격(초) |
| `max_retries` | `0` | 0이면 무한 재시도 |
| `goal_accept_timeout` | `3.0` | 액션 goal 수락 대기 시간(초) |

여러 노드를 한 번에 띄우고 싶으면 통합 launch 파일을 쓴다.

```bash
ros2 launch state seek6d.launch.py
```

## 제공 인터페이스

| 이름 | 타입 | 용도 |
|---|---|---|
| `/state/target_search` | srv (`interfaces/TargetSearch`) | 음성 노드의 탐색 명령 접수(게이트키퍼 — `IDLE`이 아니면 거절) |
| `/state/robot_result` | srv (`interfaces/RobotResult`) | 제어 노드가 작업 진행 상황을 push. 받은 즉시 `/ui/task_state`로 중계하고 바로 응답(다른 서비스를 동기 호출하지 않음 — 제어 노드가 이 응답을 동기로 기다리는 구조라 응답이 늦으면 로봇 동작이 멈춘다) |
| `/state/current` | topic (`std_msgs/String`, TRANSIENT_LOCAL) | `{"state", "ready", "reason"}` JSON. 늦게 뜬 구독자도 마지막 상태를 바로 받는다 |
| `/ui/task_state` | topic (`std_msgs/String`, TRANSIENT_LOCAL) | back_ui가 구독하는 작업 상태 JSON |

## 사용하는 인터페이스

| 이름 | 타입 | 대상 |
|---|---|---|
| `<노드>/init` | srv (`interfaces/NodeInit`) | `targets`에 나열된 각 노드 |
| `search_action` (기본 `/control/search`) | action (`interfaces/Search`) | 제어 노드 — 탐색 실행 지시 |
| `/db/save` | srv (`interfaces/DbSave`) | 작업 종료 시 `tasks` 테이블에 기록 |

## 작업 기록(tasks) 저장 흐름

1. `on_target_search()`에서 탐색 goal이 수락되면 `self.current_task`에
   `command_text`/`voice_command`/`target_name`/`destination`/`started_at`을 담은
   dict를 만든다. `voice_command`는 음성 노드가 `TargetSearch.srv`의
   `target_name` 필드에 실어 보내는 원문 발화 텍스트를 그대로 저장한다.
2. 액션 결과가 오면(`on_search_result`) `status`/`fail_stage`/`fail_reason`/
   `found_at`/`ended_at`을 채워 `/db/save`를 **`call_async`로만** 호출한다.
   서비스 콜백 안에서 동기 `call()`을 쓰면 데드락이라 반드시 비동기로
   던지고 `add_done_callback`으로만 결과를 확인한다.
3. 제어 노드가 `/state/robot_result`로 진행 이벤트를 보내면 `current_task`의
   `stage`를 갱신하고 `/ui/task_state`로 그대로 중계한다.
