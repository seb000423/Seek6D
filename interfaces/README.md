# interfaces

프로젝트 전체 노드가 공유하는 ROS 2 커스텀 srv/action 정의 모음.
다른 모든 패키지가 이 패키지에 의존하므로, 워크스페이스를 빌드할 때
가장 먼저 단독으로 빌드해야 한다.

```bash
colcon build --packages-select interfaces
source install/setup.bash
```

## 공통 규약

`NodeInit`/`DbSave`/`DbLoad`/`ControlTask`/`RobotResult`/`DetectObject`는
전부 `string request` → `bool success, string response, string message`
형태다. 실제 구조화된 데이터는 필드를 늘리는 대신 `request`/`response`
문자열 안에 JSON으로 담는다 — 각 서비스가 다루는 내용이 노드마다 달라서
매번 새 srv를 만드는 대신 이 방식을 공용으로 쓴다. 정확한 JSON 형태는
그 서비스를 제공하는 노드(`db`, `state`)의 소스 상단 문서를 참고한다.

`TargetSearch`와 `Search` 액션만 이 규약에서 예외다 — 음성 명령 경로의
빈도가 높고 필드가 고정적이라 처음부터 전용 필드(`target_name`,
`class_label`)로 정의했다.

## 제공 인터페이스

### 서비스 (`srv/`)

| 이름 | 제공 노드 | 용도 |
|---|---|---|
| `NodeInit` | 전 노드 공용 (`db/init`, `<node>/init` 등) | 상태머신의 기동 확인 요청. `request`에 `{"node": "..."}` |
| `DbSave` | `db` (`db/save`) | `items`/`tasks` 테이블에 행 저장. `table` 필드로 라우팅 |
| `DbLoad` | `db` (`db/load`) | `items`/`tasks` 조회 |
| `TargetSearch` | `state` (`state/target_search`) | 음성 노드가 상태머신에 탐색 명령 전달 |
| `RobotResult` | `state` (`state/robot_result`) | 제어 노드가 작업 진행 상황을 상태머신에 보고 |
| `ControlTask` | 제어 노드(이 저장소 외부) | 상태머신 → 제어 작업 지시 |
| `DetectObject` | `vision_nodes` | 제어 → 비전 객체 탐지 요청 |
| `UpdateTcpPose` | `vision_nodes` | 제어 → 비전에 현재 TCP 자세 전달 |

### 액션 (`action/`)

| 이름 | 제공 노드 | 용도 |
|---|---|---|
| `Search` | 제어 노드(이 저장소 외부) | 상태머신이 발행하는 탐색 실행 요청. Goal(`target_name`, `class_label`) / Result(`success`, `location`, `message`) / Feedback(`step`, `progress`) |

## 패키지 추가 시 주의

새 srv/action 파일을 추가하면 `CMakeLists.txt`의 `rosidl_generate_interfaces()`
목록에도 반드시 추가해야 빌드에 포함된다. 파일만 추가하고 이 목록에
안 넣는 실수가 잦다 — 빌드 후 `ros2 interface show interfaces/srv/<이름>`으로
확인한다.
