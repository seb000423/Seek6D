# back_ui

ROS 2 토픽/서비스를 `front_ui`가 이해하는 HTTP(JSON) 계약으로 변환하는
어댑터 노드. `front_ui`는 ROS를 전혀 모르므로, 이 노드가 유일한 접점이다.

## 빌드 & 실행

```bash
colcon build --packages-select interfaces db back_ui
source install/setup.bash
ros2 run back_ui node
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `robot_name` | `dsr01` | 관절 상태를 구독할 로봇 네임스페이스(`/<robot_name>/joint_states`) |

HTTP 서버는 `127.0.0.1:8765`에서 뜬다(front_ui의 `config.BASE_URL`과 일치해야 함).

## 하는 일

| 입력 | 방식 | 출력 |
|---|---|---|
| `/<robot_name>/joint_states` | 구독 | 순정 FK(`robot_fk.py`)로 계산한 링크별 pos/rpy → `robot.links` |
| `/camera/camera/color/image_raw` | 구독 | JPEG 인코딩 후 `/frame.jpg`로 서빙 |
| `/ui/task_state` (state가 발행) | 구독 | `task` 스냅샷 갱신 |
| `/state/current` (state가 발행, TRANSIENT_LOCAL) | 구독 | `system.state`/`system.nodes` 갱신. `targets` 이름 → front_ui 노드 키 매핑은 `_NODE_KEY_MAP` 참고(`"control"` → `"main"`) |
| `db/load` (table=items, tasks) | 1초 주기 폴링(`call_async`) | `objects`(mm→m 변환), `recent_tasks`(status 코드 매핑: `SUCCEEDED/FAILED/ABORTED` → `SUCCESS/FAILED/CANCELED`) |

`db`는 이벤트를 쏘지 않고 물어보면 답하는 서비스라서, 최신 상태를
계속 반영하려면 타이머로 계속 폴링하는 수밖에 없다 — 폴링 콜백은
`call_async`만 쓰고 응답을 기다리지 않는다(다른 콜백을 막지 않기 위해).

## 제공 HTTP 엔드포인트

`front_ui/README.md`의 계약과 동일하다.

| 경로 | 내용 |
|---|---|
| `GET /state` | `StateStore.get_snapshot()`을 그대로 JSON으로 반환 |
| `GET /health` | `{"ok": true}` |
| `GET /frame.jpg` | 최신 카메라 프레임(JPEG). 없으면 없음 응답 |

## 좌표계 관련 주의

`db`의 `items.x/y/z`는 비전 노드가 mm 단위로 주는 것으로 보여서
`/1000.0`로 나눠 미터로 변환한다. **좌표축 방향이 front_ui가 가정하는
base_link 기준(REP-103: x 전방/y 좌/z 상)과 실제로 일치하는지는 카메라-로봇
캘리브레이션 쪽에서 별도로 확인이 필요하다** — 이 노드는 단위 변환만
책임진다.
