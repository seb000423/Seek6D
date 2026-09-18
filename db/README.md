# db

물체 위치와 작업 기록을 저장하는 SQLite 기반 DB 노드. 다른 노드는 DB
파일을 직접 열지 않고, 전부 이 노드가 제공하는 ROS 2 서비스를 통해서만
읽고 쓴다.

## 빌드 & 실행

```bash
colcon build --packages-select interfaces db
source install/setup.bash
ros2 run db db_node
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `db_path` | `~/.ros/robot_db/robot.db` | SQLite 파일 경로 |
| `require_init` | `false` | true면 `/db/init` 확인 전 다른 요청을 거부 |
| `allow_clear` | `true` | `/db/clear` 허용 여부(운영 환경에서는 false 권장) |

```bash
ros2 run db db_node --ros-args -p db_path:=/tmp/test.db
```

## 제공 서비스

| 서비스 | 타입 | 용도 |
|---|---|---|
| `/db/init` | `interfaces/NodeInit` | 상태머신의 기동 확인 요청 |
| `/db/save` | `interfaces/DbSave` | `items`/`tasks` 테이블에 행 저장 |
| `/db/load` | `interfaces/DbLoad` | `items`/`tasks` 조회 |
| `/db/clear` | `std_srvs/Trigger` | `items` 테이블 전체 삭제(테스트용) |

## 테이블

**`items`** — 마지막으로 확인된 물체 위치. `class_name`이 UNIQUE 키라
같은 클래스가 다시 감지되면 최신 값으로 덮어쓴다(upsert).

| 컬럼 | 타입 |
|---|---|
| class_name | TEXT UNIQUE |
| confidence | REAL |
| x, y, z | REAL |
| last_seen | TEXT (ISO8601) |

**`tasks`** — 작업 완료 시점에 1행씩 추가(수정 없음). 화이트리스트
(`ALLOWED_COLUMNS`)에 없는 컬럼이 요청에 하나라도 섞여 있으면 전체
요청을 거부한다(부분 저장 없음).

| 컬럼 | 타입 |
|---|---|
| command_text | TEXT |
| voice_command | TEXT |
| target_name | TEXT |
| destination | TEXT |
| status | TEXT (NOT NULL) |
| fail_stage | TEXT |
| fail_reason | TEXT |
| found_at | TEXT |
| started_at | TEXT (NOT NULL) |
| ended_at | TEXT |

스키마가 바뀌면(코드에서 컬럼을 추가/변경하면) 기존 DB 파일의 실제
컬럼과 비교해서 다르면 기동 시 해당 테이블을 자동으로 `DROP`하고
다시 만든다 — 개발 단계에서 죽지 않는 것이 우선이라 데이터 보존보다
안전한 재기동을 택한 것이다.

## 요청/응답 JSON 예시

`DbSave`/`DbLoad`의 `request` 문자열은 `table` 필드로 대상 테이블을
고른다(생략하면 `items`).

```jsonc
// DbSave, table=items (rows 대신 예전 방식 {"items":[...]}도 호환)
{"table": "items", "rows": [{"class_name": "green_frog", "confidence": 0.87,
                              "x": 412.5, "y": -135.8, "z": 125.3}]}
// 응답: {"inserted": 1, "updated": 0, "results": [...]}

// DbSave, table=tasks
{"table": "tasks", "rows": [{"target_name": "green_frog", "status": "SUCCEEDED",
                              "started_at": "...", "ended_at": "..."}]}
// 응답: {"table": "tasks", "inserted": 1, "results": [...]}

// DbLoad, table=items
{"class_name": "green_frog"}   // 생략하면 전체 조회
// 응답: {"count": 1, "items": [{...}]}

// DbLoad, table=tasks
{"table": "tasks"}
// 응답: {"table": "tasks", "count": 12, "rows": [{...}, ...]}
```

## 대화형 테스트 도구

```bash
ros2 run db save_test
```

메뉴에서 테스트 데이터 생성/조회/직접 등록/전체 삭제/초기화 확인을
숫자로 선택해 실제 서비스 호출 없이 손으로 검증할 수 있다.
