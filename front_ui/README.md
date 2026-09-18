# front_ui

숨은 물체 탐색 협동로봇의 **관측 전용 대시보드**. ROS 2를 전혀 모르는
독립 프로세스로 동작하는 Flet(Flutter 기반 파이썬 UI) 앱이다 — `rclpy`를
가져오지 않고, `back_ui`가 HTTP로 내려주는 JSON 스냅샷을 폴링해서만
화면을 그린다.

화면은 3개다.

| 화면 | 내용 |
|---|---|
| 홈 | 현재 작업 상태(원형 진행률), 로봇 시스템 상태, 찾고 있는 물체의 3D 뷰어, 물체 위치 3D 지도, 최근 작업 5건 |
| 작업 | 카메라 패널, 현재 판단/행동, 3D 지도(현재 탐색 구역 강조), 실행 단계 체크리스트, 작업 정보 |
| 로그 | DB에 쌓인 작업 기록/물체 위치 전체 목록 (탭 전환) |

---

## 실행

```bash
conda env create -f environment.yaml   # 최초 1회
conda activate front_ui
cd front_ui
flet run
```

`back_ui`가 아직 없거나 로봇/DB 없이 화면만 확인하고 싶을 때는 `tools/fake_server.py`를
대신 띄운다. front_ui 코드는 반대편이 `fake_server`인지 진짜 `back_ui`인지
구분하지 않는다 — 아래 계약(HTTP 주소·경로·JSON 스키마)만 같으면 그대로 붙는다.

```bash
python tools/fake_server.py
```

---

## 아키텍처

```
back_ui / fake_server.py  ──HTTP GET /state (JSON)──▶  StateClient (폴링 스레드)
                                                              │
                                                    on_snapshot 콜백
                                                              ▼
                                                    main.py (Flet Page)
                                                              │
                                            ┌─────────────────┼─────────────────┐
                                            ▼                 ▼                 ▼
                                       HomeView          MonitorView         LogView
                                            │                 │
                                            └────────┬────────┘
                                                      ▼
                                            MapView / ObjectViewer
                                            (render3d/ — numpy 기반 자체 투영)
```

- **`src/client/state_client.py`**: 백그라운드 스레드에서 `/state`를 주기적으로 GET, 콜백으로 스냅샷 전달
- **`src/views/`**: 화면별 컴포넌트 조립 (`home_view.py`, `monitor_view.py`, `log_view.py`)
- **`src/render3d/`**: 3D 지도(`map_view.py`)와 물체 뷰어(`object_viewer.py`) — pyrender/open3d 등 외부 3D 엔진 없이 numpy로 직접 투영·셰이딩
- **`src/components/`**: 패널, 상태 배지 등 공용 UI 조각
- **`src/theme.py`, `src/labels.py`, `src/config.py`**: 색상/간격 상수, 코드값→한글 라벨 매핑, 서버 주소/폴링 주기 설정
- **`src/assets/`**: 씬 배경 치수(`scene_config.json`), 로봇 팔 점군(`robot/*.npy`), 물체 3D 모델(`models/*.obj`)

---

## back_ui가 줘야 하는 것 (HTTP 계약)

`back_ui`(또는 `fake_server.py`)를 작성할 때 이 표를 계약으로 삼는다.
front_ui 코드가 실제로 읽는 필드만 적었고, 어느 화면이 쓰는지 표시했다.
정확한 참고 구현은 `tools/fake_server.py`.

### 1. HTTP 서버

주소는 `src/config.py`에 있다.

| 항목 | 값 |
|---|---|
| BASE_URL | `http://127.0.0.1:8765` |
| 요청 타임아웃 | 1.0초 (`REQUEST_TIMEOUT`) |
| 폴링 주기 | 0.5초 (`POLL_HOME`) |
| 응답 헤더 | `Content-Type: application/json; charset=utf-8`, `Access-Control-Allow-Origin: *` |

| 경로 | 필수 여부 | 내용 |
|---|---|---|
| `GET /state` | 필수 | 아래 스키마. 폴링마다 계속 불림 |
| `GET /health` | 필수 | `{"ok": true}` 아무 200 응답이면 됨(생존 확인용) |
| `GET /frame.jpg` | 선택 | 작업 화면 카메라 패널용 JPEG. 없으면(501 등) "카메라 연결 없음"으로 표시 |

`GET /state`가 실패하거나 응답의 `ts`가 3초(`STALE_AFTER`) 이상 과거면
상단 배지가 "연결 끊김"/"지연"으로 바뀐다 — **`ts`는 매 응답마다 현재 unix
timestamp(초)로 갱신해서 보내야 한다.**

### 2. `GET /state` 응답 스키마

한 덩어리 JSON 스냅샷이다(화면 갱신 타이밍이 어긋나지 않도록 토픽별로
안 쪼갠다). 최상위 키:

```json
{
  "ts": 1733300000.0,
  "frame_id": 123,
  "system": { ... },
  "task": { ... },
  "objects": [ ... ],
  "zones": [ ... ],
  "robot": { "links": [ ... ] },
  "recent_tasks": [ ... ]
}
```

#### `system`

| 필드 | 타입 | 허용값 | 화면 |
|---|---|---|---|
| `state` | string | `LOAD`/`IDLE`/`RUN` | 홈 |
| `nodes` | object | `{"image","main","db","voice","state"}` (키 5개 고정, bool) | 홈 |
| `robot_connected` | bool | | 홈 |
| `camera_connected` | bool | | 홈, 작업(카메라 패널 표시 여부) |
| `gripper_state` | string | `open`/`closed`/`holding` | 홈 |
| `object_count_total/confirmed/unknown` | int | | 홈 |

#### `task`

| 필드 | 타입 | 허용값 | 화면 |
|---|---|---|---|
| `task_id` | string | | 작업 |
| `voice_command` | string \| null | | 홈, 작업 |
| `target_id` | string \| null | | 홈(`objects[].id`와 매칭, `assets/models/{target_id}.obj` 있으면 드래그 뷰어 표시) |
| `target_name` | string \| null | | 홈, 작업 |
| `status` | string | `RUNNING`/`SUCCESS`/`FAILED`/`CANCELED`/`ERROR` | 홈, 작업 |
| `stage` | string | `labels.STAGE_ORDER` 12개(`idle`~`done`) + `failed` | 홈(진행률 %), 작업(단계 체크리스트) |
| `elapsed_sec` | number \| null | | 홈, 작업 |
| `current_zone` | string \| null | `zones[].id`와 매칭 | 홈/작업 3D 지도에서 강조 |
| `action` | string | | 작업("현재 판단과 행동" 패널) |
| `action_reason` | string | | 작업 |
| `detections` | array of `{label, confidence}` | | 작업(검출 결과 텍스트) |

#### `objects[]`

| 필드 | 타입 | 허용값 | 화면 |
|---|---|---|---|
| `id` | string | | 홈(3D 뷰어 매칭) |
| `name` | string | | 홈(3D 지도 라벨), 로그 |
| `category` | string | | 홈(뷰어 대체 카드) |
| `pos` | `[x,y,z]` \| null | 미터, base_link 기준 | **null이면 3D 지도에 안 그림** — 홈/작업, 로그(좌표 표시) |
| `status` | string | `unknown`/`searching`/`confirmed`/`held`/`warning`/`error` | 홈/작업(마커 색), 로그 |
| `zone` | string \| null | | 미사용 |
| `confidence` | number \| null | | 로그 |
| `last_seen` | string(ISO) | | 로그 |

#### `zones[]`

| 필드 | 타입 | 허용값 | 비고 |
|---|---|---|---|
| `id` | string | | `task.current_zone`과 매칭 |
| `name` | string | | 미사용(표시 안 함) |
| `type` | string | `drawer`/`door`(그 외는 서랍처럼 취급) | `door`는 회전(힌지 근사), 그 외는 직선 이동 |
| `pos` | `[x,y,z]` | 미터, 박스 중심, `open_ratio=0` 기준 | |
| `size` | `[x,y,z]` | 미터 | |
| `open_axis` | `[x,y,z]` | 단위벡터, 보통 `[1,0,0]`류 | `type=drawer`일 때만 씀 |
| `open_ratio` | number | 0.0~1.0 | 여는 동안 계속 갱신해서 보내야 함(캐시 없이 매 폴링 다시 그림) |
| `search_state` | string | `untouched`/`observing`/`done`/`found`/`failed` | `current_zone`이 아닐 때 이 값으로 색 결정 |

#### `robot.links[]`

| 필드 | 타입 | 비고 |
|---|---|---|
| `name` | string | `base_link`/`link_1`~`link_6`(M0609 기준). `assets/robot/<name>.npy`와 이름이 정확히 일치해야 함(`tools/mesh_to_points.py` 참고) |
| `pos` | `[x,y,z]` | 미터, base_link(=world) 기준. TF/FK 계산이 끝난 값으로 — front_ui는 TF를 다루지 않음 |
| `rpy` | `[roll,pitch,yaw]` | 라디안. `render3d/shapes.py`의 `rpy_matrix()`: `R = Rz(yaw)·Ry(pitch)·Rx(roll)` |

7개 링크를 다 안 보내도 된다 — 보낸 것만 그린다. 링크가 없거나 `robot`
자체가 비어 있으면 로봇 팔을 그냥 안 그린다(에러 아님).

#### `recent_tasks[]`

| 필드 | 타입 | 허용값 | 화면 |
|---|---|---|---|
| `task_id` | string | | 미사용 |
| `target_name` | string | | 홈(최근 5건만), 로그(전체) |
| `result` | string | `SUCCESS`/`FAILED`/`CANCELED`/`ERROR` | 홈, 로그 |
| `ended_at` | string(ISO 8601) | `2026-08-04T12:03:38` | 홈(시:분만), 로그(전체 일시) |
| `duration_sec` | number | | 홈, 로그 |

### 3. back_ui가 몰라도 되는 것

- `src/assets/scene_config.json` (바닥판·선반·수납장·박스 크기/위치): front_ui가
  직접 들고 있는 고정 배경이다. `/state`로 안 보내도 된다.
- `src/assets/robot/*.npy` (로봇 팔 mesh 점군): `tools/mesh_to_points.py`가
  로봇 mesh에서 미리 뽑아둔 결과물. back_ui는 링크 이름 + pos/rpy만 주면 된다.
- `src/assets/models/*.obj` (물체 3D 모델): 있으면 드래그 뷰어로 보여주고,
  없으면 자동으로 텍스트 카드로 대체한다.

---

## 개발 도구 (`tools/`)

| 스크립트 | 용도 |
|---|---|
| `fake_server.py` | 위 계약과 동일한 HTTP 응답을 흉내내는 가짜 서버. `back_ui` 없이 화면 확인용 |
| `mesh_to_points.py` | 로봇 팔 COLLADA mesh → `src/assets/robot/*.npy` 점군 오프라인 변환 |
| `render_object.py` | 물체 OBJ → 정적 PNG 오프라인 렌더 (드래그 뷰어를 못 쓰는 경우의 폴백) |
