# control_node 1.6.1

ROS 2 Humble용 `robot_control` MoveIt 2 마이그레이션 패키지입니다. DB 조회,
Any6D 서비스, Camera→Base 변환, 작업 큐, `/control/task`, `/control/search`,
`/state/robot_result`, RG2 Modbus 제어는 기존 흐름을 유지하고 모션 계층만
MoveIt 2로 교체했습니다.

## 사용 인터페이스

| 기능 | MoveIt 인터페이스 |
|---|---|
| 홈·탐색 관절 이동 | `/move_action` (`MoveGroup`) |
| 충돌회피 자유공간 Pose 이동 | `/move_action` (`MoveGroup`) |
| 직선 파지·상승·상자 조작 | `/compute_cartesian_path` → `/execute_trajectory` |
| 파지 후보 IK 검사 | `/compute_ik` |
| 현재 TCP | TF `base_link` → `rg2_tcp` |
| 외력 해제 | `/dsr01/aux_control/get_tool_force` TCP 힘 변화량 |

일반 물체 파지는 기존 50 mm 후퇴 접근점을 만들지 않고 현재 TCP에서 최종
파지점까지 충돌 검사된 Cartesian 경로를 계산합니다. 상자 뚜껑과 서랍은
조작 특성상 별도의 상부 여유 자세를 유지합니다.

DB에 Base 기준 물체 XYZ가 있으면 해당 위치가 카메라 광학 +Z축 전방 300 mm에
오도록 관측 자세로 먼저 이동하고 `search_zone=0` 탐지를 한 번 요청합니다.
현재 카메라 방향은 유지하며 TCP-to-camera 보정도 적용합니다. 따라서 정상적인
하향 관측 자세에서는 카메라가 물체 위쪽에 위치합니다.
DB 위치에서 찾지 못하거나 위치가 작업공간 밖이거나 MoveIt 계획에 실패하면
작업을 종료하지 않고 기존 1~6번 탐색구역을 순서대로 진행합니다.

DB 우선 탐색의 높이와 허용 범위는 `SearchConfig`의
`db_search_camera_clearance_mm`, `db_search_workspace_min_xyz_mm`,
`db_search_workspace_max_xyz_mm`에서 실측 환경에 맞게 조정할 수 있습니다.

`green_box` 뚜껑 파지점은 탐지된 CAD 원점에서 물체 로컬 좌표계 기준
`(X, Y, Z) = (0, 0, +40) mm` 오프셋을 적용합니다. v1.5.5보다 물체 로컬
Z 방향으로 10 mm 높은 파지점입니다. 이 최종 파지점을
카메라→Base 변환 직후 Z 안전검사에 사용하며, raw CAD 원점 Z와 오프셋 적용 후
`Object Z`를 함께 로그로 남깁니다.

`green_box` 내부에서 요청 물체를 찾은 경우에는 물체를 사용자에게 전달하고 RG2를
해제한 뒤 2초간 대기합니다. 이후 저장된 뚜껑 이동 경로를 역으로 실행해 뚜껑을
원래 위치에 배치하고 Home으로 복귀한 다음에만 작업 완료 및 대기 상태로
전환합니다. 내부에서 물체를 찾지 못한 경우에는 뚜껑을 즉시 닫고 탐색을 계속하며,
이후 다른 구역에서 물체를 찾아도 뚜껑 복구 동작을 다시 실행하지 않습니다.

`gray_box`는 `sliding_drawer_test_node`의 기하와 순서를 제어 작업에
통합했습니다. Any6D의 Base 기준 서랍 자세에서 손잡이 B를 로컬
`(116.5, 0, -6) mm`로 계산하고, A(50 mm 접근), C(로컬 +X 155 mm 당김),
D(40 mm 후퇴)를 만듭니다. 가능한 접근 방위각을 `0, -1, +1 ... +/-20 deg`
순서로 IK 검사한 후 선택하며, 서랍을 연 뒤 150 mm 수직 상승을 거쳐 툴 +Z가
Base -Z를 향하는 높이 380 mm 관측 자세로 이동합니다.

내부 물체를 찾으면 전달 완료 뒤 저장한 상승 관절 자세로 복귀하여
`상승점 -> D -> C -> B로 155 mm 밀기 -> RG2 해제 -> A -> Home`의
역경로로 닫습니다. 내부에서 물체를 찾지 못하면 같은 닫기 흐름을 즉시 수행한
뒤 다음 탐색 구역으로 진행합니다. 닫기 실패 상태에서는 다음 작업 수락과 이미
대기 중인 작업 실행을 모두 차단합니다.

## 반드시 실제 URDF/SRDF와 맞출 값

`control_node/config.py`의 `MoveItConfig`를 먼저 확인하십시오.

- `planning_group`: 기본 `manipulator`
- `base_frame`: 기본 `base_link`
- `eef_link`: RG2 실제 TCP 링크, 기본 `rg2_tcp`
- `joint_names`: 기본 `joint_1` ~ `joint_6`
이 패키지는 Planning Scene을 직접 수정하지 않습니다. 시작 시 고정 장애물을
생성하지 않으며, 일반 파지물·green_box 뚜껑·gray_box 서랍도
`CollisionObject` 또는 `AttachedCollisionObject`로 등록하지 않습니다.
따라서 `/apply_planning_scene` 서비스도 사용하지 않습니다. MoveIt의 로봇 자체
충돌 검사와 외부 노드가 이미 등록한 환경 충돌 검사는 그대로 유지되지만, 운반 중인
물체의 크기는 경로 계획에 반영되지 않습니다.

v1.5.6이 이미 등록한 가상 부착물은 v1.6.0이 삭제하지 않습니다. 버전 교체 직후에는
`control_node`와 `move_group`을 함께 재시작해 기존 Planning Scene 상태를 한 번
초기화하십시오. 이후 v1.6.0은 새 동적 물체를 등록하지 않습니다.

Any6D Pose를 Base 좌표로 변환할 때 입력 `frame_id`, 카메라 기준 XYZ,
mm 환산 배율 및 최종 Base 기준 XYZ를 INFO 로그로 남깁니다. `Object Z ... is
below ...` 오류가 발생하면 이 로그를 이용해 카메라 Pose 단위·프레임,
TCP-to-camera 외부 파라미터 및 실제 작업면 높이를 구분할 수 있습니다.

## 빌드 및 실행

```bash
cd ~/cobot_ws
colcon build --packages-select control_node
source install/setup.bash

# 먼저 M0609 ros2_control + move_group를 실행
ros2 launch control_node control_node.launch.py
```

기존 `robot_control`과 동일한 제어 노드명 및 서비스/액션 이름을 사용하므로 두
패키지를 동시에 실행하면 안 됩니다.

## 실패 복구

계획 실패, IK 실패, Cartesian fraction 부족, 실행 실패는 예외로 전환됩니다.
제어 노드는 다음 순서로 처리합니다.

1. 현재 작업을 실패로 확정
2. 현재 MoveIt 환경을 사용해 충돌회피 홈 계획 시도
3. 복귀 성공 여부와 원인을 `/state/robot_result`에 포함
4. 작업 큐 항목 종료 후 다음 요청 대기

v1.6.0은 `holding_object`와 Planning Scene 부착 상태를 비교해 자동 복구를
차단하지 않습니다. 작업 중 오류가 발생하면 그리퍼 상태와 관계없이 MoveIt Home
계획을 시도하고, 계획 또는 실행 자체가 실패한 경우에만 복구 실패로 보고합니다.
운반 물체가 Planning Scene에 없으므로 실제 물체와 바닥·주변 설비 사이의 여유는
동작 Pose와 경로 설계에서 별도로 확보해야 합니다.

## v1.6.1 작업 수락 정책

`holding_object`는 RG2 파지·해제 및 사용자 전달 대기 상태를 관리하고 상태 응답에
표시하기 위해 유지합니다. 다만 이 값이 `True`라는 이유만으로 `/control/task` 요청,
`/control/search` 액션 목표 또는 대기열 작업 실행을 차단하지 않습니다.

`green_box` 덮개나 `gray_box` 서랍의 복구 경로가 남아 있는 경우에는 기존과 같이
새 작업 수락 및 대기열 실행을 차단합니다.
