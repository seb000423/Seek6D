# vision_nodes

ROS 2 Humble용 비전 패키지입니다.

## 포함 파일

- `dino_any6d_node`: Grounding DINO + Any6D 기반 객체 6D pose 서비스
- `dino_all_object_node`: 전체 객체 검출, robot base 좌표 변환, DB 저장 요청
- `SetPickedObject.srv`: Any6D 노드와 전체 객체 검출 노드 사이의 선택 객체 전달 서비스

## 필요한 같은 워크스페이스 패키지

`interfaces` 패키지에 아래 서비스가 있어야 합니다.

- `interfaces/srv/DetectObject`
- `interfaces/srv/DbSave`
- `interfaces/srv/UpdateTcpPose`

## 외부 Python 환경

아래 항목은 ROS 패키지 설치만으로 자동 설치되지 않습니다.

- GroundingDINO
- Any6D 및 수정된 `estimater.py`
- PyTorch/CUDA
- trimesh
- Pillow
- scipy
- OpenCV
- NumPy

노드는 기존 Any6D/GroundingDINO Conda 환경에서 실행하는 것을 전제로 합니다.

## 설치

```bash
cd ~/cobot_ws/src
unzip vision_nodes.zip
cd ~/cobot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select interfaces vision_nodes --symlink-install
source install/setup.bash
```

Conda 환경에서 ROS Python 패키지를 사용할 경우, 기존에 사용하던 방식대로 ROS 환경과 Conda 환경을 함께 활성화해야 합니다.

## 실행

각 노드 개별 실행:

```bash
ros2 run vision_nodes dino_any6d_node
ros2 run vision_nodes dino_all_object_node -- --gripper-camera ~/Any6D/T_gripper2camera.npy
```

두 노드 launch 실행:

```bash
ros2 launch vision_nodes vision_nodes.launch.py
```

## 서비스 확인

```bash
ros2 interface show vision_nodes/srv/SetPickedObject
ros2 service list | grep -E 'find_object_pose|set_picked_object|update_tcp_pose|db_save'
```

## 주의

현재 업로드된 `SetPickedObject.srv`에는 TCP pose 필드가 있지만, 두 Python 노드는 `model_name`만 직접 사용합니다. `dino_all_object_node`는 TCP를 별도의 `/update_tcp_pose` 서비스로 받습니다. 따라서 TCP 필드는 현재 통신에서 0 기본값이어도 동작하지만, 인터페이스를 단순화하려면 추후 해당 필드를 제거하고 양쪽 패키지를 다시 빌드할 수 있습니다.
