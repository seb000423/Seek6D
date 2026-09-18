# voice_command

웨이크워드("Hello Rokey") 감지 → 음성 녹음 → STT → LLM 기반 물체명
정규화를 거쳐, 상태머신(`state`)에 탐색 명령을 전달하는 노드.

## 동작 흐름

1. `openWakeWord`로 마이크를 계속 감시하다가 "Hello Rokey" 웨이크워드가
   감지되면 5초간 명령을 녹음한다.
2. OpenAI Whisper API로 STT 변환.
3. GPT로 인식된 문장에서 등록 물체명을 추정해 클래스 ID로 정규화.
4. `interfaces/srv/TargetSearch`로 상태머신(`/state/target_search`)에
   요청을 보낸다. **원문 발화 텍스트는 `target_name` 필드에, 정규화된
   클래스 ID(또는 미등록이면 `"null"`)는 `class_label` 필드에 담는다** —
   DB에는 GPT가 보정하기 전 사용자의 실제 발화를 남기기 위함이다.

## 등록 물체

```
yellow_can, green_box, gray_box, white_bear, aircon_remote, green_frog, otter_in_can
```

목록에 없는 물체를 요청하면 `class_label`은 `"null"`로 보내되, 원문
텍스트는 그대로 전달한다(상태머신/DB 쪽에서 미등록 처리).

## 빌드 & 실행

```bash
colcon build --packages-select interfaces voice_command
source install/setup.bash
ros2 run voice_command voice_command_node
```

### 환경 변수

`voice_command/resource/.env`에 OpenAI API 키가 있어야 한다.

```
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4.1-mini   # 선택, 기본값 gpt-4.1-mini
```

### 실행 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--sm-service` | `/state/target_search` | 상태머신 서비스 이름 |
| `--env-file` | `<share>/voice_command/resource/.env` | API 키 파일 경로 |
| `--record-seconds` | `5.0` | 웨이크워드 감지 후 녹음 시간(초) |
| `--service-wait` | `10.0` | 상태머신 응답 대기 시간(초) |
| `--wakeword-model` | `<share>/voice_command/resource/hello_rokey_8332_32.tflite` | openWakeWord 모델 경로 |
| `--wakeword-threshold` | `0.3` | 웨이크워드 판정 임계값 |
| `--wakeword-device` | (자동 선택) | 마이크 장치 인덱스 |

## 필요한 파이썬 패키지

ROS 2가 쓰는 시스템 파이썬(또는 그와 같은 인터프리터)에 아래 패키지가
설치되어 있어야 한다. 별도 conda 환경을 쓰지 않는다.

```bash
pip install openai python-dotenv sounddevice scipy numpy pyaudio openwakeword
```

## 제공 인터페이스

이 노드는 서비스를 제공하지 않고, `interfaces/srv/TargetSearch`의
클라이언트로만 동작한다.
