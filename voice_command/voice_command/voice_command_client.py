#!/usr/bin/env python3
"""STT + GPT object-name resolver and ROS 2 service client.

No application topics are used. The original user command and the resolved
model ID are sent to the state node through interfaces/srv/TargetSearch.
target_name carries voice_command, and class_label carries the resolved class
ID or the string "null" for an unregistered object.
"""
from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path

import numpy as np
import rclpy
import scipy.io.wavfile as wav
import sounddevice as sd
import pyaudio
from scipy.signal import resample
from openwakeword.model import Model
from dotenv import load_dotenv
from openai import OpenAI
from rclpy.node import Node

from interfaces.srv import TargetSearch
from ament_index_python.packages import get_package_share_directory


VALID_MODEL_NAMES = {
    "yellow_can",
    "green_box",
    "gray_box",
    "white_bear",
    "aircon_remote",
    "green_frog",
    "otter_in_can",
}

MODEL_DESCRIPTIONS = {
    "yellow_can": "노란색 원통형 캔",
    "green_box": "초록색 직육면체 상자",
    "gray_box": "회색 직육면체 상자/수납함",
    "white_bear": "흰색/베이지색 곰 또는 쿼카 인형",
    "aircon_remote": "회색 에어컨 리모컨",
    "green_frog": "초록색 개구리 인형",
    "otter_in_can": "통 안에 있는 흰색/아이보리 수달 인형",
}

default_env_file = (
    Path(get_package_share_directory("voice_command"))
    / "resource"
    / ".env"
)

default_wakeword_model = (
    Path(get_package_share_directory("voice_command"))
    / "resource"
    / "hello_rokey_8332_32.tflite"
)

class ObjectNameResolver:
    def __init__(self, api_key: str, model: str) -> None:
        self.client = OpenAI(api_key=api_key)
        self.model = model

    def resolve(self, user_text: str) -> tuple[str | None, str, str]:
        prompt = f"""
너는 로봇이 찾을 물체 이름을 해석하는 분류기다.
사용자 입력을 아래 7개 등록 물체 중 정확히 하나로 분류하거나, 해당하는 물체가 없으면 null로 분류하라.

[오타 및 STT 음성인식 오류 보정 규칙]
사용자 입력은 음성 인식(STT) 결과일 수 있으므로 철자, 받침, 음절 일부가 틀릴 수 있다.
등록된 물체 이름과 발음상/문맥상 매우 유사한 경우에는 자연스러운 한국어 표현으로 보정해서 판단하라.

예:
- "초독 개구디", "초록 개구디", "초록 개구이", "개구디" -> "초록 개구리" 또는 "개구리" -> green_frog
- "노란 캥", "노랑 캔", "노란 깡똥", "깡통" -> yellow_can
- "회색 박쓰", "회색 빡스", "그레이 박쓰" -> gray_box
- "초록 박쓰", "녹색 빡스" -> green_box
- "리모컹", "리모콘", "에어컨 리모컹" -> aircon_remote
- "쿼까", "곰이녕", "곰 인녕" -> white_bear
- "수달이녕", "수달 인녕", "목욕하는 수달이녕" -> otter_in_can

단, 오타 보정은 등록 물체를 억지로 선택하기 위해 사용하지 않는다.
특히 사용자가 명확하게 말한 색상이나 물체 종류가 등록 물체와 충돌하면
그 부분을 STT 오류라고 임의로 간주하지 말고 null로 판단한다.

예:
- "초록 캔", "녹색 캔" -> null
- "빨간 캔" -> null
- "노란 개구리" -> null
- "회색 개구리" -> null
- "파란 박스" -> null
- "노란 상자" -> null

[가장 중요한 판정 규칙]
1. 사용자가 색상을 명시했다면 그 색상 조건을 반드시 우선한다.
2. 등록 물체의 색과 사용자가 말한 색이 충돌하면 절대로 그 등록 물체로 분류하지 말고 null을 출력한다.
   예:
   - "초록 캔", "녹색 캔" -> yellow_can 아님 -> null
   - "빨간 캔" -> yellow_can 아님 -> null
   - "노란 상자" -> green_box/gray_box 아님 -> null
   - "회색 개구리" -> green_frog 아님 -> null
3. 사용자가 색상을 말하지 않은 경우에만, 아래에 명시된 일반 명칭/동의어를 해당 등록 물체로 해석할 수 있다.
4. 사용자가 명시한 형태나 물체 종류가 등록 물체와 충돌하면 null이다.
5. "찾아줘", "가져와", "집어줘", "어디 있어", "주세요" 같은 동작 표현은 물체 분류에 영향을 주지 않는다.

[등록 물체]

- yellow_can
  실제 물체: 노란색 원통형 캔.
  허용 표현: 노란 캔, 노랑 캔, 노란색 캔, 노란 통조림 캔, 노란 원통 캔, 캔, 깡통, 캔 음료, 그 캔.
  색상을 말하지 않고 단순히 "캔", "깡통"이라고 하면 yellow_can으로 본다.
  단, 초록/녹색/빨강/파랑/검정/흰색 등 노란색이 아닌 색을 명시한 캔은 null이다.

- green_box
  실제 물체: 초록색 직육면체 상자.
  허용 표현: 초록 상자, 초록 박스, 초록색 상자, 초록색 박스, 녹색 상자, 녹색 박스,
             초록 수납함, 초록 수납장, 초록 케이스.
  색상을 말하지 않고 단순히 "초록색 물건"처럼 불명확하면 분류하지 않는다.
  색상을 말하지 않은 일반 "상자"/"박스"는 green_box와 gray_box가 모두 존재하므로 모호하다. 이 경우 null이다.
  회색/노랑/빨강/파랑 등 초록색이 아닌 상자는 green_box가 아니다.

- gray_box
  실제 물체: 회색 직육면체 상자 또는 회색 수납함.
  허용 표현: 회색 상자, 회색 박스, 회색 수납함, 회색 수납장, 회색 케이스,
             그레이 박스, 그레이 상자, 회색 직육면체 상자.
  회색이 아닌 색을 명시한 상자/박스는 gray_box가 아니다.
  색상이 없는 일반 "상자"/"박스"는 green_box와 구분할 수 없으므로 null이다.

- white_bear
  실제 물체: 흰색 또는 밝은 베이지색 곰/쿼카 형태 인형.
  허용 표현: 흰 곰, 흰색 곰, 하얀 곰, 흰 곰 인형, 하얀 곰 인형, 곰, 곰 인형,
             베이지 곰, 베이지색 곰, 쿼카, 쿼카 인형, 베이지 쿼카, 흰색 쿼카.
  색상을 말하지 않은 "곰", "곰 인형", "쿼카"는 white_bear로 본다.
  검은 곰, 갈색 곰, 핑크 곰처럼 명백히 다른 색을 말하면 null이다.

- aircon_remote
  실제 물체: 회색 에어컨 리모컨.
  허용 표현: 리모컨, 리모콘, 에어컨 리모컨, 에어컨 리모콘, 회색 리모컨,
             회색 에어컨 리모컨, 에어컨 컨트롤러, 에어컨 조종기, 그 리모컨.
  색상을 말하지 않은 일반 "리모컨"/"리모콘"은 aircon_remote로 본다.
  빨간 리모컨, 검은 리모컨 등 회색과 명백하게 충돌하는 색을 말하면 null이다.

- green_frog
  실제 물체: 초록색 개구리 인형.
  허용 표현: 개구리, 개구리 인형, 초록 개구리, 초록색 개구리, 녹색 개구리,
             초록 개구리 인형, 개구리 장난감, 초록색 개구리 장난감.
  색상을 말하지 않은 "개구리"/"개구리 인형"은 green_frog로 본다.
  빨간/노란/파란/흰색 등 초록색이 아닌 개구리를 명시하면 null이다.

- otter_in_can
  실제 물체: 통 안에 들어 있는 흰색/아이보리 계열 수달 인형.
  허용 표현: 수달, 수달 인형, 흰 수달, 하얀 수달, 아이보리 수달,
             통 안 수달, 통에 든 수달, 통 속 수달, 목욕하는 수달,
             목욕하는 수달 인형, 나무통 수달, 배럴 안 수달, 수달 장난감.
  색상을 말하지 않은 "수달"/"수달 인형"은 otter_in_can으로 본다.
  검은 수달, 갈색 수달 등 등록 물체와 명백히 다른 색을 명시하면 null이다.

[판정 예시]
- "캔 찾아줘" -> yellow_can
- "노란 깡통 가져와" -> yellow_can
- "초록 캔 찾아줘" -> null
- "빨간 캔 집어줘" -> null
- "초록 박스 가져와" -> green_box
- "회색 박스 찾아" -> gray_box
- "박스 찾아줘" -> null
- "상자 가져와" -> null
- "쿼카 어디 있어" -> white_bear
- "갈색 곰 가져와" -> null
- "리모콘 줘" -> aircon_remote
- "개구리 찾아줘" -> green_frog
- "노란 개구리 찾아줘" -> null
- "목욕하는 수달 찾아줘" -> otter_in_can

사용자 입력: {user_text}

출력 규칙:
- 반드시 JSON 객체 하나만 출력한다.
- model_name은 아래 7개 ID 중 하나 또는 null이다.
- normalized_text에는 오타/STT 오류를 자연스럽게 보정한 문장을 넣는다.
- 보정할 필요가 없으면 normalized_text에는 사용자 입력과 같은 문장을 넣는다.
- 판단이 애매하거나 등록 물체와 색/형태가 충돌하면 추측하지 말고 null을 출력한다.
- reason은 판단 이유를 한국어 한 문장으로 짧게 쓴다.

{{"model_name":"yellow_can|green_box|gray_box|white_bear|aircon_remote|green_frog|otter_in_can 중 하나 또는 null", "normalized_text":"보정된 한국어 문장", "reason":"한국어 한 문장"}}
""".strip()
        response = self.client.responses.create(model=self.model, input=prompt, temperature=0)
        raw = response.output_text.strip()
        start, end = raw.find("{"), raw.rfind("}")
        if start < 0 or end < start:
            raise ValueError(f"GPT JSON 응답을 찾지 못했습니다: {raw}")
        data = json.loads(raw[start : end + 1])
        model_name = data.get("model_name")
        if model_name not in VALID_MODEL_NAMES:
            model_name = None

        normalized_text = str(data.get("normalized_text", "")).strip() or user_text
        reason = str(data.get("reason", "")).strip()
        return model_name, normalized_text, reason

    def transcribe(self, duration: float, sample_rate: int = 16000) -> str | None:
        tmp_path: str | None = None
        print(f"[STT] {duration:.1f}초 동안 말씀하세요...")
        try:
            recording = sd.rec(
                int(duration * sample_rate),
                samplerate=sample_rate,
                channels=1,
                dtype="int16",
            )
            sd.wait()
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
                tmp_path = tmp.name
            wav.write(tmp_path, sample_rate, np.asarray(recording))
            with open(tmp_path, "rb") as audio:
                result = self.client.audio.transcriptions.create(model="whisper-1", file=audio)
            text = result.text.strip()
            print(f"[STT 결과] {text}")
            return text or None
        except Exception as exc:
            print(f"[STT 오류] {exc}")
            return None
        finally:
            if tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass


class WakewordDetector:
    """'Hello Rokey' 웨이크워드가 감지될 때까지 마이크 입력을 감시한다."""

    def __init__(
        self,
        model_path: str,
        threshold: float = 0.3,
        sample_rate: int = 48000,
        buffer_size: int = 24000,
        device_index: int | None = None,
    ) -> None:
        self.model_path = str(Path(model_path).expanduser())
        if not Path(self.model_path).is_file():
            raise FileNotFoundError(f"웨이크워드 모델을 찾지 못했습니다: {self.model_path}")

        self.model_name = Path(self.model_path).stem
        self.threshold = threshold
        self.sample_rate = sample_rate
        self.buffer_size = buffer_size
        self.device_index = device_index

        self.model = Model(wakeword_models=[self.model_path])
        self.audio = None
        self.stream = None

    def open(self) -> None:
        self.audio = pyaudio.PyAudio()
        kwargs = dict(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.buffer_size,
        )
        if self.device_index is not None:
            kwargs["input_device_index"] = self.device_index
        self.stream = self.audio.open(**kwargs)

    def close(self) -> None:
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None
        if self.audio is not None:
            self.audio.terminate()
            self.audio = None

    def wait(self) -> None:
        if self.stream is None:
            self.open()

        print("[WAKEUP] 'Hello Rokey'라고 말해주세요...")
        while True:
            raw = self.stream.read(self.buffer_size, exception_on_overflow=False)
            audio_chunk = np.frombuffer(raw, dtype=np.int16)

            # 기존 hello_rokey 모델 입력에 맞게 48 kHz -> 16 kHz 변환
            audio_16k = resample(
                audio_chunk,
                int(len(audio_chunk) * 16000 / self.sample_rate),
            ).astype(np.int16)

            outputs = self.model.predict(audio_16k)
            confidence = float(outputs.get(self.model_name, 0.0))

            if confidence > self.threshold:
                print(f"[WAKEUP] Hello Rokey 감지! confidence={confidence:.3f}")
                return


class StateTriggerClient(Node):
    def __init__(self, sm_service: str) -> None:
        super().__init__("stt_gpt_trigger_client")
        self.sm_client = self.create_client(TargetSearch, sm_service)

    def trigger_state_machine(
        self,
        voice_command: str,
        model_name: str | None,
        wait_timeout: float,
    ):
        if not self.sm_client.wait_for_service(timeout_sec=wait_timeout):
            raise RuntimeError("상태머신 서비스를 찾지 못했습니다.")

        req = TargetSearch.Request()

        # 중요: GPT가 맞춤법/STT 오인식을 보정했더라도,
        # DB에는 실제 사용자가 말한 원본 문장을 남기기 위해 원문 그대로 전달한다.
        # 예: STT="초독 개구디를 찾아줘" -> class=green_frog여도
        # target_name에는 "초독 개구디를 찾아줘"를 전달한다.
        req.target_name = voice_command

        # 등록 물체면 내부 class ID를 전달하고, 미등록이면 "null" 전달
        # TargetSearch.class_label은 ROS string 필드이므로 None은 직접 넣을 수 없음
        req.class_label = model_name if model_name is not None else "null"

        future = self.sm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()


def main() -> int:
    parser = argparse.ArgumentParser(description="STT + GPT -> TargetSearch ROS 2 service client")
    parser.add_argument("--sm-service", default="/state/target_search")
    parser.add_argument("--env-file", default=str(default_env_file))
    parser.add_argument("--record-seconds", type=float, default=5.0)
    parser.add_argument("--service-wait", type=float, default=10.0)
    parser.add_argument(
        "--wakeword-model",
        default=str(default_wakeword_model),
        help="'Hello Rokey' openWakeWord tflite 모델 경로",
    )
    parser.add_argument("--wakeword-threshold", type=float, default=0.3)
    parser.add_argument("--wakeword-device", type=int, default=None)
    args = parser.parse_args()

    env_file = Path(args.env_file).expanduser()
    load_dotenv(env_file)
    api_key = os.getenv("OPENAI_API_KEY", "").strip()
    if not api_key:
        raise SystemExit(f"OPENAI_API_KEY를 읽지 못했습니다: {env_file}")

    resolver = ObjectNameResolver(api_key, os.getenv("OPENAI_MODEL", "gpt-4.1-mini"))
    rclpy.init()
    node = StateTriggerClient(args.sm_service)
    wakeup = WakewordDetector(
        model_path=args.wakeword_model,
        threshold=args.wakeword_threshold,
        device_index=args.wakeword_device,
    )

    print("음성 명령 모드: 'Hello Rokey' 감지 후 5초 동안 명령을 녹음합니다.")
    try:
        while rclpy.ok():
            try:
                # 웨이크워드용 PyAudio 스트림으로 계속 듣는다.
                wakeup.wait()

                # 같은 마이크를 sounddevice가 사용할 수 있도록
                # 웨이크워드 스트림을 잠시 닫고 5초 STT 녹음을 시작한다.
                wakeup.close()

                text = resolver.transcribe(args.record_seconds) or ""
                if not text:
                    continue

                model_name, normalized_text, reason = resolver.resolve(text)
            except KeyboardInterrupt:
                break
            except Exception as exc:
                print(f"[음성/GPT 오류] {exc}")
                wakeup.close()
                continue

            print(f"[GPT 보정] '{text}' -> '{normalized_text}'")
            print(f"[GPT CLASS] model_name={model_name}; {reason}")

            if model_name is None:
                print(
                    "[미등록] 등록 물체가 아닙니다. "
                    "voice_command는 그대로 전송하고 class는 null로 전송합니다."
                )

            try:
                class_for_log = model_name if model_name is not None else "null"
                print(
                    "[서비스 요청] "
                    f"voice_command='{text}', class='{class_for_log}'"
                )
                sm_resp = node.trigger_state_machine(
                    voice_command=text,
                    model_name=model_name,
                    wait_timeout=args.service_wait,
                )

                is_success = getattr(sm_resp, "success", True)
                msg = getattr(sm_resp, "message", "정상 전송됨")

                if is_success:
                    print(f"[요청 성공] 상태 머신 응답: {msg}")
                else:
                    print(f"[요청 거부] 상태 머신 거절: {msg}")
            except Exception as exc:
                print(f"[서비스 오류] {exc}")
    finally:
        wakeup.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
