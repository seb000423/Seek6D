"""상태 관리 노드 — 2단계.

부팅 시 다른 노드 준비 확인 + 음성 명령을 받아 탐색 액션으로 넘긴다.

순서도 대응:
  시작 -> State = LOAD -> 각 노드에 상태 확인 요청
       -> 성공? -> No 면 재시도 / Yes 면 State = IDLE -> 대기
       -> 명령 수신 -> 상태 확인 -> IDLE 이면 State = RUN + 액션 발행
                                  아니면 거절

상태
  LOAD  부팅 중. 다른 노드 준비를 기다리는 중
  IDLE  대기. 이 상태에서만 명령을 받는다
  RUN   작업 중. 새 명령은 거절한다

제공
  service  /state/target_search  interfaces/TargetSearch  음성 노드의 명령 접수
  service  /state/robot_result   interfaces/RobotResult   제어 노드의 진행 보고 수신
  topic    /state/current        std_msgs/String          현재 상태 (JSON)
  topic    /ui/task_state        std_msgs/String          작업 상태 (JSON, back_ui 구독)

사용
  service  /<노드>/init          interfaces/NodeInit      각 노드의 준비 확인
  action   /control/search       interfaces/Search        탐색 실행 지시
  service  /db/save              interfaces/DbSave        작업 종료 시 기록(table="tasks")

제어 노드 진행 보고 중계
  제어 노드가 /state/robot_result 로 진행 상황(RobotResult.srv, request에
  JSON)을 push 하면 on_robot_result() 가 그대로 받아 publish_ui_task() 로
  /ui/task_state 에 중계한다. 응답은 즉시 success=True 로만 돌려준다 —
  여기서 다른 서비스를 동기 호출하면 제어 노드의 작업 루프가 그만큼
  멈춘다(제어 쪽이 이 응답을 동기로 기다리는 구조).

작업기록(tasks) 저장 — db 저장 로그 명세 2026-08-06 참고
  탐색 goal 이 수락된 순간 self.current_task 에 dict 하나를 만든다
  (command_text/voice_command/target_name/destination/started_at). action 결과가 오면
  status/fail_stage/fail_reason/found_at/ended_at 을 채워서 /db/save 를
  call_async 로 1회 호출한다 — 서비스 콜백 안에서 동기 call() 을 쓰면
  SingleThreadedExecutor 기준 데드락이라 반드시 비동기로 던지고
  add_done_callback 으로만 결과를 본다(spin_until_future_complete 금지).

  TargetSearch.srv/Search.action 에 원문 음성 명령이나 배송 목적지 필드가
  없어서(interfaces 패키지는 안 건드림) command_text/destination 은 지금
  항상 None 으로 저장된다 — 그 필드가 필요하면 인터페이스부터 넓혀야 한다.
  fail_stage 도 지금은 탐색(SEARCH) 단계만 존재해서 실패하면 항상
  'SEARCH' 다 — 파지/배송 단계가 생기면 그때 단계 추적을 넓히면 된다.
"""

import json
import time
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from interfaces.action import Search
from interfaces.srv import DbSave, NodeInit, RobotResult, TargetSearch


# 상태 정의
LOAD = 'LOAD'
IDLE = 'IDLE'
RUN = 'RUN'


def now_iso() -> str:
    """db_node.py 와 형식을 맞춘다(ISO8601, 초 단위)."""
    return datetime.now().isoformat(timespec='seconds')


class StateNode(Node):

    # 제어 노드 outcome -> front_ui 가 보여줄 한 줄 설명.
    # 새 outcome 이 생기면 여기에 추가한다. 없으면 message 를 그대로 쓴다.
    _OUTCOME_REASON = {
        'queued': '작업 요청을 접수했습니다.',
        'db_lookup': 'DB에서 마지막 위치를 조회하는 중입니다.',
        'waiting_pose': '비전 노드의 자세 응답을 기다리는 중입니다.',
        'searching': '다음 탐색 구역으로 이동하는 중입니다.',
        'zone_not_found': '이 구역에서는 찾지 못했습니다.',
        'landmark_searching': '상자를 찾는 중입니다.',
        'landmark_found': '상자를 여는 중입니다.',
        'landmark_not_found': '상자 안에서도 찾지 못했습니다.',
        'pick_completed': '물건을 잡고 복귀했습니다.',
        'not_found': '모든 구역을 탐색했지만 찾지 못했습니다.',
        'blocked_holding_object': '이전 물건을 들고 있어 시작할 수 없습니다.',
        'rejected': '요청이 거절되었습니다.',
        'failed': '오류로 작업이 중단되었습니다.',
    }

    def __init__(self):
        super().__init__('state_node')

        # 확인할 노드 목록. 노드가 늘어나면 여기에 추가한다
        # 비전 노드는 아직 NodeInit 서버가 없어서 제외 — 생기면 'vision' 추가
        self.declare_parameter('targets', ['db', 'control'])
        self.declare_parameter('wait_timeout', 5.0)     # 서비스 등장 대기 [s]
        self.declare_parameter('retry_period', 2.0)     # 재시도 간격 [s]
        self.declare_parameter('max_retries', 0)        # 0 이면 무한 재시도
        # 제어 노드가 제공하는 Search 액션. dsr01 네임스페이스와 무관한
        # 절대명이라 앞에 '/'를 붙여둔다(상대명이면 네임스페이스가 붙어 어긋남)
        self.declare_parameter('search_action', '/control/search')
        self.declare_parameter('goal_accept_timeout', 3.0)

        self.targets = list(self.get_parameter('targets').value)
        self.wait_timeout = self.get_parameter('wait_timeout').value
        self.retry_period = self.get_parameter('retry_period').value
        self.max_retries = self.get_parameter('max_retries').value
        self.search_action = self.get_parameter('search_action').value
        self.goal_accept_timeout = self.get_parameter('goal_accept_timeout').value

        self.state = LOAD
        self.ready = {name: False for name in self.targets}
        self.current_goal = None        # 진행 중인 탐색 goal handle
        self.current_task = None        # 진행 중인 작업기록 dict (db 저장용)

        cb = ReentrantCallbackGroup()

        # 현재 상태 발행. 늦게 뜨는 노드도 마지막 상태를 바로 받도록 TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            String, 'state/current',
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self.init_clients = {
            name: self.create_client(NodeInit, f'{name}/init',
                                     callback_group=cb)
            for name in self.targets
        }

        # 음성 노드가 부르는 명령 접수 창구
        self.create_service(
            TargetSearch, 'state/target_search',
            self.on_target_search, callback_group=cb)

        # 제어 노드가 작업 진행 상황을 밀어 넣는 창구.
        # 받은 내용을 그대로 /ui/task_state 로 중계한다.
        self.create_service(
            RobotResult, 'state/robot_result',
            self.on_robot_result, callback_group=cb)

        # back_ui 가 구독하는 작업 상태 토픽. 늦게 뜬 UI 도 마지막 상태를
        # 바로 받도록 state/current 와 같은 TRANSIENT_LOCAL 을 쓴다
        self.ui_task_pub = self.create_publisher(
            String, 'ui/task_state',
            QoSProfile(depth=1,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # 실제 탐색을 시킬 액션 클라이언트
        self.search_cli = ActionClient(
            self, Search, self.search_action, callback_group=cb)

        # 작업 종료 시 tasks 테이블에 기록할 때 쓴다 (call_async 전용,
        # 서비스 준비 여부를 startup_check 대상에 넣진 않았다 — db 가
        # 이미 targets 기본값에 있어서 그때 같이 확인된다)
        self.db_save_cli = self.create_client(
            DbSave, 'db/save', callback_group=cb)

        self.publish_state()
        self.get_logger().info(f'확인 대상: {self.targets}')
        self.startup_check()

    def set_state(self, new_state, reason=''):
        old = self.state
        self.state = new_state
        self.publish_state(reason)
        if old != new_state:
            self.get_logger().info(
                f'상태 전이: {old} -> {new_state}'
                + (f' ({reason})' if reason else ''))

    def publish_state(self, reason=''):
        msg = String()
        msg.data = json.dumps(
            {'state': self.state, 'ready': self.ready, 'reason': reason},
            ensure_ascii=False)
        self.state_pub.publish(msg)

    # 부팅 확인 (순서도의 LOAD 구간)
    def startup_check(self):
        """전부 준비될 때까지 재시도한다. 되면 IDLE 로 넘어간다."""
        attempt = 0

        while rclpy.ok():
            attempt += 1

            # 아직 준비 안 된 노드만 다시 물어본다
            for name in self.targets:
                if not self.ready[name]:
                    self.ready[name] = self.check_one(name)

            if all(self.ready.values()):
                self.set_state(IDLE, '전체 노드 준비 완료')
                return

            not_ready = [n for n, ok in self.ready.items() if not ok]

            if self.max_retries and attempt >= self.max_retries:
                self.get_logger().error(
                    f'{attempt}회 시도 후 포기 — 미준비: {not_ready}')
                return

            self.get_logger().warn(
                f'[{attempt}회차] 미준비: {not_ready} — '
                f'{self.retry_period}초 뒤 재시도')
            self.publish_state('재시도 대기')
            time.sleep(self.retry_period)

    # 명령 접수 (게이트키퍼)
    def on_target_search(self, request, response):
        """음성 노드의 탐색 명령. 상태를 보고 받을지 거절할지 판단한다."""
        name = request.target_name.strip()
        label = request.class_label.strip()
        self.get_logger().info(f"명령 수신: target='{name}' class='{label}'")

        if not name and not label:
            response.success = False
            response.message = 'target_name 과 class_label 이 모두 비어 있습니다'
            self.get_logger().warn(f'거절: {response.message}')
            return response

        if self.state != IDLE:
            response.success = False
            response.message = f'지금은 {self.state} 상태라 명령을 받을 수 없습니다'
            self.get_logger().warn(f'거절: {response.message}')
            return response

        if not self.search_cli.server_is_ready():
            response.success = False
            response.message = f'탐색 노드 없음 (/{self.search_action})'
            self.get_logger().error(f'거절: {response.message}')
            return response

        handle = self.send_search_goal(name, label)
        if handle is None:
            response.success = False
            response.message = '탐색 노드가 명령을 받지 않았습니다'
            self.get_logger().error(f'거절: {response.message}')
            return response

        # 여기서 응답은 "접수 완료" 까지만. 결과는 액션으로 따로 돌아온다
        self.current_goal = handle
        self.current_task = {
            'command_text': None,   # TargetSearch.srv에 원문이 없어서 못 채움
            # voice 노드가 target_name 필드에 원문 음성 텍스트를 그대로
            # 담아서 보낸다(voice_command_client.py 참고) — name 은 위에서
            # 이미 strip() 을 거쳐서 비어 있어도 항상 문자열이다(voice 없이
            # 테스트 요청을 보내면 빈 문자열이 그대로 저장됨).
            'voice_command': name,
            'target_name': name or label,
            'destination': None,    # 지금 인터페이스엔 배송 목적지 필드가 없음
            'stage': 'SEARCH',      # 지금은 이 단계만 존재
            'started_at': now_iso(),
        }
        self.set_state(RUN, f"탐색 시작: {name or label}")

        response.success = True
        response.message = f"'{name or label}' 탐색을 시작합니다"
        return response

    # 탐색 액션 발행
    def send_search_goal(self, name, label):
        """탐색 goal 을 보낸다. 수락되면 goal handle, 아니면 None."""
        goal = Search.Goal()
        goal.target_name = name
        goal.class_label = label

        send_future = self.search_cli.send_goal_async(
            goal, feedback_callback=self.on_search_feedback)

        # 서비스 콜백 안이라 spin_until_future_complete 를 쓰면 교착이 생긴다
        deadline = time.time() + self.goal_accept_timeout
        while not send_future.done() and time.time() < deadline:
            time.sleep(0.02)

        if not send_future.done():
            self.get_logger().error('탐색 goal 응답 시간 초과')
            return None

        handle = send_future.result()
        if not handle.accepted:
            return None

        # 결과가 오면 IDLE 로 되돌린다
        handle.get_result_async().add_done_callback(self.on_search_result)
        return handle

    def on_search_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'  탐색 중: {fb.step} ({fb.progress * 100:.0f}%)')

    def on_search_result(self, future):
        """탐색이 끝나면 호출된다. 성공/실패/취소 모두 여기로 온다."""
        self.current_goal = None
        task = self.current_task
        self.current_task = None
        ended_at = now_iso()

        try:
            wrapped = future.result()
            res = wrapped.result
        except Exception as e:      # noqa: BLE001
            self.get_logger().error(f'탐색 결과 수신 실패: {e}')
            self._save_task(task, status='FAILED', ended_at=ended_at,
                            fail_reason=str(e))
            self.set_state(IDLE, '탐색 실패')
            return

        if wrapped.status == GoalStatus.STATUS_CANCELED:
            status = 'ABORTED'
        elif res.success:
            status = 'SUCCEEDED'
        else:
            status = 'FAILED'

        if status == 'SUCCEEDED':
            self.get_logger().info(f'탐색 완료: {res.message} @ {res.location}')
        else:
            self.get_logger().warn(f'탐색 실패: {res.message}')

        self._save_task(
            task, status=status, ended_at=ended_at,
            found_at=res.location or None,
            fail_reason=None if status == 'SUCCEEDED' else res.message)

        # 제어 노드가 completed 이벤트를 이미 보냈을 수도 있지만, 액션이
        # abort 되면 그 이벤트가 누락될 수 있어 여기서 최종 상태를 확정한다
        self.publish_ui_task({
            'status': 'completed',
            'success': status == 'SUCCEEDED',
            'outcome': 'pick_completed' if status == 'SUCCEEDED' else 'failed',
            'message': res.message,
            'task_id': task['target_name'] if task else None,
            'target': {'name': task['target_name'] if task else None,
                       'class_label': None},
        })

        self.set_state(IDLE, '탐색 종료')

    # 작업기록 저장 (db 저장 로그 명세 2026-08-06)
    def _save_task(self, task, status, ended_at, found_at=None, fail_reason=None):
        """tasks 테이블에 1행 기록한다. call_async만 쓴다 — 결과를
        기다렸다가 다음 동작을 하면 안 된다(데드락 주의, 파일 상단 참고).
        """
        if task is None:
            # 거절된 요청(게이트키퍼 단계)은 애초에 current_task 를 안
            # 만들어서 여기 안 온다 — 방어적으로만 막아둔다.
            return

        row = {
            'command_text': task['command_text'],
            'voice_command': task['voice_command'],
            'target_name': task['target_name'],
            'destination': task['destination'],
            'status': status,
            'fail_stage': None if status == 'SUCCEEDED' else task['stage'],
            'fail_reason': fail_reason,
            'found_at': found_at,
            'started_at': task['started_at'],
            'ended_at': ended_at,
        }

        req = DbSave.Request()
        req.request = json.dumps(
            {'table': 'tasks', 'rows': [row]}, ensure_ascii=False)

        future = self.db_save_cli.call_async(req)
        future.add_done_callback(self._on_task_saved)

    def _on_task_saved(self, future):
        try:
            res = future.result()
        except Exception as e:      # noqa: BLE001
            self.get_logger().error(f'[작업기록] 저장 요청 실패: {e}')
            return

        log = self.get_logger().info if res.success else self.get_logger().error
        log(f'[작업기록] {res.message}')

    # 제어 노드 진행 보고 수신 -> UI 중계
    def on_robot_result(self, request, response):
        """제어 노드의 진행 보고. 받아서 UI 로 중계만 하고 바로 응답한다.

        여기서 무거운 일을 하면 제어 노드의 작업 루프가 그만큼 멈춘다
        — 제어는 이 서비스 응답을 기다린 뒤 다음 동작으로 넘어간다.
        """
        try:
            ev = json.loads(request.request) if request.request.strip() else {}
        except json.JSONDecodeError as e:
            response.success = False
            response.response = '{}'
            response.message = f'JSON 파싱 실패: {e}'
            return response

        self.publish_ui_task(ev)

        # 진행 단계를 작업기록에 반영해 둔다 — 실패 시 fail_stage 가
        # 'SEARCH' 고정이 아니라 실제 단계로 남는다
        if self.current_task is not None:
            outcome = ev.get('outcome')
            if outcome:
                self.current_task['stage'] = outcome.upper()

        response.success = True
        response.response = '{}'
        response.message = 'ok'
        return response

    def publish_ui_task(self, ev):
        """제어 이벤트 -> back_ui 의 task 스키마로 바꿔 발행한다.

        back_ui 의 StateStore.update_task 는 None 인 필드를 무시하므로
        모르는 값은 굳이 채우지 않고 None 으로 둔다.
        """
        status_raw = ev.get('status')
        if status_raw == 'completed':
            status = 'SUCCESS' if ev.get('success') else 'FAILED'
        else:
            status = 'RUNNING'

        target = ev.get('target') or {}
        zone = ev.get('search_zone')

        elapsed = None
        if self.current_task is not None:
            try:
                elapsed = int(
                    (datetime.now()
                     - datetime.fromisoformat(self.current_task['started_at'])
                     ).total_seconds())
            except ValueError:
                pass

        payload = {
            'task_id': ev.get('task_id'),
            'target_id': target.get('class_label'),
            'target_name': target.get('name'),
            # TargetSearch.srv 에 원문 음성이 없어서 아직 못 채운다
            'voice_command': None,
            'status': status,
            'stage': ev.get('outcome'),
            'action': ev.get('message'),
            'action_reason': self._OUTCOME_REASON.get(ev.get('outcome')),
            'elapsed_sec': elapsed,
            'current_zone': f'{zone}구역' if zone else None,
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.ui_task_pub.publish(msg)

    def check_one(self, name):
        """노드 하나에 준비 확인을 보낸다. 준비됐으면 True."""
        cli = self.init_clients[name]

        if not cli.wait_for_service(timeout_sec=self.wait_timeout):
            self.get_logger().warn(f'[{name}] 서비스 없음 (/{name}/init)')
            return False

        req = NodeInit.Request()
        req.request = json.dumps({'node': 'state_node'}, ensure_ascii=False)

        future = cli.call_async(req)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self.wait_timeout)

        if not future.done():
            self.get_logger().warn(f'[{name}] 응답 시간 초과')
            return False

        res = future.result()
        if not res.success:
            self.get_logger().warn(f'[{name}] 준비 안 됨 — {res.message}')
            return False

        self.get_logger().info(f'[{name}] {res.message}')

        # 노드별 상태 JSON. 내용은 노드마다 다르므로 로그로만 남긴다
        try:
            status = json.loads(res.response) if res.response else {}
            if status:
                self.get_logger().info(f'       {status}')
        except json.JSONDecodeError:
            self.get_logger().warn(f'[{name}] 상태 JSON 파싱 실패')

        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StateNode()
        # 서비스 콜백 안에서 액션 goal 을 보내야 해서 멀티스레드가 필요하다
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:      # noqa: BLE001
        print(f'state_node 종료: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
