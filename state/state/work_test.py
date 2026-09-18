"""더미 탐색 노드 (임시).

실제 작업 노드가 만들어지기 전까지 /item/search 액션 서버 역할을 대신한다.
상태 노드의 LOAD -> IDLE -> RUN -> IDLE 흐름과 취소 동작을 확인하는 용도.
실제 노드가 생기면 이 파일은 지우면 된다.

제공
  service  /item/init     interfaces/NodeInit   준비 확인 (상태 노드가 호출)
  action   /item/search   interfaces/Search     탐색 실행

파라미터
  duration   탐색에 걸리는 시간 [s]        기본 5.0
  found      찾았다고 응답할지             기본 True
  location   찾았을 때 돌려줄 위치          기본 '주방 싱크대'
  ready      init 요청에 준비됐다고 할지    기본 True
"""

import json
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from interfaces.action import Search
from interfaces.srv import NodeInit


# 탐색하는 척하는 단계들
STEPS = ('이동 준비', '목표 지점 이동', '주변 스캔', '대상 확인')


class DummySearchNode(Node):

    def __init__(self):
        super().__init__('dummy_search_node')

        self.declare_parameter('duration', 5.0)
        self.declare_parameter('found', True)
        self.declare_parameter('location', '주방 싱크대')
        self.declare_parameter('ready', True)

        self.duration = self.get_parameter('duration').value
        self.found = self.get_parameter('found').value
        self.location = self.get_parameter('location').value
        self.ready = self.get_parameter('ready').value

        cb = ReentrantCallbackGroup()

        self.create_service(NodeInit, 'item/init', self.on_init,
                            callback_group=cb)

        self.action_server = ActionServer(
            self, Search, 'item/search',
            execute_callback=self.on_execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=cb)

        self.get_logger().info(
            f'더미 탐색 노드 준비 (duration={self.duration}s, '
            f'found={self.found}, ready={self.ready})')

    def on_init(self, request, response):
        """상태 노드의 준비 확인에 응답한다."""
        response.success = self.ready
        response.response = json.dumps(
            {'ready': self.ready, 'note': 'dummy node'}, ensure_ascii=False)
        response.message = ('더미 탐색 노드 준비됨' if self.ready
                            else '더미 탐색 노드 준비 안 됨 (일부러)')
        self.get_logger().info(f'[init] {response.message}')
        return response

    def on_goal(self, goal_request):
        self.get_logger().info(
            f"goal 수신: target='{goal_request.target_name}' "
            f"class='{goal_request.class_label}'")
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.get_logger().info('취소 요청 수락')
        return CancelResponse.ACCEPT

    def on_execute(self, goal_handle):
        goal = goal_handle.request
        target = goal.target_name or goal.class_label

        result = Search.Result()
        tick = 0.5
        total = max(1, int(self.duration / tick))

        for i in range(total):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.location = ''
                result.message = '사용자 취소'
                self.get_logger().info('탐색 취소됨')
                return result

            progress = (i + 1) / total
            fb = Search.Feedback()
            fb.step = STEPS[min(int(progress * len(STEPS)), len(STEPS) - 1)]
            fb.progress = float(progress)
            goal_handle.publish_feedback(fb)

            time.sleep(tick)

        goal_handle.succeed()

        if self.found:
            result.success = True
            result.location = self.location
            result.message = f"'{target}' 을(를) 찾았습니다"
        else:
            result.success = False
            result.location = ''
            result.message = f"'{target}' 을(를) 찾지 못했습니다"

        self.get_logger().info(f'탐색 종료: {result.message}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DummySearchNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:      # noqa: BLE001
        print(f'dummy_search_node 종료: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
