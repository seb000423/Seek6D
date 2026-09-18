"""통합 실행 launch — db / back_ui / state 3개 노드를 한 번에 띄운다.

비전·제어(로봇)·MoveIt·음성은 여기 안 넣는다 — conda 환경이나
실제 하드웨어(로봇 팔, 마이크)가 있어야 떠서 각자 별도 터미널에서 실행해야
하기 때문이다(예: `ros2 launch dsr_moveit_config_m0609 start_2.launch.py ...`
후 `ros2 run robot_control_moveit robot_control_any6d_moveit`, 또는
`ros2 run robot_control robot_control_any6d` — 둘 중 하나만, 동시 실행 금지).

state_node의 targets/search_action 파라미터는 코드 기본값과 이미 같지만
(state_node.py 참고) 여기서도 명시해 둔다 — launch 파일만 보고도 무엇을
기다리는 노드인지 알 수 있게, 그리고 나중에 기본값이 바뀌어도 이 launch가
쓰는 값은 여기서 따로 고정하고 싶을 수 있어서다.

실행:
    ros2 launch state seek6d.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='db',
            executable='db_node',
        ),
        Node(
            package='back_ui',
            executable='node',
        ),
        Node(
            package='state',
            executable='state_node',
            parameters=[{
                'targets': ['db', 'control'],
                'search_action': '/control/search',
            }],
        ),
    ])
