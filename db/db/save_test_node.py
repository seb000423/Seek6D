"""DB 테스트 노드 (대화형).

터미널에서 숫자를 입력해 DB 노드의 서비스를 호출한다.

  ros2 run db test_node

  1) 테스트 데이터 생성   -> /db/save
  2) 전체 목록 출력       -> /db/load  {}
  3) class_name 으로 검색 -> /db/load  {"class_name": "green_frog"}
  4) 직접 등록            -> /db/save  (입력받은 1건)
  8) 전체 삭제            -> /db/clear (되돌릴 수 없음)
  9) 초기화 확인          -> /db/init
  0) 종료
"""

import json

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from interfaces.srv import DbLoad, DbSave, NodeInit


# 탐색 작업 1회분 = 리스트 1개. 호출할 때마다 다음 회차가 나간다.
# class_name 은 voice_command 가 아는 등록 물체 6종 중에서 썼다(x/y/z는
# 비전 좌표계 예시값, 단위는 비전이 실제로 주는 그대로다 — mm로 보임).
TEST_RUNS = [
    # 1회차 — 3개 신규 등록
    [
        {'class_name': 'green_frog',    'confidence': 0.92, 'x': 412.5, 'y': -135.8, 'z': 125.3},
        {'class_name': 'white_bear',    'confidence': 0.88, 'x': 505.2, 'y': 82.4,   'z': 118.9},
        {'class_name': 'aircon_remote', 'confidence': 0.75, 'x': 300.0, 'y': 10.0,   'z': 90.0},
    ],
    # 2회차 — green_frog는 이동(갱신), yellow_can은 신규, white_bear는 못 봄(그대로 유지)
    [
        {'class_name': 'green_frog', 'confidence': 0.81, 'x': 380.0, 'y': -90.0, 'z': 120.0},
        {'class_name': 'yellow_can', 'confidence': 0.69, 'x': 250.0, 'y': 60.0,  'z': 95.0},
    ],
    # 3회차 — 같은 class_name이 배치 안에 두 번. 뒤에 것이 최종 위치가 된다
    [
        {'class_name': 'green_frog', 'confidence': 0.70, 'x': 100.0, 'y': 0.0,  'z': 110.0},
        {'class_name': 'green_frog', 'confidence': 0.85, 'x': 120.0, 'y': 20.0, 'z': 112.0},
    ],
]

MENU = """
==================================
  1) 테스트 데이터 생성 (다음 회차)
  2) 전체 목록 출력
  3) class_name 으로 검색
  4) 직접 등록
  8) 전체 삭제 (되돌릴 수 없음)
  9) 초기화 확인 (/db/init)
  0) 종료
==================================
선택: """


class DBTestNode(Node):

    def __init__(self):
        super().__init__('db_test_node')

        self.save_cli = self.create_client(DbSave, 'db/save')
        self.load_cli = self.create_client(DbLoad, 'db/load')
        self.init_cli = self.create_client(NodeInit, 'db/init')
        self.clear_cli = self.create_client(Trigger, 'db/clear')

        self.run_index = 0      # 다음에 보낼 테스트 회차

        self.get_logger().info('DB 노드 기다리는 중...')
        for cli, name in ((self.save_cli, 'save'),
                          (self.load_cli, 'load'),
                          (self.init_cli, 'init'),
                          (self.clear_cli, 'clear')):
            if not cli.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'/db/{name} 서비스를 찾을 수 없습니다')
        self.get_logger().info('연결 완료')

    def call(self, client, request):
        """서비스를 호출하고 응답이 올 때까지 기다린다."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            print('  !! 응답 시간 초과')
            return None
        return future.result()

    def send_save(self, rows):
        req = DbSave.Request()
        req.request = json.dumps(
            {'table': 'items', 'rows': rows}, ensure_ascii=False)

        res = self.call(self.save_cli, req)
        if res is None:
            return

        print(f'  [{"OK" if res.success else "실패"}] {res.message}')
        if res.success:
            data = json.loads(res.response)
            for r in data['results']:
                mark = '신규' if r['action'] == 'insert' else '갱신'
                print(f'    {mark}  id={r["id"]}  {r["class_name"]}')

    def send_load(self, payload, title):
        req = DbLoad.Request()
        req.request = json.dumps(payload, ensure_ascii=False)

        res = self.call(self.load_cli, req)
        if res is None:
            return

        print(f'  [{"OK" if res.success else "실패"}] {res.message}')
        if not res.success:
            return

        data = json.loads(res.response)
        print(f'  --- {title} ({data["count"]}건) ---')
        if data['count'] == 0:
            print('    (없음)')
            return
        for it in data['items']:
            print(f'    [{it["id"]}] {it["class_name"]} (conf={it["confidence"]}) '
                  f'@ ({it["x"]}, {it["y"]}, {it["z"]})  / {it["last_seen"]}')

    def menu_generate(self):
        if self.run_index >= len(TEST_RUNS):
            self.run_index = 0
            print('  (마지막 회차까지 다 보내서 1회차로 되돌아갑니다)')

        rows = TEST_RUNS[self.run_index]
        label = self.run_index + 1
        print(f'  {label}회차 {len(rows)}건 전송: '
              + ', '.join(f'{r["class_name"]}@({r["x"]},{r["y"]},{r["z"]})' for r in rows))
        self.send_save(rows)
        self.run_index += 1

    def menu_manual(self):
        print('  (이미 있는 class_name을 또 넣으면 새로 안 생기고 값만 갱신됩니다'
              ' — upsert 확인용으로 같은 이름을 값만 바꿔서 여러 번 넣어보세요)')
        class_name = input('  class_name: ').strip()
        if not class_name:
            print('  !! class_name은 필수입니다')
            return
        try:
            x = float(input('  x: ').strip())
            y = float(input('  y: ').strip())
            z = float(input('  z: ').strip())
        except ValueError:
            print('  !! x/y/z는 숫자여야 합니다')
            return

        conf_raw = input('  confidence (그냥 엔터 = 1.0): ').strip()
        try:
            confidence = float(conf_raw) if conf_raw else 1.0
        except ValueError:
            print('  !! confidence는 숫자여야 합니다')
            return

        self.send_save([{'class_name': class_name, 'confidence': confidence,
                          'x': x, 'y': y, 'z': z}])

    def menu_clear(self):
        print('  !! items 를 전부 지웁니다. 되돌릴 수 없습니다.')
        answer = input('  정말 지우려면 DELETE 를 그대로 입력하세요: ').strip()
        if answer != 'DELETE':
            print('  취소했습니다')
            return

        res = self.call(self.clear_cli, Trigger.Request())
        if res is not None:
            print(f'  [{"OK" if res.success else "실패"}] {res.message}')
            if res.success:
                self.run_index = 0      # 테스트 회차도 1회차부터 다시
                print('  테스트 회차를 1회차로 되돌렸습니다')

    def menu_init(self):
        req = NodeInit.Request()
        req.request = json.dumps({'node': 'db_test_node'}, ensure_ascii=False)

        res = self.call(self.init_cli, req)
        if res is None:
            return

        print(f'  [{"OK" if res.success else "준비 안 됨"}] {res.message}')
        st = json.loads(res.response) if res.response else {}
        if st:
            print(f"    ready     : {st.get('ready')}")
            print(f"    db_path   : {st.get('db_path')}")
            print(f"    tables    : {st.get('tables')}")
            print(f"    items     : {st.get('items')}건")
            print(f"    tasks     : {st.get('tasks')}건")

    def loop(self):
        while rclpy.ok():
            try:
                choice = input(MENU).strip()
            except EOFError:
                break

            if choice == '1':
                self.menu_generate()
            elif choice == '2':
                self.send_load({}, '전체 목록')
            elif choice == '3':
                class_name = input('  검색할 class_name: ').strip()
                self.send_load({'class_name': class_name}, f"'{class_name}' 검색 결과")
            elif choice == '4':
                self.menu_manual()
            elif choice == '8':
                self.menu_clear()
            elif choice == '9':
                self.menu_init()
            elif choice == '0':
                print('종료합니다')
                break
            else:
                print('  !! 0~4, 8, 9 중에서 선택하세요')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DBTestNode()
        node.loop()
    except KeyboardInterrupt:
        pass
    except Exception as e:      # noqa: BLE001
        print(f'test_node 종료: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
