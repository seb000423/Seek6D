"""DB 노드 — 4단계.

순서도 대응:
  부팅 -> DB 연결 -> (실패 시 재시도) -> 대기
       -> 상태 노드 초기화 요청 -> 응답 -> 대기
       -> 요청 분기: 데이터 저장 / DB 탐색

제공 서비스
  /db/init      interfaces/NodeInit  상태 노드의 초기화 확인 요청 (전 노드 공용 타입)
  /db/save      interfaces/DbSave    작업 완료 후 일괄 저장
  /db/load      interfaces/DbLoad    물건 유무 및 위치 조회
  /db/clear     std_srvs/Trigger     전체 삭제 (테스트용, allow_clear 로 차단 가능)
                                      — table 분기 없음. items 전체 삭제 그대로.

테이블은 딱 둘이다: items(물건 목록), tasks(작업기록). sightings(목격
이력)는 2026-08-06에 없앴다 — items 갱신마다 자동으로 쌓이던 이력 로그였는데
쓰는 데가 없어서 정리했다. 기존 DB 파일에 남아있는 sightings 테이블은
이 코드가 더는 안 건드리니 그냥 방치돼 있다(수동으로 DROP 해야 없어짐).

JSON 형식
  NodeInit 요청  {"node": "state_node"}
  NodeInit 응답  {"ready": true, "db_path": "...", "items": 4, "tasks": 1}

  DbSave/DbLoad 요청의 "table" 필드로 대상 테이블을 고른다. 없으면 "items"
  (기존 호출부와 완전히 같게 동작). "items"는 upsert 전용 로직을 그대로 쓰고,
  "tasks"는 ALLOWED_COLUMNS 화이트리스트로 걸러진 컬럼만 그대로
  INSERT/SELECT하는 공용 경로를 쓴다(db 저장 로그 명세 2026-08-06 참고).

  items 스키마는 비전 노드가 실제로 내는 형식(class_name/confidence/x/y/z)에
  맞췄다(2026-08-06 정정) — 사람이 부르는 "name"이나 문자열 "location"은
  없다. class_name 이 upsert 기준 키다(같은 class_name 이면 최신 감지값으로
  덮어씀).

  DbSave 요청(table 생략 또는 "items")
    {"table": "items",  # 생략 가능
     "rows": [{"class_name": "green_frog", "confidence": 0.87,
               "x": 412.5, "y": -135.8, "z": 125.3}, ...]}
  DbSave 응답  {"inserted": 1, "updated": 2, "results": [...]}
  -> rows 대신 예전 방식대로 {"items": [...]} 로 보내도 여전히 된다(호환용).

  DbSave 요청(table="tasks")
    {"table": "tasks",
     "rows": [{"command_text": "...", "target_name": "bear", ...}, ...]}
  DbSave 응답  {"table": "tasks", "inserted": 1, "results": [...]}
  -> 화이트리스트에 없는 키가 하나라도 있으면 전부 안 쓰고 에러(success=false).

  DbLoad 요청(table 생략 또는 "items")
    {}                            -> 전체
    {"class_name": "green_frog"}  -> 클래스명으로 조회
  DbLoad 응답  {"count": 4, "items": [{...}, ...]}

  DbLoad 요청(table="tasks")
    {"table": "tasks"}      -> 테이블 전체 조회(필터 없음)
  DbLoad 응답  {"table": "tasks", "count": 1, "rows": [{...}, ...]}
"""

import json
import os
import sqlite3
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from interfaces.srv import DbLoad, DbSave, NodeInit


# 스키마
CREATE_ITEMS = """
CREATE TABLE IF NOT EXISTS items (
    id          INTEGER PRIMARY KEY,
    class_name  TEXT NOT NULL UNIQUE COLLATE NOCASE,  -- 비전이 주는 클래스명(green_frog 등)
    confidence  REAL,                                 -- 인식 신뢰도
    x           REAL NOT NULL,                        -- 위치. 좌표계·단위는 비전이 주는 그대로
    y           REAL NOT NULL,
    z           REAL NOT NULL,
    last_seen   TEXT NOT NULL                         -- ISO8601. 서버가 저장 시각에 자동으로 찍음
)
"""

CREATE_TASKS = """
CREATE TABLE IF NOT EXISTS tasks (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    command_text  TEXT,
    voice_command TEXT,                    -- 사용자가 말한 원문 음성 텍스트
    target_name   TEXT,
    destination   TEXT,
    status        TEXT NOT NULL,
    fail_stage    TEXT,
    fail_reason   TEXT,
    found_at      TEXT,
    started_at    TEXT NOT NULL,
    ended_at      TEXT
)
"""

# class_name 이 UNIQUE라 이미 자동으로 인덱싱된다 — 별도 인덱스 불필요.
CREATE_INDEXES = []

# /db/save, /db/load 의 "table" 분기에서 쓰는 컬럼 화이트리스트. "items"는
# record_items_batch()/search_items()라는 전용 로직(upsert)이 따로 있어서
# 여기 값은 참고용으로만 두고, 실제로는 insert_rows()/select_rows()가
# "tasks"에만 쓰인다.
# 동적 INSERT의 컬럼명을 payload 키에서 직접 안 받고 이 화이트리스트를
# 거치는 이유: 오타·임의 컬럼이 조용히 무시되거나 SQL에 그대로 꽂히면 안 된다.
ALLOWED_COLUMNS = {
    "items": {"class_name", "confidence", "x", "y", "z", "last_seen"},
    "tasks": {
        "command_text", "voice_command", "target_name", "destination",
        "status", "fail_stage", "fail_reason",
        "found_at", "started_at", "ended_at",
    },
}


# DB 조작 (ROS 와 무관한 순수 함수)
def now_iso() -> str:
    return datetime.now().isoformat(timespec='seconds')


def _needs_recreate(conn, table, expected_columns) -> bool:
    """이미 있는 테이블의 컬럼이 지금 기대하는 것과 다르면 True.

    `CREATE TABLE IF NOT EXISTS`는 이미 있는 테이블은 안 건드린다 — 개발
    중 스키마가 바뀌었는데 예전 DB 파일을 그대로 쓰면, 코드는 새 컬럼을
    기대하는데 파일엔 옛날 컬럼이 남아있어서 조회할 때 "그런 컬럼 없음"
    으로 죽는다(2026-08-06 실제로 겪음 — items가 name/class_label/location
    에서 class_name/confidence/x/y/z로 바뀐 뒤).
    테이블이 아예 없으면 False — 그건 그냥 새로 만들면 되니 여기서 상관할
    일이 아니다.
    """
    existing = {row[1] for row in conn.execute(f'PRAGMA table_info({table})')}
    return bool(existing) and existing != expected_columns


def normalize_record(rec):
    """(class_name, confidence, x, y, z) 튜플로 통일한다.

    비전 노드가 실제로 내는 형식 그대로다(2026-08-06 정정) — 사람이 부르는
    "name"이나 문자열 "location"은 없다.
    """
    if isinstance(rec, dict):
        class_name = rec.get('class_name')
        confidence = rec.get('confidence')
        x, y, z = rec.get('x'), rec.get('y'), rec.get('z')
    else:
        class_name, confidence, x, y, z = rec

    if not class_name or not str(class_name).strip():
        raise ValueError('class_name 이 비어 있습니다')
    if x is None or y is None or z is None:
        raise ValueError(f"'{class_name}' 의 위치(x/y/z)가 비어 있습니다")

    return (str(class_name).strip(),
            float(confidence) if confidence is not None else None,
            float(x), float(y), float(z))


def _write_one(cur, class_name, confidence, x, y, z, last_seen):
    """물건 1건을 upsert한다. (트랜잭션은 호출한 쪽이 연다)"""
    before = cur.execute(
        'SELECT id FROM items WHERE class_name = ?', (class_name,)).fetchone()

    cur.execute(
        """
        INSERT INTO items (class_name, confidence, x, y, z, last_seen)
        VALUES (?, ?, ?, ?, ?, ?)
        ON CONFLICT(class_name) DO UPDATE SET
            confidence = excluded.confidence,
            x          = excluded.x,
            y          = excluded.y,
            z          = excluded.z,
            last_seen  = excluded.last_seen
        """,
        (class_name, confidence, x, y, z, last_seen),
    )

    item_id = cur.execute(
        'SELECT id FROM items WHERE class_name = ?', (class_name,)).fetchone()[0]

    return item_id, ('update' if before else 'insert')


def record_items_batch(conn, records):
    """작업 1회분을 한 트랜잭션으로 기록한다.

    본 것만 갱신(UPSERT)하고, 이번에 못 본 물건은 건드리지 않는다.
    하나라도 실패하면 전체가 롤백된다.
    """
    seen_at = now_iso()          # 같은 작업분은 같은 시각으로 묶는다
    normalized = [normalize_record(r) for r in records]
    results = []

    with conn:                   # 여기서 트랜잭션 한 번만 열린다
        cur = conn.cursor()
        for class_name, confidence, x, y, z in normalized:
            item_id, action = _write_one(
                cur, class_name, confidence, x, y, z, seen_at)
            results.append({'class_name': class_name, 'id': item_id, 'action': action})

    return results


def search_items(conn, class_name=None):
    """조건에 맞는 items 를 dict 리스트로 돌려준다. 조건이 없으면 전체."""
    conn.row_factory = sqlite3.Row

    if class_name:
        sql = 'SELECT * FROM items WHERE class_name = ? ORDER BY class_name'
        args = (class_name.strip(),)
    else:
        sql = 'SELECT * FROM items ORDER BY id'
        args = ()

    return [dict(row) for row in conn.execute(sql, args)]


def insert_rows(conn, table, rows):
    """`table`(items 제외)에 화이트리스트 컬럼만 그대로 INSERT한다.

    `table`은 호출한 쪽이 `ALLOWED_COLUMNS`에 있는 값인지 미리 확인해야
    한다 — 여기서는 그 값을 SQL에 직접 꽂아 쓰므로(파라미터 바인딩이 안
    되는 식별자라서) 검증되지 않은 값이 넘어오면 안 된다.

    행 하나라도 화이트리스트에 없는 컬럼을 담고 있으면 아무것도 안 쓰고
    바로 에러를 낸다("조용히 무시" 금지) — 그래서 전체 행을 먼저 다 검사한
    다음에 트랜잭션을 연다.
    """
    allowed = ALLOWED_COLUMNS[table]

    for row in rows:
        unknown = set(row.keys()) - allowed
        if unknown:
            raise ValueError(
                f"'{table}'에 허용되지 않은 컬럼: {sorted(unknown)}"
            )
        if not row:
            raise ValueError(f"'{table}' 행에 컬럼이 하나도 없습니다")

    results = []
    with conn:
        cur = conn.cursor()
        for row in rows:
            cols = list(row.keys())
            placeholders = ", ".join("?" for _ in cols)
            col_list = ", ".join(cols)
            cur.execute(
                f"INSERT INTO {table} ({col_list}) VALUES ({placeholders})",
                [row[c] for c in cols],
            )
            results.append({"id": cur.lastrowid, **row})

    return results


def select_rows(conn, table, limit=None):
    """`table`(items 제외) 을 dict 리스트로 돌려준다. 필터 없음.

    `limit`을 주면 최근 것(id 내림차순)부터 그만큼만 가져온다 — `tasks`처럼
    계속 쌓이기만 하는 테이블을 back_ui가 1초마다 폴링하는데(2026-08-06),
    매번 테이블 전체를 긁으면 시간이 지날수록 느려진다. front_ui는 최근
    5건만 쓰므로(README) 여유 있게 20건 정도로 잡아서 부르면 된다.

    `table` 검증은 `insert_rows`와 같은 이유로 호출한 쪽 책임이다.
    """
    conn.row_factory = sqlite3.Row
    sql = f"SELECT * FROM {table} ORDER BY id DESC"
    args = ()
    if limit is not None:
        sql += " LIMIT ?"
        args = (limit,)
    return [dict(row) for row in conn.execute(sql, args)]


def clear_all(conn):
    """items 를 통째로 비운다. 되돌릴 수 없다.

    tasks는 안 건드린다 — /db/clear 동작 자체는 이번에 바꾸지 말라는
    요청이 있었다(2026-08-06). tasks도 같이 지우게 하려면 별도로 정해야 함.

    반환: 지워진 items 수
    """
    with conn:
        cur = conn.cursor()
        n_items = cur.execute('SELECT COUNT(*) FROM items').fetchone()[0]
        cur.execute('DELETE FROM items')
    return n_items


# 서비스 요청 처리 (JSON in -> JSON out)
# ROS 객체를 안 써서 노드 없이도 단독 테스트가 된다
def db_status(conn, db_path):
    """DB 준비 상태를 dict 로 돌려준다."""
    tables = [r[0] for r in conn.execute(
        "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name")]
    return {
        'ready': 'items' in tables and 'tasks' in tables,
        'db_path': db_path,
        'tables': tables,
        'items': conn.execute('SELECT COUNT(*) FROM items').fetchone()[0],
        'tasks': conn.execute('SELECT COUNT(*) FROM tasks').fetchone()[0],
    }


def handle_init(conn, request_json, db_path):
    """초기화 확인 요청을 처리한다. 반환: (success, response_json, message)"""
    try:
        payload = json.loads(request_json) if request_json.strip() else {}
    except json.JSONDecodeError as e:
        return False, '{}', f'JSON 파싱 실패: {e}'

    who = payload.get('node', 'unknown')

    try:
        status = db_status(conn, db_path)
    except sqlite3.Error as e:
        return False, '{}', f'상태 확인 실패: {e}'

    msg = (f"DB 준비됨 (items {status['items']}건, "
           f"tasks {status['tasks']}건) / 요청자={who}")
    return (status['ready'],
            json.dumps(status, ensure_ascii=False),
            msg if status['ready'] else '테이블이 준비되지 않았습니다')


def handle_save(conn, request_json):
    """반환: (success, response_json, message)

    "table" 필드로 분기한다. 생략하면 "items" — 기존 호출부(items 키)가
    그대로 동작해야 하므로 upsert 로직을 그대로 쓴다("source"는 여전히
    받지만 저장할 곳이 없어져서 무시한다). "tasks"는 화이트리스트 컬럼만
    그대로 INSERT.
    """
    try:
        payload = json.loads(request_json) if request_json.strip() else {}
    except json.JSONDecodeError as e:
        return False, '{}', f'JSON 파싱 실패: {e}'

    table = payload.get('table', 'items')
    if table not in ALLOWED_COLUMNS:
        return False, '{}', f"허용되지 않은 테이블입니다: '{table}'"

    if table == 'items':
        # 비전 노드는 "rows" 키로 보낸다(tasks와 통일). 예전 호출부가
        # "items" 키로 보내던 것도 여전히 받아준다.
        records = payload.get('rows') or payload.get('items') or []

        if not records:
            return False, '{}', '저장할 항목이 없습니다'

        try:
            results = record_items_batch(conn, records)
        except Exception as e:      # noqa: BLE001
            return False, '{}', f'저장 실패 — 전체 롤백: {e}'

        inserted = sum(1 for r in results if r['action'] == 'insert')
        updated = len(results) - inserted
        response = {'inserted': inserted, 'updated': updated, 'results': results}

        return (True, json.dumps(response, ensure_ascii=False),
                f'저장 완료: 신규 {inserted} / 갱신 {updated}')

    # tasks — 화이트리스트 컬럼만 그대로 INSERT
    rows = payload.get('rows', [])
    if not rows:
        return False, '{}', '저장할 행이 없습니다'

    try:
        results = insert_rows(conn, table, rows)
    except (ValueError, sqlite3.Error) as e:
        return False, '{}', f"'{table}' 저장 실패 — 전체 롤백: {e}"

    response = {'table': table, 'inserted': len(results), 'results': results}
    return (True, json.dumps(response, ensure_ascii=False),
            f"'{table}' 저장 완료: {len(results)}건")


def handle_search(conn, request_json):
    """반환: (success, response_json, message)

    "table" 필드로 분기한다. 생략하면 "items" — 기존 name/class_label 필터
    조회 그대로. "tasks"는 필터 없이 테이블 전체를 돌려준다.
    """
    try:
        payload = json.loads(request_json) if request_json.strip() else {}
    except json.JSONDecodeError as e:
        return False, '{}', f'JSON 파싱 실패: {e}'

    table = payload.get('table', 'items')
    if table not in ALLOWED_COLUMNS:
        return False, '{}', f"허용되지 않은 테이블입니다: '{table}'"

    if table == 'items':
        class_name = payload.get('class_name')

        try:
            rows = search_items(conn, class_name=class_name)
        except sqlite3.Error as e:
            return False, '{}', f'조회 실패: {e}'

        if class_name:
            msg = f"'{class_name}' {'찾음' if rows else '없음'}"
        else:
            msg = f'전체 {len(rows)}건'

        # 조회 자체는 성공. 물건이 없는 것은 실패가 아니라 count=0 으로 알린다
        response = {'count': len(rows), 'items': rows}
        return True, json.dumps(response, ensure_ascii=False), msg

    # tasks — 필터 없이 최근 것 위주로 조회. 계속 쌓이기만 하는 테이블이라
    # 전체를 매번 긁지 않는다(위 select_rows 설명 참고). front_ui 홈 화면은
    # 앞 5건만 잘라 쓰지만(recent_tasks[:5]) 로그 화면 "작업 기록" 탭은
    # 안 자르고 온 걸 다 보여주므로(훑어보는 화면), 5보다 넉넉하게
    # 잡는다 — 그래도 SQLite 로컬 쿼리라 100건이어도 비용은 무시할 만하다.
    try:
        rows = select_rows(conn, table, limit=100 if table == 'tasks' else None)
    except sqlite3.Error as e:
        return False, '{}', f'조회 실패: {e}'

    response = {'table': table, 'count': len(rows), 'rows': rows}
    return True, json.dumps(response, ensure_ascii=False), f"'{table}' {len(rows)}건"


class DBNode(Node):

    def __init__(self):
        super().__init__('db_node')

        self.declare_parameter('db_path', '~/.ros/robot_db/robot.db')
        self.declare_parameter('require_init', False)
        self.declare_parameter('allow_clear', True)   # 실서비스에서는 false 로

        raw_path = self.get_parameter('db_path').value
        self.db_path = os.path.abspath(os.path.expanduser(raw_path))
        self.require_init = self.get_parameter('require_init').value
        self.allow_clear = self.get_parameter('allow_clear').value

        self.conn = None
        self.initialized = False
        self.setup_database()

        self.create_service(NodeInit, 'db/init', self.on_init)
        self.create_service(DbSave, 'db/save', self.on_save)
        self.create_service(DbLoad, 'db/load', self.on_load)
        self.create_service(Trigger, 'db/clear', self.on_clear)

        self.dump_tables()
        self.get_logger().info('DB 준비 완료 — 요청 대기 중')

    def setup_database(self):
        """DB 파일을 열고 테이블을 만든다."""
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        self.get_logger().info(f'DB 파일 경로: {self.db_path}')

        try:
            self.conn = sqlite3.connect(self.db_path)
            self.conn.execute('PRAGMA foreign_keys = ON')

            cur = self.conn.cursor()

            # 스키마가 바뀌었는데 예전 파일 그대로면(_needs_recreate 참고)
            # 지우고 새로 만든다 — 개발 단계라 데이터 보존보다 안 죽는 게
            # 우선이다. 운영 데이터가 쌓이기 시작하면 이 방식(드롭)은 진짜
            # 마이그레이션으로 바꿔야 한다.
            for table, expected in (
                ('items', ALLOWED_COLUMNS['items'] | {'id'}),
                ('tasks', ALLOWED_COLUMNS['tasks'] | {'id'}),
            ):
                if _needs_recreate(self.conn, table, expected):
                    self.get_logger().warn(
                        f"'{table}' 테이블 스키마가 바뀌어서 다시 만듭니다"
                        f' (기존 데이터는 사라집니다)')
                    cur.execute(f'DROP TABLE {table}')

            cur.execute(CREATE_ITEMS)
            cur.execute(CREATE_TASKS)
            for sql in CREATE_INDEXES:
                cur.execute(sql)
            self.conn.commit()

        except sqlite3.Error as e:
            self.get_logger().fatal(f'DB 초기화 실패: {e}')
            raise

        tables = [row[0] for row in self.conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name")]
        self.get_logger().info(f'테이블 확인: {tables}')

    def on_init(self, request, response):
        """상태 노드의 초기화 확인 요청에 응답한다."""
        success, data, message = handle_init(
            self.conn, request.request, self.db_path)

        response.success = success
        response.response = data
        response.message = message

        # 준비가 확인된 경우에만 초기화 완료로 표시한다
        if success:
            self.initialized = True

        log = self.get_logger().info if success else self.get_logger().error
        log(f'[init] {message}')
        return response

    def _check_init(self, response):
        """require_init 이 켜져 있으면 초기화 전 요청을 막는다."""
        if self.require_init and not self.initialized:
            response.success = False
            response.response = '{}'
            response.message = '아직 초기화되지 않았습니다 (/db/init 먼저 호출)'
            self.get_logger().warn(response.message)
            return False
        return True

    def on_save(self, request, response):
        """데이터 저장 — 작업 1회분을 통째로 반영한다."""
        if not self._check_init(response):
            return response

        success, data, message = handle_save(self.conn, request.request)
        response.success = success
        response.response = data
        response.message = message

        log = self.get_logger().info if success else self.get_logger().error
        log(f'[save] {message}')
        if success:
            self.dump_tables()
        return response

    def on_load(self, request, response):
        """DB 탐색 — 물건 유무 및 위치를 돌려준다."""
        if not self._check_init(response):
            return response

        success, data, message = handle_search(self.conn, request.request)
        response.success = success
        response.response = data
        response.message = message

        log = self.get_logger().info if success else self.get_logger().error
        log(f'[load] {message}')
        return response

    def on_clear(self, request, response):
        """전체 삭제 — 테스트용. 되돌릴 수 없다."""
        if not self.allow_clear:
            response.success = False
            response.message = '전체 삭제가 비활성화되어 있습니다 (allow_clear=false)'
            self.get_logger().warn(f'[clear] 거절 — {response.message}')
            return response

        try:
            n_items = clear_all(self.conn)
        except sqlite3.Error as e:
            response.success = False
            response.message = f'삭제 실패 — 롤백: {e}'
            self.get_logger().error(f'[clear] {response.message}')
            return response

        response.success = True
        response.message = f'전체 삭제 완료 (items {n_items}건)'
        self.get_logger().warn(f'[clear] {response.message}')
        self.dump_tables()
        return response

    def dump_tables(self):
        """현재 DB 내용을 로그로 출력한다. (확인용)"""
        self.conn.row_factory = sqlite3.Row

        items = self.conn.execute('SELECT * FROM items ORDER BY id').fetchall()
        n_tasks = self.conn.execute(
            'SELECT COUNT(*) FROM tasks').fetchone()[0]

        self.get_logger().info(
            f'--- items {len(items)}건 / tasks {n_tasks}건 ---')
        for r in items:
            self.get_logger().info(
                f"  [{r['id']}] {r['class_name']} (conf={r['confidence']}) "
                f"@ ({r['x']}, {r['y']}, {r['z']}) / {r['last_seen']}")

    def destroy_node(self):
        if self.conn is not None:
            self.conn.close()
            self.get_logger().info('DB 연결 종료')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DBNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:      # noqa: BLE001
        print(f'db_node 종료: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
