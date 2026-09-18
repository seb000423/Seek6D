"""ROS 2 client for the Seek6D DB node."""

from __future__ import annotations

import json
import time
from typing import Any

import rclpy
from interfaces.srv import DbLoad
from rclpy.node import Node

from .config import InterfaceConfig
from .models import DBLookupError, DBLookupResult, RobotTask
from .search_planner import db_position_mm, first_item, make_item_query


class DBClient:
    def __init__(self, node: Node, config: InterfaceConfig) -> None:
        self.node = node
        self.config = config
        self.client = node.create_client(DbLoad, config.db_load_service)

    def _call(self, query: dict[str, str]) -> dict[str, Any]:
        if not self.client.wait_for_service(
            timeout_sec=self.config.db_service_wait_timeout_sec
        ):
            raise DBLookupError(
                f"DB 서비스를 찾을 수 없습니다: {self.config.db_load_service}"
            )

        request = DbLoad.Request()
        request.request = json.dumps(query, ensure_ascii=False)
        future = self.client.call_async(request)
        deadline = time.monotonic() + self.config.db_response_timeout_sec

        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.1, max(0.0, remaining)),
            )

        if not future.done():
            future.cancel()
            raise DBLookupError(
                f"DB 응답 시간 초과: "
                f"{self.config.db_response_timeout_sec:.1f}초"
            )
        if future.exception() is not None:
            raise DBLookupError(f"DB 서비스 호출 실패: {future.exception()}")

        response = future.result()
        if response is None:
            raise DBLookupError("DB 서비스가 빈 응답을 반환했습니다")
        if not response.success:
            raise DBLookupError(response.message or "DB 조회 실패")

        try:
            data = json.loads(response.response) if response.response else {}
        except json.JSONDecodeError as error:
            raise DBLookupError(f"DB 응답 JSON 파싱 실패: {error}") from error
        if not isinstance(data, dict):
            raise DBLookupError("DB response는 JSON 객체여야 합니다")
        if not isinstance(data.get("items", []), list):
            raise DBLookupError("DB response.items는 배열이어야 합니다")
        return data

    def lookup(self, task: RobotTask) -> DBLookupResult:
        """Load the unique items row using the DB's ``class_name`` key."""
        try:
            query = make_item_query(task.name, task.class_label)
            data = self._call(query)
            item = first_item(data)
        except ValueError as error:
            raise DBLookupError(str(error)) from error
        return DBLookupResult(db_position_mm(item) is not None, query, item)
