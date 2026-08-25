#!/usr/bin/env python3
"""Scenario 0의 작업큐, cuOpt 호출, AMR Pull 요청을 관리하는 FMS ROS 2 Node."""

from __future__ import annotations

import time
from collections import deque
from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from interfaces.msg import NodeMapChanged
from interfaces.srv import GetNodeMap, RequestTask

from fms.defined import (
    AMRState,
    LOCATION_BY_ID,
    PARTS_SUPERMARKET,
    OptimizationRequest,
    OptimizationResult,
    Task,
)

class FleetManagementSystem(Node):

    # =====================================================
    # 기본 설정
    # =====================================================

    QUEUE_CAPACITY = 10

    # 실제 Assembly Node에서 사용 중인 Topic
    TASK_REQUEST_TOPIC = "/assembly/request"

    # AMR Pull 요청 Service
    TASK_REQUEST_SERVICE = "/fms/request_task"

    # AMR 상태 이벤트
    AMR_STATUS_TOPIC = "/amr/status"

    # Isaac Sim NodeMap Service
    NODE_MAP_SERVICE = "/get_node_map"
    NODE_MAP_CHANGED_TOPIC = "/node_map_changed"
    NODE_MAP_RETRY_INTERVAL_SEC = 2.0

    # 완료로 간주할 상태
    FINISHED_STATUSES = {
        "DELIVERY_COMPLETE",
        "MISSION_COMPLETE",
    }

    def __init__(self) -> None:

        super().__init__("FleetManagementSystem")

        # =================================================
        # FMS Task Queue
        #
        # 아직 AMR에 할당되지 않은 waiting task
        # =================================================

        self.task_queue: deque[Task] = deque(maxlen=self.QUEUE_CAPACITY)

        # =================================================
        # Active Task Registry
        #
        # 이미 어떤 AMR에 할당된 작업을 저장
        #
        # key   : task_id
        # value : Task
        # =================================================

        self.active_tasks: dict[str, Task] = {}

        # =================================================
        # 마지막 최적화 결과
        # =================================================

        self.latest_plan: OptimizationResult | None = None

        # =================================================
        # cuOpt 실행 중 여부
        # =================================================

        self.is_optimizing = False

        # =================================================
        # AMR 상태 저장
        # =================================================

        self.amr_states: dict[str, dict[str, str]] = {}

        # =================================================
        # Isaac Sim에서 받은 NodeMap Runtime 데이터
        # =================================================

        self.node_map_nodes: dict[int, dict[str, object]] = {}
        self.node_map_edges: list[dict[str, object]] = []
        self.node_map_revision = 0
        self.expected_node_map_revision: int | None = None
        self.node_map_request_pending = False
        self.node_map_wait_logged = False

        # =================================================
        # Assembly -> FMS
        # Task 요청
        # =================================================

        self.task_subscription = self.create_subscription(
            String,
            self.TASK_REQUEST_TOPIC,
            self.task_request_callback,
            self.QUEUE_CAPACITY,
        )

        # =================================================
        # AMR -> FMS
        # AMR 상태 변화 Event
        # =================================================

        self.amr_status_subscription = self.create_subscription(
            String,
            self.AMR_STATUS_TOPIC,
            self.amr_status_callback,
            10,
        )

        # =================================================
        # AMR -> FMS
        # Pull 방식 Task 요청
        # =================================================

        self.task_request_service = self.create_service(
            RequestTask,
            self.TASK_REQUEST_SERVICE,
            self.request_task_callback,
        )

        # =================================================
        # FMS -> Isaac Sim NodeMap Service Client
        # =================================================

        self.node_map_client = self.create_client(
            GetNodeMap,
            self.NODE_MAP_SERVICE,
        )

        self.node_map_changed_subscription = self.create_subscription(
            NodeMapChanged,
            self.NODE_MAP_CHANGED_TOPIC,
            self.node_map_changed_callback,
            10,
        )
        
        self.node_map_request_timer = self.create_timer(
            self.NODE_MAP_RETRY_INTERVAL_SEC,
            self.request_node_map,
        )

        # =================================================
        # 시작 로그
        # =================================================

        self.get_logger().info("=================================")
        self.get_logger().info("Scenario 0 FMS started")
        self.get_logger().info(
            f"Assembly Topic : {self.TASK_REQUEST_TOPIC}"
        )
        self.get_logger().info(
            f"Task Service   : {self.TASK_REQUEST_SERVICE}"
        )
        self.get_logger().info(
            f"AMR Status     : {self.AMR_STATUS_TOPIC}"
        )
        self.get_logger().info(
            f"NodeMap Service: {self.NODE_MAP_SERVICE}"
        )
        self.get_logger().info(
            f"NodeMap Event  : {self.NODE_MAP_CHANGED_TOPIC}"
        )
        self.get_logger().info("Task Queue     : EMPTY")
        self.get_logger().info("=================================")




# region FMS -> Isaac Sim NodeMap 요청

    # =====================================================
    # FMS -> Isaac Sim NodeMap 요청
    # =====================================================

    def node_map_changed_callback(self, message: NodeMapChanged) -> None:
        self.expected_node_map_revision = int(message.revision)
        self.get_logger().info(
            f"[NODE MAP] Change detected: revision={message.revision}, "
            f"Nodes={message.node_count}, Edges={message.edge_count}, "
            f"Stage={message.stage_identifier}"
        )
        self.request_node_map(force=True)

    def request_node_map(self, force: bool = False) -> None:
        if self.node_map_request_pending:
            return
        if (
            self.node_map_nodes
            and self.expected_node_map_revision is None
            and not force
        ):
            return

        if not self.node_map_client.service_is_ready():
            if not self.node_map_wait_logged:
                self.get_logger().info(
                    f"[NODE MAP] Waiting for {self.NODE_MAP_SERVICE}"
                )
                self.node_map_wait_logged = True
            return

        self.node_map_wait_logged = False
        self.node_map_request_pending = True

        self.get_logger().info("[NODE MAP] Requesting NodeMap from Isaac Sim")
        future = self.node_map_client.call_async(GetNodeMap.Request())
        future.add_done_callback(self.node_map_response_callback)

    def node_map_response_callback(self, future) -> None:
        self.node_map_request_pending = False

        try:
            response = future.result()
            if response is None:
                raise RuntimeError("GetNodeMap returned no response.")
            if not response.success:
                raise RuntimeError(response.message or "Isaac Sim rejected the request.")

            nodes, edges = self._parse_node_map_response(response)
            response_revision = int(response.revision)
            if (
                self.expected_node_map_revision is not None
                and response_revision != self.expected_node_map_revision
            ):
                raise RuntimeError(
                    "NodeMap revision mismatch: "
                    f"expected={self.expected_node_map_revision}, "
                    f"received={response_revision}"
                )
        except Exception as error:
            self.get_logger().error(f"[NODE MAP] Request failed: {error}")
            self.node_map_request_timer.reset()
            return

        self.node_map_nodes = nodes
        self.node_map_edges = edges
        self.node_map_revision = response_revision
        self.expected_node_map_revision = None
        self.node_map_request_timer.cancel()

        self.get_logger().info(
            f"[NODE MAP] Loaded revision={self.node_map_revision}, "
            f"Nodes={len(nodes)}, Edges={len(edges)}"
        )


    # 노드맵 작명규칙에 따른 속성 데이터 주입.
    @staticmethod
    def _parse_node_map_response(response) -> tuple[
        dict[int, dict[str, object]],
        list[dict[str, object]],
    ]:
        node_fields = (
            response.node_ids,
            response.node_names,
            response.node_types,
            response.node_x,
            response.node_y,
            response.node_z,
        )
        node_count = len(response.node_ids)
        if node_count == 0:
            raise ValueError("NodeMap response contains no nodes.")
        if any(len(field) != node_count for field in node_fields):
            raise ValueError("NodeMap node arrays have different lengths.")

        edge_fields = (
            response.edge_from,
            response.edge_to,
            response.edge_weights,
            response.edge_bidirectional,
        )
        edge_count = len(response.edge_from)
        if any(len(field) != edge_count for field in edge_fields):
            raise ValueError("NodeMap edge arrays have different lengths.")

        nodes: dict[int, dict[str, object]] = {}
        for index in range(node_count):
            node_id = int(response.node_ids[index])
            if node_id in nodes:
                raise ValueError(f"Duplicate Node ID: {node_id}")

            nodes[node_id] = {
                "name": response.node_names[index],
                "type": response.node_types[index],
                "position": (
                    float(response.node_x[index]),
                    float(response.node_y[index]),
                    float(response.node_z[index]),
                ),
                "available": True,
            }

        edges: list[dict[str, object]] = []
        for index in range(edge_count):
            start = int(response.edge_from[index])
            end = int(response.edge_to[index])
            if start not in nodes or end not in nodes:
                raise ValueError(f"Edge references unknown Node: {start} -> {end}")

            edges.append(
                {
                    "start": start,
                    "end": end,
                    "weight": float(response.edge_weights[index]),
                    "bidirectional": bool(response.edge_bidirectional[index]),
                }
            )

        return nodes, edges

    # =====================================================
    # 문자열 key=value 메시지 Parser
    # =====================================================

    @staticmethod
    def parse_key_value_message(data: str) -> dict[str, str]:

        result: dict[str, str] = {}

        for item in data.split(","):

            item = item.strip()

            if not item:
                continue

            if "=" not in item:
                continue

            key, value = item.split("=", 1)
            result[key.strip()] = value.strip()

        return result



# endregion



    # =====================================================
    # Queue에 동일 task_id가 있는지 확인
    # =====================================================

    def is_task_in_queue(self, task_id: str) -> bool:

        return any(
            queued.task_id == task_id
            for queued in self.task_queue
        )

    # =====================================================
    # Active Task인지 확인
    # =====================================================

    def is_task_active(self, task_id: str) -> bool:

        return task_id in self.active_tasks

    # =====================================================
    # Queue 상태 로그
    # =====================================================

    def log_queue_summary(self) -> None:

        self.get_logger().info(
            f"[QUEUE] waiting={len(self.task_queue)} "
            f"active={len(self.active_tasks)}"
        )

    # =====================================================
    # Assembly -> FMS
    #
    # Task 요청 수신
    #
    # 현재 Assembly 메시지 예:
    #
    # cell_id=A,
    # task_id=1,
    # kit_id=KIT_STAR,
    # shape=STAR,
    # processing_time=3.0
    #
    # 현재 Assembly에는 아직
    # urgency / requested_at / deadline이 없으므로
    # Scenario 0 기본값을 임시 적용
    # =====================================================

    def task_request_callback(self, message: String) -> None:

        try:

            fields = self.parse_key_value_message(
                message.data
            )

            # =============================================
            # 필수 값 확인
            # =============================================

            cell_id = fields.get("cell_id")
            task_id = fields.get("task_id")
            kit_id = fields.get("kit_id")
            processing_time = fields.get("processing_time")

            if (
                cell_id is None
                or task_id is None
                or kit_id is None
                or processing_time is None
            ):
                raise ValueError(
                    "Assembly request requires "
                    "cell_id, task_id, kit_id, processing_time."
                )

            # =============================================
            # Cell 이름 통일
            # A -> cell_a
            # =============================================

            delivery_cell = f"cell_{cell_id.lower()}"

            # =============================================
            # Scenario 0 임시 metadata
            # =============================================

            urgency = int(fields.get("urgency", 1))

            requested_at = int(
                fields.get("requested_at", int(time.monotonic()))
            )

            deadline = int(
                fields.get("deadline", requested_at + 600)
            )

            # =============================================
            # Task 생성
            # =============================================

            task = Task(
                task_id=str(task_id),
                kit_id=str(kit_id),
                delivery_cell=delivery_cell,
                urgency=urgency,
                requested_at=requested_at,
                deadline=deadline,
                processing_time=float(processing_time),
            )

            self.add_task(task)

        except (ValueError, OverflowError) as error:

            self.get_logger().warning(
                f"Rejected Assembly Task: {error}"
            )
            return

        self.get_logger().info(
            f"[TASK QUEUED] "
            f"{task.task_id} {task.kit_id} -> {task.delivery_cell} "
            f"(processing={task.processing_time:.1f}s) "
            f"queue={len(self.task_queue)}/{self.QUEUE_CAPACITY}"
        )

        self.log_queue_summary()

    # =====================================================
    # Task Queue 추가
    # =====================================================

    def add_task(self, task: Task) -> None:

        # =================================================
        # Queue Full
        # =================================================

        if len(self.task_queue) >= self.QUEUE_CAPACITY:
            raise OverflowError(
                f"Task queue is full (capacity={self.QUEUE_CAPACITY})."
            )

        # =================================================
        # Cell 확인
        # =================================================

        if task.delivery_cell not in {
            "cell_a",
            "cell_b",
            "cell_c",
        }:
            raise ValueError(
                f"Unknown Assembly Cell: {task.delivery_cell}"
            )

        # =================================================
        # 중복 Task 방지
        #
        # 1) waiting queue 안에 이미 있으면 무시
        # 2) active_tasks 안에 이미 있으면 무시
        #
        # Assembly가 같은 task_id를
        # 재전송할 수 있으므로 반드시 필요
        # =================================================

        if self.is_task_in_queue(task.task_id):
            raise ValueError(
                f"Duplicate task_id already in waiting queue: "
                f"{task.task_id}"
            )

        if self.is_task_active(task.task_id):
            raise ValueError(
                f"Duplicate task_id already active: "
                f"{task.task_id}"
            )

        self.task_queue.append(task)

    # =====================================================
    # AMR 상태 Event 수신
    #
    # 예:
    # amr_id=AMR_01,state=BUSY,status=MOVING_TO_DELIVERY,
    # task_id=17,load_state=LOADED
    # =====================================================

    def amr_status_callback(self, message: String) -> None:

        fields = self.parse_key_value_message(message.data)

        amr_id = fields.get("amr_id")

        if not amr_id:
            self.get_logger().warning(
                f"Invalid AMR status: {message.data}"
            )
            return

        state = fields.get("state", "")
        status = fields.get("status", "")
        task_id = fields.get("task_id", "")
        load_state = fields.get("load_state", "")

        self.amr_states[amr_id] = {
            "state": state,
            "status": status,
            "current_task_id": task_id,
            "load_state": load_state,
        }

        self.get_logger().info(
            f"[AMR EVENT] "
            f"{amr_id} "
            f"state={state} "
            f"status={status} "
            f"task={task_id or '-'} "
            f"load={load_state}"
        )

        # =================================================
        # 작업 완료 이벤트면 active registry에서 제거
        # =================================================

        if (
            task_id
            and task_id != "-"
            and status in self.FINISHED_STATUSES
        ):
            if task_id in self.active_tasks:
                finished_task = self.active_tasks.pop(task_id)

                self.get_logger().info(
                    f"[TASK FINISHED] "
                    f"{finished_task.task_id} removed from active registry"
                )

                self.log_queue_summary()

    # =====================================================
    # AMR -> FMS
    #
    # 다음 Task 요청
    #
    # Pull 방식 핵심 Callback
    # =====================================================

    def request_task_callback(
        self,
        request: RequestTask.Request,
        response: RequestTask.Response,
    ) -> RequestTask.Response:

        self.get_logger().info(
            f"[TASK REQUEST] "
            f"{request.amr_id} "
            f"state={request.state} "
            f"position=({request.x:.2f}, {request.y:.2f}) "
            f"load={request.load_state}"
        )

        # =================================================
        # Request 자체를 최신 상태로 반영
        # =================================================

        self.amr_states[request.amr_id] = {
            "state": request.state,
            "status": "REQUESTING_TASK",
            "current_task_id": request.current_task_id,
            "load_state": request.load_state,
        }

        # =================================================
        # 대기 Task 없음
        # =================================================

        if not self.task_queue:
            response.has_task = False
            response.message = "No waiting task."

            self.get_logger().info(
                f"[NO TASK] {request.amr_id}"
            )
            return response

        # =================================================
        # 이미 cuOpt 실행 중
        # =================================================

        if self.is_optimizing:
            response.has_task = False
            response.message = (
                "Optimization is already in progress."
            )
            return response

        # =================================================
        # 현재 AMR 상태 생성
        # =================================================

        amr_state = AMRState(
            amr_id=request.amr_id,
            state=request.state,
            x=float(request.x),
            y=float(request.y),
            yaw=0.0,  # 현재 RequestTask.srv에는 yaw 없음
            load_state=request.load_state,
            current_task_id=request.current_task_id,
        )

        # =================================================
        # 현재 waiting queue 전체를 cuOpt에 전달
        # =================================================

        tasks = list(self.task_queue)
        self.is_optimizing = True

        self.get_logger().info(
            f"[CUOPT] Sending {len(tasks)} tasks for {request.amr_id}"
        )

        try:

            # cuOpt/cudf는 실제 최적화 요청이 들어올 때만 필요하다.
            # 이를 지연 import해 NodeMap 통신 확인은 cuOpt 환경 없이도 가능하다.
            from fms.cuopt_solver import CuOptSolver

            optimization_request = OptimizationRequest(
                tasks=tuple(tasks),
                amr_state=amr_state,
            )

            self.latest_plan = CuOptSolver(
                optimization_request
            ).solve()

            # =============================================
            # cuOpt 결과 확인
            # =============================================

            if (
                not self.latest_plan.success
                or not self.latest_plan.ordered_tasks
            ):
                response.has_task = False
                response.message = self.latest_plan.message
                return response

            self.get_logger().info(
                "=== Optimized Task Order ==="
            )

            for ordered_task in self.latest_plan.ordered_tasks:
                self.get_logger().info(
                    f"{ordered_task.sequence:02d}. "
                    f"{ordered_task.task_id} -> "
                    f"{ordered_task.delivery_cell}"
                )

            # =============================================
            # 첫 번째 Task 선택
            # =============================================

            selected_order = self.latest_plan.ordered_tasks[0]

            selected_task = next(
                task
                for task in tasks
                if task.task_id == selected_order.task_id
            )

            # =============================================
            # logical location -> physical coordinate
            # =============================================

            pickup_location = PARTS_SUPERMARKET
            delivery_location = LOCATION_BY_ID[
                selected_task.delivery_cell
            ]

            # =============================================
            # AMR Response
            # =============================================

            response.has_task = True
            response.task_id = selected_task.task_id
            response.kit_id = selected_task.kit_id
            response.processing_time = float(
                selected_task.processing_time
            )

            # Pickup
            response.pickup_id = pickup_location.location_id
            response.pickup_x = float(pickup_location.x)
            response.pickup_y = float(pickup_location.y)

            # Delivery
            response.delivery_id = delivery_location.location_id
            response.delivery_x = float(delivery_location.x)
            response.delivery_y = float(delivery_location.y)

            response.message = (
                f"Assigned {selected_task.task_id} "
                f"to {request.amr_id}"
            )

            # =============================================
            # Task 상태 변경
            # =============================================

            selected_task.status = "ASSIGNED"

            # =============================================
            # waiting queue -> active_tasks 이동
            # =============================================

            self.task_queue.remove(selected_task)
            self.active_tasks[selected_task.task_id] = selected_task

            # =============================================
            # FMS AMR 상태 갱신
            # =============================================

            self.amr_states[request.amr_id] = {
                "state": "BUSY",
                "status": "TASK_ASSIGNED",
                "current_task_id": selected_task.task_id,
                "load_state": request.load_state,
            }

            self.get_logger().info(
                f"[TASK ASSIGNED] "
                f"{selected_task.task_id} -> {request.amr_id}"
            )

            self.get_logger().info(
                f"Pickup   : {pickup_location.location_id} "
                f"({pickup_location.x:.2f}, {pickup_location.y:.2f})"
            )

            self.get_logger().info(
                f"Delivery : {delivery_location.location_id} "
                f"({delivery_location.x:.2f}, {delivery_location.y:.2f})"
            )

            self.log_queue_summary()

            return response

        except Exception as error:

            response.has_task = False
            response.message = str(error)

            self.get_logger().error(
                f"cuOpt optimization failed: {error}"
            )
            return response

        finally:
            self.is_optimizing = False


def main(args: Sequence[str] | None = None) -> None:

    rclpy.init(args=args)

    node = FleetManagementSystem()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
