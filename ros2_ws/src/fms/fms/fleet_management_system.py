#!/usr/bin/env python3
"""NodeMap 기반 작업큐, cuOpt 경로계획, AMR 요청을 관리하는 FMS."""

from __future__ import annotations

from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from interfaces.msg import NodeMapChanged
from interfaces.srv import GetNodeMap, RequestTask

from fms.NodeMapGraph import (
    EdgeData,
    NodeData,
    NodeMapGraphManager,
)
from fms.TaskManager import (
    AMRState,
    OptimizationRequest,
    OptimizationResult,
    TaskManager,
)


# 시나리오 검증 중에만 True로 사용한다.
TEST_MODE = True


class FleetManagementSystem(Node):

    # =====================================================
    # 기본 설정
    # =====================================================

    QUEUE_CAPACITY = 10

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

        self.task_manager = TaskManager(self.QUEUE_CAPACITY)
        self.latest_plan: OptimizationResult | None = None
        self.is_optimizing = False
        self.validation_task_created = False

        # =================================================
        # AMR 상태 저장
        # =================================================

        self.amr_states: dict[str, dict[str, str]] = {}

        # =================================================
        # Isaac Sim에서 받은 NodeMap Runtime 데이터
        # =================================================

        
        self.node_map_graph = NodeMapGraphManager()
        self.expected_node_map_revision: int | None = None     # 노드맵 버전번호
        self.waiting_nodemap_response = False                  # NodeMap 응답 대기 상태
        self.ready_to_nodemap_service = False                  # Service 준비 대기 상태

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
        self.get_logger().info("NodeMap-based FMS started")
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
        if self.waiting_nodemap_response:
            return
        
        if (
            self.node_map_graph.nodes
            and self.expected_node_map_revision is None
            and not force
        ):
            return

        if not self.node_map_client.service_is_ready():
            if not self.ready_to_nodemap_service:
                self.get_logger().info(
                    f"[NODE MAP] Waiting for {self.NODE_MAP_SERVICE}"
                )
                self.ready_to_nodemap_service = True
            return

        self.ready_to_nodemap_service = False
        self.waiting_nodemap_response = True

        self.get_logger().info("[NODE MAP] Requesting NodeMap from Isaac Sim")
        future = self.node_map_client.call_async(GetNodeMap.Request())
        future.add_done_callback(self.node_map_response_callback)




    def node_map_response_callback(self, future) -> None:
        self.waiting_nodemap_response = False

        try:
            response = future.result()
            if response is None:
                raise RuntimeError("GetNodeMap returned no response.")
            if not response.success:
                raise RuntimeError(response.message or "Isaac Sim rejected the request.")

            # 노드, 엣지 데이터들 파싱 및 구조화
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


            # 노드맵 구성 및 상태 업데이트
            self.node_map_graph.update_nodemap(
                nodes,
                edges,
                response_revision,
            )
        except Exception as error:
            self.get_logger().error(f"[NODE MAP] Request failed: {error}")
            self.node_map_request_timer.reset()
            return

        # 노드맵 요청 완료처리
        self.expected_node_map_revision = None
        self.node_map_request_timer.cancel()

        self.get_logger().info(
            f"[NODE MAP] Loaded revision={self.node_map_graph.revision}, "
            f"Nodes={len(nodes)}, Edges={len(edges)}"
        )

        # 결과확인
        self.print_node_map_csr()

        if TEST_MODE and not self.validation_task_created:
            self.create_validation_task()


    # NVIDIA cuOpt 작명규칙에 따른 노드, 엣지 데이터 구성.
    @staticmethod
    def _parse_node_map_response(response) -> tuple[
        dict[int, NodeData],
        list[EdgeData],
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

        nodes: dict[int, NodeData] = {}
        for index in range(node_count):
            node_id = int(response.node_ids[index])
            if node_id in nodes:
                raise ValueError(f"Duplicate Node ID: {node_id}")

            nodes[node_id] = NodeData(
                node_id=node_id,
                name=response.node_names[index],
                node_type=response.node_types[index],
                x=float(response.node_x[index]),
                y=float(response.node_y[index]),
                z=float(response.node_z[index]),
                available=True,
            )

        edges: list[EdgeData] = []
        for index in range(edge_count):
            start = int(response.edge_from[index])
            end = int(response.edge_to[index])
            if start not in nodes or end not in nodes:
                raise ValueError(f"Edge references unknown Node: {start} -> {end}")

            edges.append(
                EdgeData(
                    edge_id=index,
                    start=start,
                    end=end,
                    weight=float(response.edge_weights[index]),
                    bidirectional=bool(response.edge_bidirectional[index]),
                    available=True,
                    path_points=[
                        (nodes[start].x, nodes[start].y, nodes[start].z),
                        (nodes[end].x, nodes[end].y, nodes[end].z),
                    ],
                )
            )

        return nodes, edges

    def print_node_map_csr(self) -> None:
        """현재 NodeMap CSR을 FMS 실행 터미널에 출력한다."""
        csr = self.node_map_graph.get_csr()
        print(
            "\n========== NODE MAP CSR ==========\n"
            f"revision : {self.node_map_graph.revision}\n"
            f"node_ids : {csr.node_ids}\n"
            f"offsets  : {csr.offsets}\n"
            f"indices  : {csr.indices}\n"
            f"weights  : {csr.weights}\n"
            "==================================",
            flush=True,
        )

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



    def log_queue_summary(self) -> None:
        self.get_logger().info(
            f"[QUEUE] waiting={self.task_manager.waiting_count} "
            f"active={self.task_manager.active_count}"
        )

    def create_validation_task(self) -> None:
        """NodeMap의 연결 가능한 두 Node로 검증용 Task를 생성한다."""
        start_node_id, goal_node_id = (
            self.node_map_graph.choose_random_reachable_nodes()
        )
        start = self.node_map_graph.nodes[start_node_id]
        goal = self.node_map_graph.nodes[goal_node_id]
        task = self.task_manager.create_task(
            start,
            goal,
            task_id="validation_task_01",
            kit_id="VALIDATION",
        )
        self.validation_task_created = True

        self.get_logger().info(
            "\n========== VALIDATION TASK ==========\n"
            f"Task ID : {task.task_id}\n"
            f"Start   : {start.name} ({start.x:.2f}, {start.y:.2f}, {start.z:.2f})\n"
            f"Goal    : {goal.name} ({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f})\n"
            f"Status  : {task.status}\n"
            f"Queue   : {self.task_manager.waiting_count}/{self.QUEUE_CAPACITY}\n"
            "====================================="
        )

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
            finished_task = self.task_manager.complete_task(task_id)
            if finished_task is not None:
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

        if not self.task_manager.waiting_count:
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

        if not self.node_map_graph.nodes:
            response.has_task = False
            response.message = "NodeMap is not ready."
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

        self.get_logger().info(
            f"[CUOPT] Sending {self.task_manager.waiting_count} "
            f"tasks for {request.amr_id}"
        )

        self.is_optimizing = True
        try:
            from fms.cuopt_solver import CuOptSolver

            tasks = self.task_manager.get_waiting_tasks()
            optimization_request = OptimizationRequest(tasks, amr_state)
            self.latest_plan = CuOptSolver(
                optimization_request,
                self.node_map_graph,
            ).solve()
            if not self.latest_plan.ordered_tasks:
                raise RuntimeError("cuOpt returned no Task.")

            self.get_logger().info(
                "=== Optimized Task Order ==="
            )

            for ordered_task in self.latest_plan.ordered_tasks:
                self.get_logger().info(
                    f"{ordered_task.sequence:02d}. "
                    f"{ordered_task.task_id}: "
                    f"{ordered_task.route.start_node_id} -> "
                    f"{ordered_task.route.goal_node_id} "
                    f"(cost={ordered_task.route.total_cost:.3f})"
                )

            selected_plan = self.latest_plan.ordered_tasks[0]
            selected_id = selected_plan.task_id
            selected_task = next(task for task in tasks if task.task_id == selected_id)
            selected_task = self.task_manager.assign_task(
                selected_id,
                selected_plan.route,
                self.node_map_graph.revision,
            )
            start = selected_task.start
            goal = selected_task.goal
            task_route = selected_task.route

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
            response.pickup_id = start.name
            response.pickup_x = float(start.x)
            response.pickup_y = float(start.y)

            # Delivery
            response.delivery_id = goal.name
            response.delivery_x = float(goal.x)
            response.delivery_y = float(goal.y)

            response.node_map_revision = task_route.node_map_revision
            response.route_node_ids = list(task_route.node_ids)
            response.route_x = [point[0] for point in task_route.points]
            response.route_y = [point[1] for point in task_route.points]
            response.route_z = [point[2] for point in task_route.points]
            response.route_total_cost = task_route.total_cost

            response.message = (
                f"Assigned {selected_task.task_id} "
                f"to {request.amr_id}"
            )

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
                f"Start    : {start.name} "
                f"({start.x:.2f}, {start.y:.2f}, {start.z:.2f})"
            )

            self.get_logger().info(
                f"Goal     : {goal.name} "
                f"({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f})"
            )

            self.get_logger().info(
                f"Route    : {' -> '.join(map(str, task_route.node_ids))}"
            )

            self.get_logger().info(
                f"Cost     : {task_route.total_cost:.3f} "
                f"(revision={task_route.node_map_revision})"
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
