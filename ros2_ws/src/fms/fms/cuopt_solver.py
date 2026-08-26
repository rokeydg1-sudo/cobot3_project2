#!/usr/bin/env python3
"""Active NodeMap 비용으로 FMS Task의 수행 순서를 계산한다."""

import cudf
import numpy as np
from cuopt import routing

from fms.NodeMapGraph import NodeMapGraphManager, PlannedNodeRoute
from fms.TaskManager import (
    OptimizationRequest,
    OptimizationResult,
    OrderedTask,
    Task,
)


class CuOptSolver:
    """CSR 최단경로 비용을 cuOpt에 전달해 Task 순서를 최적화한다."""

    def __init__(
        self,
        request: OptimizationRequest,
        graph: NodeMapGraphManager,
        time_limit: float = 5.0,
    ) -> None:
        if not request.tasks:
            raise ValueError("Optimization requires at least one Task.")
        if not graph.get_csr().node_ids.size:
            raise ValueError("Active NodeMap CSR is empty.")

        self.request = request
        self.tasks = list(request.tasks)
        self.graph = graph
        self.time_limit = float(time_limit)
        self.recovery_node = graph.find_nearest_active_node(
            request.amr_state.x,
            request.amr_state.y,
        )
        self.task_routes: dict[str, PlannedNodeRoute] = {}
        self.cost_matrix: np.ndarray | None = None
        self.data_model = None
        self.solution = None

        self._validate_tasks()

    def _validate_tasks(self) -> None:
        task_ids: set[str] = set()
        active_node_ids = set(map(int, self.graph.get_csr().node_ids))

        for task in self.tasks:
            if task.task_id in task_ids:
                raise ValueError(f"Duplicate task_id: {task.task_id}")
            if task.start.node_id not in active_node_ids:
                raise ValueError(f"Task start Node is not active: {task.start.node_id}")
            if task.goal.node_id not in active_node_ids:
                raise ValueError(f"Task goal Node is not active: {task.goal.node_id}")

            task_ids.add(task.task_id)
            self.task_routes[task.task_id] = self.graph.create_route(
                task.start.node_id,
                task.goal.node_id,
            )

    def _path_cost(self, start_node_id: int, goal_node_id: int) -> float:
        return self.graph.find_shortest_path(start_node_id, goal_node_id)[1]

    def build_cost_matrix(self) -> np.ndarray:
        """AMR 복귀 Node와 Task 사이의 CSR 최단경로 비용을 구성한다."""
        location_count = len(self.tasks) + 1
        matrix = np.zeros((location_count, location_count), dtype=np.float32)

        for target_index, target_task in enumerate(self.tasks, start=1):
            matrix[0, target_index] = (
                self._path_cost(self.recovery_node.node_id, target_task.start.node_id)
                + self.task_routes[target_task.task_id].total_cost
            )

        for source_index, source_task in enumerate(self.tasks, start=1):
            matrix[source_index, 0] = self._path_cost(
                source_task.goal.node_id,
                self.recovery_node.node_id,
            )

            for target_index, target_task in enumerate(self.tasks, start=1):
                if source_index == target_index:
                    continue
                matrix[source_index, target_index] = (
                    self._path_cost(
                        source_task.goal.node_id,
                        target_task.start.node_id,
                    )
                    + self.task_routes[target_task.task_id].total_cost
                )

        self.cost_matrix = matrix
        return matrix

    def build_data_model(self):
        if self.cost_matrix is None:
            self.build_cost_matrix()

        task_count = len(self.tasks)
        self.data_model = routing.DataModel(task_count + 1, 1, task_count)
        self.data_model.add_cost_matrix(cudf.DataFrame(self.cost_matrix))
        self.data_model.set_vehicle_locations(
            cudf.Series([0], dtype="int32"),
            cudf.Series([0], dtype="int32"),
        )
        self.data_model.set_order_locations(
            cudf.Series(range(1, task_count + 1), dtype="int32")
        )
        return self.data_model

    def run_optimizer(self):
        if self.data_model is None:
            self.build_data_model()

        settings = routing.SolverSettings()
        settings.set_time_limit(self.time_limit)
        self.solution = routing.Solve(self.data_model, settings)
        status = int(self.solution.get_status())
        if status != 0:
            raise RuntimeError(
                f"cuOpt failed: status={status}, message={self.solution.get_message()}"
            )
        return self.solution

    def format_result(self) -> OptimizationResult:
        if self.solution is None or self.cost_matrix is None:
            raise RuntimeError("Optimizer has not been run.")

        route_nodes = [
            int(value)
            for value in self.solution.get_route()["location"].to_arrow().to_pylist()
        ]
        task_nodes = [node for node in route_nodes if node != 0]
        if sorted(task_nodes) != list(range(1, len(self.tasks) + 1)):
            raise RuntimeError("cuOpt result does not contain every Task exactly once.")

        ordered_tasks = tuple(
            OrderedTask(
                sequence=sequence,
                task_id=self.tasks[node - 1].task_id,
                route=self.task_routes[self.tasks[node - 1].task_id],
            )
            for sequence, node in enumerate(task_nodes, start=1)
        )
        total_cost = sum(
            float(self.cost_matrix[source, target])
            for source, target in zip(route_nodes, route_nodes[1:])
        )
        return OptimizationResult(
            ordered_tasks=ordered_tasks,
            recovery_node_id=self.recovery_node.node_id,
            total_cost=total_cost,
        )

    def solve(self) -> OptimizationResult:
        self.build_cost_matrix()
        self.build_data_model()
        self.run_optimizer()
        return self.format_result()
