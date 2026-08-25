#!/usr/bin/env python3

"""Scenario 0에서 운송 Task의 실행 우선순위를 계산하는 cuOpt Solver."""

from __future__ import annotations

import cudf
import numpy as np

from cuopt import routing

from fms.defined import (
    LOCATION_BY_ID,
    PARTS_SUPERMARKET,
    OptimizationRequest,
    OptimizationResult,
    OrderedTask,
    Task,
)


class CuOptSolver:

    """
    거리, 긴급도, Deadline, Processing Time을 고려해
    Task 수행 순서를 결정한다.

    중요:
    cuOpt는 실제 Nav2 Path를 계산하지 않는다.

    cuOpt 결과:
        Task 순서 + 논리적인 Delivery Cell

    실제 물리좌표 변환:
        FMS

    실제 주행경로:
        Nav2
    """

    # =====================================================
    # Scenario 0 설정
    # =====================================================

    AMR_SPEED_MPS = 1.0

    # Parts Supermarket에서 Kit 적재에 걸리는 시간
    #
    # 현재 AMR Node에서도 2초를 사용하므로 동일하게 맞춤
    PICKUP_SERVICE_TIME_SEC = 2


    # =====================================================
    # 긴급도별 목표 완료시간
    #
    # requested_at 기준
    # urgency가 높을수록 더 빠른 완료를 요구
    # =====================================================

    URGENCY_TARGET_SECONDS = {
        1: 600,
        2: 520,
        3: 450,
        4: 380,
        5: 320,
    }


    def __init__(
        self,
        request: OptimizationRequest,
        time_limit: float = 5.0,
    ) -> None:

        if not 1 <= len(request.tasks) <= 10:

            raise ValueError(
                "Scenario 0 requires 1 to 10 tasks."
            )


        self.request = request

        self.tasks = list(
            request.tasks
        )

        self.time_limit = float(
            time_limit
        )


        self.cost_matrix: np.ndarray | None = None

        self.transit_time_matrix: np.ndarray | None = None

        self.data_model = None

        self.solution = None


        self._validate_tasks()


    # =====================================================
    # Task 검증
    # =====================================================

    def _validate_tasks(self) -> None:

        task_ids: set[str] = set()


        for task in self.tasks:

            # -------------------------------------------------
            # 중복 Task ID
            # -------------------------------------------------

            if task.task_id in task_ids:

                raise ValueError(
                    f"Duplicate task_id: {task.task_id}"
                )


            task_ids.add(
                task.task_id
            )


            # -------------------------------------------------
            # Delivery Cell 검증
            # -------------------------------------------------

            if task.delivery_cell not in {
                "cell_a",
                "cell_b",
                "cell_c",
            }:

                raise ValueError(
                    f"Unknown Assembly Cell: "
                    f"{task.delivery_cell}"
                )


    # =====================================================
    # Euclidean Distance
    # =====================================================

    @staticmethod
    def _distance(
        source_x: float,
        source_y: float,
        target_x: float,
        target_y: float,
    ) -> float:

        return float(
            np.hypot(
                target_x - source_x,
                target_y - source_y,
            )
        )


    # =====================================================
    # Cost / Transit Time Matrix 생성
    # =====================================================

    def build_matrices(
        self,
    ) -> tuple[np.ndarray, np.ndarray]:

        """
        cuOpt Task 간 이동 비용을 생성한다.

        Node 0
            현재 AMR 위치

        Node 1 ~ N
            각 Task의 Delivery Cell


        Task 수행 규칙:

            현재 위치
                ↓
            Parts Supermarket
                ↓
            Delivery Cell


        Task i 이후 Task j:

            Cell i
                ↓
            Parts Supermarket
                ↓
            Cell j


        즉 Parts Supermarket 방문은
        cuOpt 결과에 별도 Node로 반환하지 않고
        운송 비용 계산에 포함한다.

        실제 Mission 생성 시에는 FMS가:

            Pickup = SP
            Delivery = Cell

        형태로 AMR에 전달한다.
        """

        node_count = (
            len(self.tasks) + 1
        )


        # =================================================
        # 거리 Cost Matrix
        # =================================================

        matrix = np.zeros(
            (
                node_count,
                node_count,
            ),
            dtype=np.float32,
        )


        supermarket = (
            PARTS_SUPERMARKET
        )


        amr = (
            self.request.amr_state
        )


        # =================================================
        # 현재 AMR 위치 → 첫 Task
        #
        # AMR
        # ↓
        # SP
        # ↓
        # Cell
        # =================================================

        for (
            target_node,
            target_task,
        ) in enumerate(
            self.tasks,
            start=1,
        ):

            target_cell = (
                LOCATION_BY_ID[
                    target_task.delivery_cell
                ]
            )


            matrix[
                0,
                target_node,
            ] = (

                self._distance(
                    amr.x,
                    amr.y,
                    supermarket.x,
                    supermarket.y,
                )

                +

                self._distance(
                    supermarket.x,
                    supermarket.y,
                    target_cell.x,
                    target_cell.y,
                )
            )


        # =================================================
        # Task → 다음 Task
        #
        # Cell i
        # ↓
        # SP
        # ↓
        # Cell j
        # =================================================

        for (
            source_node,
            source_task,
        ) in enumerate(
            self.tasks,
            start=1,
        ):

            source_cell = (
                LOCATION_BY_ID[
                    source_task.delivery_cell
                ]
            )


            # =============================================
            # 마지막 Task 완료 후
            # AMR 시작 위치로 복귀하는 비용
            #
            # Scenario 0 Solver의 종료 조건용
            # =============================================

            matrix[
                source_node,
                0,
            ] = self._distance(

                source_cell.x,
                source_cell.y,

                amr.x,
                amr.y,
            )


            # =============================================
            # 다른 Task로 이동
            # =============================================

            for (
                target_node,
                target_task,
            ) in enumerate(
                self.tasks,
                start=1,
            ):

                if (
                    source_node
                    == target_node
                ):

                    continue


                target_cell = (
                    LOCATION_BY_ID[
                        target_task.delivery_cell
                    ]
                )


                matrix[
                    source_node,
                    target_node,
                ] = (

                    self._distance(

                        source_cell.x,
                        source_cell.y,

                        supermarket.x,
                        supermarket.y,
                    )

                    +

                    self._distance(

                        supermarket.x,
                        supermarket.y,

                        target_cell.x,
                        target_cell.y,
                    )
                )


        self.cost_matrix = (
            matrix
        )


        # =================================================
        # 거리 → 이동 시간
        # =================================================

        transit_time_matrix = np.ceil(

            matrix
            / self.AMR_SPEED_MPS

        ).astype(
            np.float32
        )


        # =================================================
        # Pickup 적재시간 추가
        #
        # 모든 Task 수행 전에는 SP Pickup이 있으므로
        # Task로 이동하는 모든 Arc에
        # 적재 시간을 추가한다.
        #
        # target_node == 0은
        # 종료 / 복귀이므로 추가하지 않음
        # =================================================

        for source_node in range(
            node_count
        ):

            for target_node in range(
                1,
                node_count,
            ):

                if (
                    source_node
                    == target_node
                ):

                    continue


                transit_time_matrix[
                    source_node,
                    target_node,
                ] += (
                    self.PICKUP_SERVICE_TIME_SEC
                )


        self.transit_time_matrix = (
            transit_time_matrix
        )


        return (
            self.cost_matrix,
            self.transit_time_matrix,
        )


    # =====================================================
    # Task의 가장 늦은 Delivery 가능 시점
    # =====================================================

    def _latest_delivery_time(
        self,
        task: Task,
    ) -> int:

        """
        Task가 deadline 안에 Assembly Processing까지
        완료되기 위한 가장 늦은 배송 시각을 계산한다.

        예:

            deadline = 100초
            processing_time = 10초

        → 늦어도 90초에는 Kit가 Cell에 도착해야 함


        중요:

        processing_time 동안 AMR이 Cell에
        대기한다는 의미가 아니다.

        배송 후 AMR은 다음 Task를 수행할 수 있고,
        Assembly Cell이 독립적으로 Processing한다.
        """


        urgency_deadline = (

            task.requested_at

            + self.URGENCY_TARGET_SECONDS[
                task.urgency
            ]
        )


        completion_deadline = min(

            task.deadline,

            urgency_deadline,
        )


        latest_delivery = (

            completion_deadline

            - task.processing_time
        )


        if (
            latest_delivery
            < task.requested_at
        ):

            raise ValueError(

                f"Task {task.task_id} "
                f"cannot finish before its deadline."
            )


        return int(
            latest_delivery
        )


    # =====================================================
    # cuOpt DataModel 생성
    # =====================================================

    def build_data_model(
        self,
    ):

        if (
            self.cost_matrix is None
            or
            self.transit_time_matrix is None
        ):

            self.build_matrices()


        task_count = (
            len(self.tasks)
        )


        # =================================================
        # Scenario 0
        #
        # Vehicle = 1
        # Task = 1 ~ 10
        # =================================================

        self.data_model = (
            routing.DataModel(

                task_count + 1,

                1,

                task_count,
            )
        )


        # =================================================
        # 거리 Cost
        # =================================================

        self.data_model.add_cost_matrix(

            cudf.DataFrame(
                self.cost_matrix
            )
        )


        # =================================================
        # 이동 시간
        # =================================================

        self.data_model.add_transit_time_matrix(

            cudf.DataFrame(
                self.transit_time_matrix
            )
        )


        # =================================================
        # AMR 시작 / 종료 Node
        # =================================================

        self.data_model.set_vehicle_locations(

            cudf.Series(
                [0],
                dtype="int32",
            ),

            cudf.Series(
                [0],
                dtype="int32",
            ),
        )


        # =================================================
        # Task Node 설정
        #
        # node 1 → tasks[0]
        # node 2 → tasks[1]
        # ...
        # =================================================

        self.data_model.set_order_locations(

            cudf.Series(

                range(
                    1,
                    task_count + 1,
                ),

                dtype="int32",
            )
        )


        # =================================================
        # Task Time Window
        #
        # 요청 시간 이후부터
        # latest delivery 이전까지 배송
        # =================================================

        self.data_model.set_order_time_windows(

            cudf.Series(

                [
                    task.requested_at
                    for task in self.tasks
                ],

                dtype="int32",
            ),

            cudf.Series(

                [
                    self._latest_delivery_time(
                        task
                    )
                    for task in self.tasks
                ],

                dtype="int32",
            ),
        )


        # =================================================
        # Delivery Cell에서의 AMR Service Time
        #
        # 현재 Scenario 0에서는 0초.
        #
        # processing_time은 Assembly 작업시간이므로
        # AMR을 Cell에 묶어두지 않는다.
        #
        # 추후 실제 하역시간이 생기면
        # DELIVERY_SERVICE_TIME 등을 별도 추가 가능
        # =================================================

        self.data_model.set_order_service_times(

            cudf.Series(

                [
                    0
                    for _ in self.tasks
                ],

                dtype="int32",
            )
        )


        return (
            self.data_model
        )


    # =====================================================
    # cuOpt 실행
    # =====================================================

    def run_optimizer(
        self,
    ):

        if (
            self.data_model is None
        ):

            self.build_data_model()


        settings = (
            routing.SolverSettings()
        )


        settings.set_time_limit(
            self.time_limit
        )


        self.solution = routing.Solve(

            self.data_model,

            settings,
        )


        status = int(
            self.solution.get_status()
        )


        if (
            status != 0
        ):

            raise RuntimeError(

                f"cuOpt failed: "
                f"status={status}, "
                f"message="
                f"{self.solution.get_message()}"
            )


        return (
            self.solution
        )


    # =====================================================
    # cuOpt 결과 변환
    # =====================================================

    def format_result(
        self,
    ) -> OptimizationResult:

        if (
            self.solution is None
            or
            self.cost_matrix is None
        ):

            raise RuntimeError(
                "Optimizer has not been run."
            )


        route_df = (
            self.solution.get_route()
        )


        route_nodes = [

            int(value)

            for value in (
                route_df[
                    "location"
                ]
                .to_arrow()
                .to_pylist()
            )
        ]


        # =================================================
        # node 0은 AMR Start
        #
        # 실제 결과에는 Task Node만 사용
        # =================================================

        task_nodes = [

            node

            for node in route_nodes

            if node != 0
        ]


        if (
            len(task_nodes)
            != len(self.tasks)
        ):

            raise RuntimeError(

                "cuOpt result does not contain "
                "every Task exactly once."
            )


        # =================================================
        # 중요:
        #
        # cuOpt 결과는 논리적인 정보만 반환
        #
        # x / y / yaw ❌
        #
        # 물리좌표 변환은 FMS가 담당
        # =================================================

        ordered_tasks = tuple(

            OrderedTask(

                sequence=sequence,

                task_id=(
                    self.tasks[
                        node - 1
                    ].task_id
                ),

                delivery_cell=(
                    self.tasks[
                        node - 1
                    ].delivery_cell
                ),
            )

            for (
                sequence,
                node,
            ) in enumerate(
                task_nodes,
                start=1,
            )
        )


        # =================================================
        # 전체 거리
        # =================================================

        total_distance = sum(

            float(
                self.cost_matrix[
                    source,
                    target,
                ]
            )

            for (
                source,
                target,
            ) in zip(
                route_nodes,
                route_nodes[1:],
            )
        )


        return OptimizationResult(

            success=True,

            message=(
                f"Optimized "
                f"{len(ordered_tasks)} tasks."
            ),

            ordered_tasks=(
                ordered_tasks
            ),

            total_distance=(
                total_distance
            ),
        )


    # =====================================================
    # 전체 Solve
    # =====================================================

    def solve(
        self,
    ) -> OptimizationResult:

        self.build_matrices()

        self.build_data_model()

        self.run_optimizer()

        return self.format_result()