import threading
import time

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from interfaces.action import DockDolly, VisualizeRoute
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from interfaces.srv import RequestTask


# =========================================================
# 이동 설정
# =========================================================

# NavigateToPose 한 구간(Pickup / Delivery)의 최대 대기 시간
MOVE_TIMEOUT_SEC = 60.0

# Vision Dolly Docking Action의 최대 대기 시간
DOCK_TIMEOUT_SEC = 120.0

# DockDolly Action Server 준비 대기 시간
DOCK_SERVER_WAIT_TIMEOUT_SEC = 5.0

# FMS에 다음 작업을 요청하는 주기
TASK_REQUEST_INTERVAL_SEC = 1.0

# 현재는 실제 Pick 장비가 없으므로
# Supermarket 도착 후 적재 시간을 임시로 사용
LOADING_TIME_SEC = 2.0


class AMRNode(Node):

    def __init__(self):

        super().__init__("amr_node")

        # =================================================
        # AMR Identity
        # =================================================

        self.amr_id = "AMR_01"

        self.declare_parameter(
            "dock_action_name",
            "/dock_dolly",
        )

        self.dock_action_name = str(
            self.get_parameter(
                "dock_action_name"
            ).value
        )


        # =================================================
        # AMR 상태
        # =================================================

        self.state = "IDLE"
        self.load_state = "EMPTY"

        self.current_task_id = ""
        self.current_kit_id = ""


        # =================================================
        # 내부 동작 상태
        # =================================================

        self.task_request_pending = False
        self.task_running = False


        # =================================================
        # Callback Group
        #
        # - /amr/odom
        # - FMS RequestTask
        # - Timer
        # - Nav2 Action
        #
        # 서로 독립적으로 처리
        # =================================================

        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()


        # =================================================
        # AMR -> FMS
        #
        # Pull 방식:
        # AMR이 IDLE일 때 다음 Task 요청
        # =================================================

        self.task_client = self.create_client(
            RequestTask,
            "/fms/request_task",
            callback_group=self.service_group,
        )

        self.route_visualizer_client = ActionClient(
            self,
            VisualizeRoute,
            "/visualize_route",
            callback_group=self.action_group,
        )


        # =================================================
        # AMR -> Nav2
        #
        # 기존 /amr/goal 직접 제어 제거
        #
        # 이제 목적지는 Nav2 NavigateToPose Action으로 전달
        # =================================================

        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self.action_group,
        )


        # =================================================
        # AMR -> Vision Docking
        #
        # Pickup 좌표까지의 Nav2 Goal이 성공한 뒤에만 실행한다.
        # =================================================

        self.dock_client = ActionClient(
            self,
            DockDolly,
            self.dock_action_name,
            callback_group=self.action_group,
        )


        # =================================================
        # Isaac Sim -> AMR
        #
        # Isaac ROS2 Bridge가 계속 발행하는 Odometry
        #
        # 이 값은:
        # - AMR 현재 위치 저장
        # - FMS Task 요청 시 현재 위치 전달
        # 에 사용
        #
        # 실제 이동 성공 판정은 더 이상 여기서 거리 계산하지 않고
        # NavigateToPose Action Result를 사용
        # =================================================

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/amr/odom",
            self.odom_callback,
            10,
            callback_group=self.odom_group,
        )


        # =================================================
        # AMR -> FMS
        #
        # 상태 변화 Event
        # =================================================

        self.status_publisher = self.create_publisher(
            String,
            "/amr/status",
            10,
        )


        # =================================================
        # 최신 AMR 위치
        # =================================================

        self.pose_lock = threading.Lock()
        self.latest_xy = None


        # =================================================
        # Task 상태 보호
        # =================================================

        self.task_lock = threading.Lock()


        # =================================================
        # IDLE일 때 FMS에 다음 작업 요청
        # =================================================

        self.task_request_timer = self.create_timer(
            TASK_REQUEST_INTERVAL_SEC,
            self.try_request_task,
            callback_group=self.timer_group,
        )


        # =================================================
        # 시작 로그
        # =================================================

        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "AMR Node started"
        )

        self.get_logger().info(
            f"AMR ID         : {self.amr_id}"
        )

        self.get_logger().info(
            "Task Service   : /fms/request_task"
        )

        self.get_logger().info(
            "Route Service  : /visualize_route"
        )

        self.get_logger().info(
            "Nav2 Action    : /navigate_to_pose"
        )

        self.get_logger().info(
            f"Dock Action    : {self.dock_action_name}"
        )

        self.get_logger().info(
            "Odom Topic     : /amr/odom"
        )

        self.get_logger().info(
            "Status Topic   : /amr/status"
        )

        self.get_logger().info(
            "Direct Goal    : /amr/goal REMOVED"
        )

        self.get_logger().info(
            "ExecuteMission : REMOVED"
        )

        self.get_logger().info(
            "================================="
        )


        # 최초 상태 알림
        self.publish_status(
            "READY"
        )


    # =====================================================
    # /amr/odom 수신
    # =====================================================

    def odom_callback(self, msg):

        x = float(
            msg.pose.pose.position.x
        )

        y = float(
            msg.pose.pose.position.y
        )


        with self.pose_lock:

            self.latest_xy = (
                x,
                y,
            )


    # =====================================================
    # 최신 위치 반환
    # =====================================================

    def get_current_position(self):

        with self.pose_lock:

            if self.latest_xy is None:

                return None


            return (
                self.latest_xy[0],
                self.latest_xy[1],
            )


    # =====================================================
    # AMR 상태 Event 발행
    # =====================================================

    def publish_status(
        self,
        status,
    ):

        msg = String()

        msg.data = (
            f"amr_id={self.amr_id},"
            f"state={self.state},"
            f"status={status},"
            f"task_id={self.current_task_id},"
            f"load_state={self.load_state}"
        )


        self.status_publisher.publish(
            msg
        )


        self.get_logger().info(
            f"[AMR STATUS] "
            f"amr_id={self.amr_id}, "
            f"state={self.state}, "
            f"status={status}, "
            f"task_id={self.current_task_id or '-'}, "
            f"load={self.load_state}"
        )


    # =====================================================
    # IDLE 상태일 때 FMS에 다음 Task 요청
    # =====================================================

    def try_request_task(self):

        # 현재 Task 수행 중
        if self.task_running:

            return


        # 이미 Service 요청 중
        if self.task_request_pending:

            return


        # ERROR 상태에서는 자동으로 새 Task를 받지 않음
        if self.state == "ERROR":

            return


        # IDLE 상태에서만 요청
        if self.state != "IDLE":

            return


        # /amr/odom이 아직 안 들어온 경우
        current_position = (
            self.get_current_position()
        )


        if current_position is None:

            return


        # FMS Service가 아직 준비되지 않은 경우
        if not self.task_client.service_is_ready():

            self.get_logger().info(
                "Waiting for FMS Task Service..."
            )

            return


        current_x, current_y = (
            current_position
        )


        # =================================================
        # RequestTask 요청 생성
        # =================================================

        request = RequestTask.Request()

        request.amr_id = self.amr_id
        request.state = self.state
        request.current_task_id = (
            self.current_task_id
        )

        request.x = float(
            current_x
        )

        request.y = float(
            current_y
        )

        request.load_state = (
            self.load_state
        )


        self.task_request_pending = True


        self.get_logger().info(
            f"[TASK REQUEST] "
            f"{self.amr_id} -> FMS "
            f"(x={current_x:.2f}, "
            f"y={current_y:.2f})"
        )


        future = self.task_client.call_async(
            request
        )


        future.add_done_callback(
            self.task_response_callback
        )


    # =====================================================
    # FMS Task 응답
    # =====================================================

    def task_response_callback(
        self,
        future,
    ):

        self.task_request_pending = False


        try:

            response = future.result()


        except Exception as error:

            self.get_logger().error(
                f"Task request failed: {error}"
            )

            return


        # FMS에 대기 Task가 없음
        if not response.has_task:

            self.get_logger().info(
                f"[NO TASK] "
                f"{response.message}"
            )

            return


        # =================================================
        # 새로운 Task 수신
        # =================================================

        with self.task_lock:

            if self.task_running:

                self.get_logger().warning(
                    "Task is already running. "
                    "Ignoring duplicated assignment."
                )

                return


            self.task_running = True

            self.current_task_id = (
                response.task_id
            )

            self.current_kit_id = (
                response.kit_id
            )

            self.state = "BUSY"


        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            "NEW TASK RECEIVED"
        )

        self.get_logger().info(
            f"Task ID         : "
            f"{response.task_id}"
        )

        self.get_logger().info(
            f"Kit ID          : "
            f"{response.kit_id}"
        )

        self.get_logger().info(
            f"Processing Time : "
            f"{response.processing_time:.1f}s"
        )

        self.get_logger().info(
            f"Pickup          : "
            f"{response.pickup_id} "
            f"({response.pickup_x:.2f}, "
            f"{response.pickup_y:.2f})"
        )

        self.get_logger().info(
            f"Delivery        : "
            f"{response.delivery_id} "
            f"({response.delivery_x:.2f}, "
            f"{response.delivery_y:.2f})"
        )

        self.get_logger().info(
            "================================="
        )


        self.publish_status(
            "TASK_ASSIGNED"
        )

        self.send_route_to_isaac(response)

    def send_route_to_isaac(self, task) -> None:
        """FMS에서 받은 계획 경로를 Isaac Sim에 전달한다."""
        route_size = len(task.route_node_ids)
        coordinate_sizes = (
            len(task.route_x),
            len(task.route_y),
            len(task.route_z),
        )

        if route_size == 0 or any(
            size not in (0, route_size) for size in coordinate_sizes
        ):
            self.handle_task_failure("Invalid Planned Route received from FMS.")
            return

        if len(set(coordinate_sizes)) != 1:
            self.handle_task_failure(
                "Planned Route XYZ arrays have different lengths."
            )
            return

        if not self.route_visualizer_client.server_is_ready():
            self.handle_task_failure("Waiting for Isaac Sim Route Action Server.")
            return

        goal = VisualizeRoute.Goal()
        goal.amr_id = self.amr_id
        goal.task_id = task.task_id
        goal.node_map_revision = task.node_map_revision
        goal.node_ids = list(task.route_node_ids)
        goal.node_x = list(task.route_x)
        goal.node_y = list(task.route_y)
        goal.node_z = list(task.route_z)

        self.get_logger().info(
            f"[PLANNED ROUTE] Sending to Isaac Sim: {goal.node_ids}"
        )
        future = self.route_visualizer_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self.route_goal_response_callback(result, task)
        )

    def route_goal_response_callback(self, future, task) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.handle_task_failure(f"Route visualization Goal failed: {error}")
            return

        if not goal_handle.accepted:
            self.handle_task_failure("Route visualization Goal was rejected.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.route_result_callback(result, task)
        )

    def route_result_callback(self, future, task) -> None:
        try:
            result = future.result().result
        except Exception as error:
            self.handle_task_failure(
                f"Route visualization Result failed: {error}"
            )
            return

        if not result.success:
            self.handle_task_failure(result.message)
            return

        self.get_logger().info(f"[PLANNED ROUTE] {result.message}")
        worker = threading.Thread(
            target=self.execute_task,
            args=(task,),
            daemon=True,
        )
        worker.start()


    # =====================================================
    # Task 실행
    #
    # FMS가 논리 Location -> 물리 좌표 변환 완료 후
    # pickup_x/y, delivery_x/y를 전달
    #
    # AMR은 받은 좌표를 Nav2에 그대로 Goal로 전달
    # =====================================================

    def execute_task(
        self,
        task,
    ):

        try:

            # =================================================
            # 1. Pickup 위치로 Nav2 이동
            # =================================================

            self.publish_status(
                "MOVING_TO_PICKUP"
            )


            pickup_success = (
                self.move_to_destination(

                    task.pickup_id,

                    task.pickup_x,

                    task.pickup_y,
                )
            )


            if not pickup_success:

                self.handle_task_failure(
                    f"Failed to move to "
                    f"{task.pickup_id}"
                )

                return


            # =================================================
            # Pickup 좌표는 현재 Pre-Docking pose로 취급한다.
            # Nav2 Goal이 종료된 뒤 Vision이 /cmd_vel 제어권을 가진다.
            # =================================================

            self.publish_status(
                "PRE_DOCKING"
            )


            docking_success = (
                self.dock_dolly()
            )


            if not docking_success:

                self.handle_task_failure(
                    "Dolly docking failed for "
                    f"task {self.current_task_id}"
                )

                return


            self.publish_status(
                "DOCKING_COMPLETE"
            )


            # =================================================
            # Pickup 도착
            # =================================================

            self.publish_status(
                "ARRIVED_PICKUP"
            )


            # =================================================
            # 2. Kit 적재
            #
            # 현재는 실제 Pick 장비가 없으므로 시간으로 가정
            # =================================================

            self.publish_status(
                "LOADING"
            )


            time.sleep(
                LOADING_TIME_SEC
            )


            self.load_state = "LOADED"


            self.publish_status(
                "LOAD_COMPLETE"
            )


            # =================================================
            # 3. Delivery 위치로 Nav2 이동
            # =================================================

            self.publish_status(
                "MOVING_TO_DELIVERY"
            )


            delivery_success = (
                self.move_to_destination(

                    task.delivery_id,

                    task.delivery_x,

                    task.delivery_y,
                )
            )


            if not delivery_success:

                self.handle_task_failure(
                    f"Failed to move to "
                    f"{task.delivery_id}"
                )

                return


            # =================================================
            # Delivery 도착
            # =================================================

            self.publish_status(
                "ARRIVED_DELIVERY"
            )


            # =================================================
            # 4. Delivery 완료
            # =================================================

            self.load_state = "EMPTY"


            self.publish_status(
                "DELIVERY_COMPLETE"
            )


            # =================================================
            # 5. Mission 완료
            # =================================================

            self.publish_status(
                "MISSION_COMPLETE"
            )


            self.get_logger().info(
                f"[TASK COMPLETE] "
                f"{self.current_task_id}"
            )


            # =================================================
            # 6. 다시 IDLE
            #
            # 다음 Timer Tick에서 새 Task Pull
            # =================================================

            with self.task_lock:

                self.current_task_id = ""
                self.current_kit_id = ""

                self.state = "IDLE"

                self.task_running = False


            self.publish_status(
                "IDLE"
            )


        except Exception as error:

            self.handle_task_failure(
                str(error)
            )


    # =====================================================
    # Task 실패 처리
    # =====================================================

    def handle_task_failure(
        self,
        message,
    ):

        self.state = "ERROR"


        self.publish_status(
            "TASK_FAILED"
        )


        self.get_logger().error(
            f"[TASK FAILED] {message}"
        )


        with self.task_lock:

            self.task_running = False


    # =====================================================
    # Nav2 Feedback
    #
    # NavigateToPose가 이동 중 남은 거리를 전달
    # 너무 많은 로그를 막기 위해 1초에 한 번만 출력
    # =====================================================

    def nav2_feedback_callback(
        self,
        feedback_msg,
    ):

        feedback = feedback_msg.feedback

        now = time.monotonic()


        if not hasattr(
            self,
            "_last_nav_feedback_log_time",
        ):

            self._last_nav_feedback_log_time = 0.0


        if (
            now
            - self._last_nav_feedback_log_time
            < 1.0
        ):

            return


        self._last_nav_feedback_log_time = now


        try:

            distance_remaining = float(
                feedback.distance_remaining
            )


            self.get_logger().info(
                f"[NAV2] "
                f"distance_remaining="
                f"{distance_remaining:.2f}m"
            )


        except Exception:

            pass

    # =====================================================
    # DockDolly Feedback
    #
    # Vision 상태와 최신 pose 오차를 0.75초에 한 번만 출력
    # =====================================================

    def dock_feedback_callback(
        self,
        feedback_msg,
    ):

        now = time.monotonic()

        if not hasattr(
            self,
            "_last_dock_feedback_log_time",
        ):

            self._last_dock_feedback_log_time = 0.0

        if (
            now
            - self._last_dock_feedback_log_time
            < 0.75
        ):

            return

        self._last_dock_feedback_log_time = now
        feedback = feedback_msg.feedback

        self.get_logger().info(
            "[DOCKING] "
            f"state={feedback.state}, "
            f"distance={feedback.distance_m:.3f}m, "
            f"lateral={feedback.lateral_m:+.3f}m, "
            f"yaw={feedback.yaw_deg:+.2f}deg"
        )

    # =====================================================
    # Vision Dolly Docking
    #
    # Worker Thread는 Event로 Goal/Result를 기다리고,
    # Action callback은 MultiThreadedExecutor에서 처리한다.
    # =====================================================

    def dock_dolly(self):

        self.get_logger().info(
            f"[DOCKING] Waiting for {self.dock_action_name}..."
        )

        if not self.dock_client.wait_for_server(
            timeout_sec=DOCK_SERVER_WAIT_TIMEOUT_SEC
        ):

            self.get_logger().error(
                "DockDolly Action Server is not available."
            )
            return False

        goal_msg = DockDolly.Goal()
        goal_msg.amr_id = self.amr_id
        goal_msg.task_id = self.current_task_id

        goal_response_event = threading.Event()
        result_event = threading.Event()

        goal_handle_box = {
            "handle": None,
        }
        result_box = {
            "status": None,
            "result": None,
            "error": None,
        }

        self._last_dock_feedback_log_time = 0.0

        try:
            send_goal_future = self.dock_client.send_goal_async(
                goal_msg,
                feedback_callback=self.dock_feedback_callback,
            )
        except Exception as error:
            self.get_logger().error(
                f"[DOCKING] Goal send failed: {error}"
            )
            return False

        def goal_response_callback(future):

            try:
                goal_handle = future.result()
                goal_handle_box["handle"] = goal_handle

                if not goal_handle.accepted:
                    self.get_logger().error(
                        "[DOCKING] Goal was rejected."
                    )
                    goal_response_event.set()
                    result_event.set()
                    return

                self.get_logger().info(
                    "[DOCKING] Goal accepted."
                )
                self.publish_status(
                    "DOCKING"
                )
                goal_response_event.set()

                result_future = goal_handle.get_result_async()

                def result_callback(done_future):

                    try:
                        wrapped_result = done_future.result()
                        result_box["status"] = wrapped_result.status
                        result_box["result"] = wrapped_result.result
                    except Exception as error:
                        result_box["error"] = error
                    finally:
                        result_event.set()

                result_future.add_done_callback(
                    result_callback
                )

            except Exception as error:
                result_box["error"] = error
                goal_response_event.set()
                result_event.set()

        send_goal_future.add_done_callback(
            goal_response_callback
        )

        if not goal_response_event.wait(timeout=10.0):
            self.get_logger().error(
                "[DOCKING] Goal response timeout."
            )
            return False

        goal_handle = goal_handle_box["handle"]

        if goal_handle is None or not goal_handle.accepted:
            return False

        if not result_event.wait(timeout=DOCK_TIMEOUT_SEC):
            self.get_logger().error(
                "[DOCKING] Action result timeout."
            )

            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass

            return False

        if result_box["error"] is not None:
            self.get_logger().error(
                "[DOCKING] Action error: "
                f"{result_box['error']}"
            )
            return False

        result = result_box["result"]
        status = result_box["status"]

        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and result is not None
            and result.success
        ):
            self.get_logger().info(
                f"[DOCKING] {result.message}"
            )
            return True

        message = (
            "No DockDolly result"
            if result is None
            else result.message
        )
        self.get_logger().error(
            "[DOCKING] Docking failed: "
            f"status={status}, message={message}"
        )
        return False


    # =====================================================
    # 목적지 이동
    #
    # 기존:
    # /amr/goal Publish
    # + /amr/odom 거리 직접 계산
    #
    # 현재:
    # NavigateToPose Action Goal
    # + Action Result로 성공 / 실패 판정
    # =====================================================

    def move_to_destination(
        self,
        destination_id,
        goal_x,
        goal_y,
    ):

        # =================================================
        # 1. Nav2 Action Server 확인
        # =================================================

        self.get_logger().info(
            f"[NAV2] Waiting for "
            f"/navigate_to_pose..."
        )


        if not self.nav2_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                "Nav2 NavigateToPose "
                "Action Server is not available."
            )

            return False


        # =================================================
        # 2. Goal 생성
        #
        # FMS 좌표와 Nav2 Map 좌표가 현재 동일한
        # 공장 좌표계를 사용하므로 frame_id = map
        #
        # orientation은 우선 yaw=0으로 사용
        # =================================================

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = (
            "map"
        )


        goal_msg.pose.pose.position.x = (
            float(goal_x)
        )

        goal_msg.pose.pose.position.y = (
            float(goal_y)
        )

        goal_msg.pose.pose.position.z = 0.0


        # yaw = 0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0


        self.get_logger().info(
            "================================="
        )

        self.get_logger().info(
            f"NAV2 GOAL: {destination_id}"
        )

        self.get_logger().info(
            f"x={goal_x:.2f}, "
            f"y={goal_y:.2f}"
        )

        self.get_logger().info(
            "frame=map"
        )

        self.get_logger().info(
            "================================="
        )


        # =================================================
        # 3. Goal 전송
        #
        # Executor는 main thread에서 계속 spin 중이므로
        # Action Future 완료는 callback으로 받고
        # Worker Thread는 Event로 대기
        # =================================================

        goal_response_event = (
            threading.Event()
        )

        result_event = (
            threading.Event()
        )


        goal_handle_box = {
            "handle": None
        }

        result_box = {
            "status": None,
            "result": None,
            "error": None,
        }


        send_goal_future = (
            self.nav2_client.send_goal_async(
                goal_msg,
                feedback_callback=(
                    self.nav2_feedback_callback
                ),
            )
        )


        def goal_response_callback(
            future,
        ):

            try:

                goal_handle = (
                    future.result()
                )


                goal_handle_box[
                    "handle"
                ] = goal_handle


                if not goal_handle.accepted:

                    self.get_logger().error(
                        f"[NAV2] Goal rejected: "
                        f"{destination_id}"
                    )

                    goal_response_event.set()
                    result_event.set()

                    return


                self.get_logger().info(
                    f"[NAV2] Goal accepted: "
                    f"{destination_id}"
                )


                goal_response_event.set()


                get_result_future = (
                    goal_handle.get_result_async()
                )


                def result_callback(
                    result_future,
                ):

                    try:

                        wrapped_result = (
                            result_future.result()
                        )


                        result_box[
                            "status"
                        ] = (
                            wrapped_result.status
                        )

                        result_box[
                            "result"
                        ] = (
                            wrapped_result.result
                        )


                    except Exception as error:

                        result_box[
                            "error"
                        ] = error


                    finally:

                        result_event.set()


                get_result_future.add_done_callback(
                    result_callback
                )


            except Exception as error:

                result_box[
                    "error"
                ] = error

                goal_response_event.set()
                result_event.set()


        send_goal_future.add_done_callback(
            goal_response_callback
        )


        # =================================================
        # 4. Goal Accept / Reject 대기
        # =================================================

        if not goal_response_event.wait(
            timeout=10.0
        ):

            self.get_logger().error(
                f"[NAV2] Goal response timeout: "
                f"{destination_id}"
            )

            return False


        goal_handle = (
            goal_handle_box["handle"]
        )


        if (
            goal_handle is None

            or not goal_handle.accepted
        ):

            return False


        # =================================================
        # 5. Result 대기
        # =================================================

        if not result_event.wait(
            timeout=MOVE_TIMEOUT_SEC
        ):

            self.get_logger().error(
                f"[NAV2] Move timeout: "
                f"{destination_id}"
            )


            # Timeout이면 Nav2 Goal 취소 요청
            try:

                goal_handle.cancel_goal_async()


            except Exception:

                pass


            return False


        # =================================================
        # 6. Action 처리 중 Exception
        # =================================================

        if result_box["error"] is not None:

            self.get_logger().error(
                f"[NAV2] Action error: "
                f"{result_box['error']}"
            )

            return False


        # =================================================
        # 7. Action Result 판정
        # =================================================

        status = (
            result_box["status"]
        )


        if (
            status
            == GoalStatus.STATUS_SUCCEEDED
        ):

            current_position = (
                self.get_current_position()
            )


            if current_position is not None:

                current_x, current_y = (
                    current_position
                )


                self.get_logger().info(
                    f"[NAV2] Reached "
                    f"{destination_id}: "
                    f"x={current_x:.2f}, "
                    f"y={current_y:.2f}"
                )


            else:

                self.get_logger().info(
                    f"[NAV2] Reached "
                    f"{destination_id}"
                )


            return True


        # =================================================
        # 실패 상태
        # =================================================

        status_name = {
            GoalStatus.STATUS_UNKNOWN:
                "UNKNOWN",

            GoalStatus.STATUS_ACCEPTED:
                "ACCEPTED",

            GoalStatus.STATUS_EXECUTING:
                "EXECUTING",

            GoalStatus.STATUS_CANCELING:
                "CANCELING",

            GoalStatus.STATUS_SUCCEEDED:
                "SUCCEEDED",

            GoalStatus.STATUS_CANCELED:
                "CANCELED",

            GoalStatus.STATUS_ABORTED:
                "ABORTED",
        }.get(
            status,
            str(status),
        )


        self.get_logger().error(
            f"[NAV2] Navigation failed: "
            f"{destination_id}, "
            f"status={status_name}"
        )


        return False


def main(args=None):

    rclpy.init(
        args=args
    )


    node = AMRNode()


    # =====================================================
    # 여러 Callback 동시 처리
    #
    # - /amr/odom
    # - RequestTask Service response
    # - Task Request Timer
    # - NavigateToPose Action callbacks
    # =====================================================

    executor = MultiThreadedExecutor(
        num_threads=4
    )


    executor.add_node(
        node
    )


    try:

        executor.spin()


    except KeyboardInterrupt:

        pass


    finally:

        executor.shutdown()

        node.destroy_node()


        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
