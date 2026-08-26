#!/usr/bin/env python3
"""Execute the Scenario 0 AMR mission from FMS assignment to delivery."""

from dataclasses import dataclass
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from interfaces.action import DockDolly, LiftDolly, VisualizeRoute
from interfaces.srv import RequestTask
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from amr_control.mission_utils import (
    navigation_waypoints,
    quaternion_yaw,
    quaternion_z_w,
    traveled_distance,
    validate_mission_routes,
    validate_route,
)


@dataclass(frozen=True)
class MissionRoute:
    """One validated NodeMap route returned by FMS."""

    node_ids: tuple[int, ...]
    points: tuple[tuple[float, float, float], ...]
    total_cost: float


class AMRNode(Node):
    """Run one assigned Dolly pickup/delivery mission at a time."""

    def __init__(self, parameter_overrides=None) -> None:
        """Create parameterized ROS clients, publishers, and mission state."""
        super().__init__(
            "amr_node",
            parameter_overrides=parameter_overrides,
        )

        self._declare_parameters()
        self._read_parameters()

        self.state = "IDLE"
        self.load_state = "EMPTY"
        self.current_task_id = ""
        self.current_kit_id = ""
        self.task_request_pending = False
        self.task_running = False

        self.pose_lock = threading.Lock()
        self.latest_pose: tuple[float, float, float] | None = None
        self.task_lock = threading.Lock()
        self._shutdown_requested = threading.Event()

        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()

        self.task_client = self.create_client(
            RequestTask,
            self.fms_service_name,
            callback_group=self.service_group,
        )
        self.route_visualizer_client = ActionClient(
            self,
            VisualizeRoute,
            self.visualize_route_action_name,
            callback_group=self.action_group,
        )
        self.nav2_client = ActionClient(
            self,
            NavigateThroughPoses,
            self.nav2_action_name,
            callback_group=self.action_group,
        )
        self.dock_client = ActionClient(
            self,
            DockDolly,
            self.dock_action_name,
            callback_group=self.action_group,
        )
        self.lift_client = ActionClient(
            self,
            LiftDolly,
            self.lift_action_name,
            callback_group=self.action_group,
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
            callback_group=self.odom_group,
        )
        self.status_publisher = self.create_publisher(
            String,
            "/amr/status",
            10,
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )
        self.task_request_timer = self.create_timer(
            self.task_request_interval_s,
            self._try_request_task,
            callback_group=self.timer_group,
        )

        self.get_logger().info(
            "AMR Node started: "
            f"fms={self.fms_service_name}, "
            f"nav2={self.nav2_action_name}, "
            f"dock={self.dock_action_name}, "
            f"lift={self.lift_action_name}, "
            f"odom={self.odom_topic}, cmd_vel={self.cmd_vel_topic}"
        )
        self._publish_status("READY")

    def _declare_parameters(self) -> None:
        self.declare_parameter("amr_id", "AMR_01")
        self.declare_parameter("odom_topic", "/amr/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("nav2_action_name", "/navigate_through_poses")
        self.declare_parameter("dock_action_name", "/dock_dolly")
        self.declare_parameter("lift_action_name", "/lift_dolly")
        self.declare_parameter("fms_service_name", "/fms/request_task")
        self.declare_parameter(
            "visualize_route_action_name",
            "/visualize_route",
        )
        self.declare_parameter("nav_frame", "map")
        self.declare_parameter("move_timeout_s", 60.0)
        self.declare_parameter("dock_timeout_s", 120.0)
        self.declare_parameter("lift_timeout_s", 30.0)
        self.declare_parameter("action_server_wait_timeout_s", 5.0)
        self.declare_parameter("task_request_interval_s", 1.0)
        self.declare_parameter("return_distance_m", 3.0)
        self.declare_parameter("return_speed_mps", 0.20)
        self.declare_parameter("return_timeout_s", 30.0)
        self.declare_parameter("reverse_control_hz", 20.0)

    def _read_parameters(self) -> None:
        self.amr_id = str(self.get_parameter("amr_id").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.nav2_action_name = str(
            self.get_parameter("nav2_action_name").value
        )
        self.dock_action_name = str(
            self.get_parameter("dock_action_name").value
        )
        self.lift_action_name = str(
            self.get_parameter("lift_action_name").value
        )
        self.fms_service_name = str(
            self.get_parameter("fms_service_name").value
        )
        self.visualize_route_action_name = str(
            self.get_parameter("visualize_route_action_name").value
        )
        self.nav_frame = str(self.get_parameter("nav_frame").value)
        self.move_timeout_s = float(
            self.get_parameter("move_timeout_s").value
        )
        self.dock_timeout_s = float(
            self.get_parameter("dock_timeout_s").value
        )
        self.lift_timeout_s = float(
            self.get_parameter("lift_timeout_s").value
        )
        self.action_server_wait_timeout_s = float(
            self.get_parameter("action_server_wait_timeout_s").value
        )
        self.task_request_interval_s = float(
            self.get_parameter("task_request_interval_s").value
        )
        self.return_distance_m = float(
            self.get_parameter("return_distance_m").value
        )
        self.return_speed_mps = abs(
            float(self.get_parameter("return_speed_mps").value)
        )
        self.return_timeout_s = float(
            self.get_parameter("return_timeout_s").value
        )
        self.reverse_control_hz = float(
            self.get_parameter("reverse_control_hz").value
        )

        positive_parameters = {
            "move_timeout_s": self.move_timeout_s,
            "dock_timeout_s": self.dock_timeout_s,
            "lift_timeout_s": self.lift_timeout_s,
            "action_server_wait_timeout_s": self.action_server_wait_timeout_s,
            "task_request_interval_s": self.task_request_interval_s,
            "return_distance_m": self.return_distance_m,
            "return_speed_mps": self.return_speed_mps,
            "return_timeout_s": self.return_timeout_s,
            "reverse_control_hz": self.reverse_control_hz,
        }
        invalid = [
            name for name, value in positive_parameters.items()
            if value <= 0
        ]
        if invalid:
            raise ValueError(
                "AMR parameters must be positive: " + ", ".join(invalid)
            )

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = quaternion_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        with self.pose_lock:
            self.latest_pose = (
                float(position.x),
                float(position.y),
                float(yaw),
            )

    def _current_pose(self) -> tuple[float, float, float] | None:
        with self.pose_lock:
            if self.latest_pose is None:
                return None
            return tuple(self.latest_pose)

    def _publish_status(self, status: str) -> None:
        with self.task_lock:
            state = self.state
            task_id = self.current_task_id
            load_state = self.load_state
        message = String()
        message.data = (
            f"amr_id={self.amr_id},"
            f"state={state},"
            f"status={status},"
            f"task_id={task_id},"
            f"load_state={load_state}"
        )
        self.status_publisher.publish(message)
        self.get_logger().info(f"[AMR STATUS] {message.data}")

    def _try_request_task(self) -> None:
        with self.task_lock:
            if (
                self.state != "IDLE"
                or self.task_running
                or self.task_request_pending
            ):
                return

        current_pose = self._current_pose()
        if current_pose is None:
            return
        if not self.task_client.service_is_ready():
            self.get_logger().info("Waiting for FMS Task Service...")
            return

        with self.task_lock:
            if (
                self.state != "IDLE"
                or self.task_running
                or self.task_request_pending
            ):
                return
            request_state = self.state
            request_task_id = self.current_task_id
            request_load_state = self.load_state
            self.task_request_pending = True

        request = RequestTask.Request()
        request.amr_id = self.amr_id
        request.state = request_state
        request.current_task_id = request_task_id
        request.x = current_pose[0]
        request.y = current_pose[1]
        request.load_state = request_load_state

        try:
            future = self.task_client.call_async(request)
        except Exception as error:
            with self.task_lock:
                self.task_request_pending = False
            self.get_logger().error(f"Task request failed to start: {error}")
            return
        future.add_done_callback(self._task_response_callback)

    def _task_response_callback(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:
            with self.task_lock:
                self.task_request_pending = False
            self.get_logger().error(f"Task request failed: {error}")
            return

        if response is None:
            with self.task_lock:
                self.task_request_pending = False
            self.get_logger().error("Task request returned no response.")
            return
        if not response.has_task:
            with self.task_lock:
                self.task_request_pending = False
            self.get_logger().info(f"[NO TASK] {response.message}")
            return

        with self.task_lock:
            if not self.task_request_pending:
                self.get_logger().warning(
                    "Ignoring stale Task response."
                )
                return
            if self.task_running or self.state != "IDLE":
                self.task_request_pending = False
                self.get_logger().warning(
                    "Ignoring duplicate Task assignment."
                )
                return
            self.current_task_id = response.task_id
            self.current_kit_id = response.kit_id
            self.state = "BUSY"
            self.task_running = True
            self.task_request_pending = False

        try:
            approach_route, delivery_route = self._routes_from_response(
                response
            )
        except Exception as error:
            self._handle_task_failure(f"Invalid FMS route contract: {error}")
            return

        self._publish_status("TASK_ASSIGNED")
        worker = threading.Thread(
            target=self._execute_task,
            args=(response, approach_route, delivery_route),
            daemon=True,
        )
        worker.start()

    @staticmethod
    def _routes_from_response(response) -> tuple[MissionRoute, MissionRoute]:
        approach_ids = tuple(
            int(value)
            for value in response.approach_route_node_ids
        )
        delivery_ids = tuple(int(value) for value in response.route_node_ids)
        approach_points = validate_route(
            approach_ids,
            response.approach_route_x,
            response.approach_route_y,
            response.approach_route_z,
        )
        delivery_points = validate_route(
            delivery_ids,
            response.route_x,
            response.route_y,
            response.route_z,
        )
        validate_mission_routes(
            (approach_ids, approach_points),
            (delivery_ids, delivery_points),
        )
        return (
            MissionRoute(
                approach_ids,
                approach_points,
                float(response.approach_route_total_cost),
            ),
            MissionRoute(
                delivery_ids,
                delivery_points,
                float(response.route_total_cost),
            ),
        )

    def _execute_task(
        self,
        task,
        approach_route: MissionRoute,
        delivery_route: MissionRoute,
    ) -> None:
        try:
            self._publish_status("MOVING_TO_WAYPOINT")
            if not self._execute_route(task, approach_route, "approach"):
                self._handle_task_failure("Approach route execution failed.")
                return
            self._publish_status("ARRIVED_WAYPOINT")

            self._publish_status("PRE_DOCKING")
            if not self._dock_dolly():
                self._handle_task_failure("DockDolly failed.")
                return
            self._publish_status("DOCKING_COMPLETE")

            self._publish_status("LIFTING_UP")
            if not self._lift_dolly(LiftDolly.Goal.LIFT_UP):
                self._handle_task_failure("LiftDolly LIFT_UP failed.")
                return
            self.load_state = "LOADED"
            self._publish_status("LIFT_UP_COMPLETE")

            self._publish_status("RETURNING_TO_WAYPOINT")
            if not self._return_to_waypoint():
                self._handle_task_failure("Odometry reverse return failed.")
                return
            self._publish_status("RETURNED_TO_WAYPOINT")

            self._publish_status("MOVING_TO_DELIVERY")
            if not self._execute_route(task, delivery_route, "delivery"):
                self._handle_task_failure("Delivery route execution failed.")
                return
            self._publish_status("ARRIVED_DELIVERY")

            self._publish_status("LIFTING_DOWN")
            if not self._lift_dolly(LiftDolly.Goal.LIFT_DOWN):
                self._handle_task_failure("LiftDolly LIFT_DOWN failed.")
                return
            self.load_state = "EMPTY"
            self._publish_status("LIFT_DOWN_COMPLETE")
            self._publish_status("DELIVERY_COMPLETE")
            self._publish_status("MISSION_COMPLETE")
            self.get_logger().info(
                f"[TASK COMPLETE] {self.current_task_id}"
            )
            self._finish_task()
        except Exception as error:
            self._handle_task_failure(str(error))

    def _execute_route(
        self,
        task,
        route: MissionRoute,
        route_name: str,
    ) -> bool:
        if not self._visualize_route(task, route, route_name):
            return False
        return self._navigate_route(route, route_name)

    def _visualize_route(
        self,
        task,
        route: MissionRoute,
        route_name: str,
    ) -> bool:
        goal = VisualizeRoute.Goal()
        goal.amr_id = self.amr_id
        goal.task_id = task.task_id
        goal.node_map_revision = task.node_map_revision
        goal.node_ids = list(route.node_ids)
        goal.node_x = [point[0] for point in route.points]
        goal.node_y = [point[1] for point in route.points]
        goal.node_z = [point[2] for point in route.points]

        outcome = self._run_action(
            self.route_visualizer_client,
            goal,
            self.move_timeout_s,
            f"VisualizeRoute({route_name})",
        )
        if outcome is None:
            return False
        status, result = outcome
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        return succeeded and result is not None and result.success

    def _navigate_route(self, route: MissionRoute, route_name: str) -> bool:
        waypoint_specs = navigation_waypoints(route.points)
        if not waypoint_specs:
            current_pose = self._current_pose()
            current_yaw = None if current_pose is None else current_pose[2]
            self.get_logger().info(
                f"[NAV2] {route_name} route already at goal; "
                f"keeping odom yaw={current_yaw}"
            )
            return True

        stamp = self.get_clock().now().to_msg()
        poses = []
        for x, y, z, yaw in waypoint_specs:
            pose = PoseStamped()
            pose.header.frame_id = self.nav_frame
            pose.header.stamp = stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            orientation_z, orientation_w = quaternion_z_w(yaw)
            pose.pose.orientation.z = orientation_z
            pose.pose.orientation.w = orientation_w
            poses.append(pose)

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        outcome = self._run_action(
            self.nav2_client,
            goal,
            self.move_timeout_s,
            f"NavigateThroughPoses({route_name})",
            feedback_callback=self._nav2_feedback_callback,
        )
        if outcome is None:
            return False
        return outcome[0] == GoalStatus.STATUS_SUCCEEDED

    def _nav2_feedback_callback(self, feedback_message) -> None:
        now = time.monotonic()
        last_log = getattr(self, "_last_nav_feedback_log_time", 0.0)
        if now - last_log < 1.0:
            return
        self._last_nav_feedback_log_time = now
        try:
            feedback = feedback_message.feedback
            self.get_logger().info(
                "[NAV2] "
                f"remaining={feedback.distance_remaining:.2f}m, "
                f"poses={feedback.number_of_poses_remaining}"
            )
        except Exception:
            return

    def _dock_dolly(self) -> bool:
        goal = DockDolly.Goal()
        goal.amr_id = self.amr_id
        goal.task_id = self.current_task_id
        outcome = self._run_action(
            self.dock_client,
            goal,
            self.dock_timeout_s,
            "DockDolly",
            feedback_callback=self._dock_feedback_callback,
            accepted_callback=lambda: self._publish_status("DOCKING"),
        )
        if outcome is None:
            return False
        status, result = outcome
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        return succeeded and result is not None and result.success

    def _dock_feedback_callback(self, feedback_message) -> None:
        now = time.monotonic()
        last_log = getattr(self, "_last_dock_feedback_log_time", 0.0)
        if now - last_log < 0.75:
            return
        self._last_dock_feedback_log_time = now
        try:
            feedback = feedback_message.feedback
            self.get_logger().info(
                "[DOCKING] "
                f"state={feedback.state}, "
                f"distance={feedback.distance_m:.3f}m, "
                f"lateral={feedback.lateral_m:+.3f}m, "
                f"yaw={feedback.yaw_deg:+.2f}deg"
            )
        except Exception:
            return

    def _lift_dolly(self, command: int) -> bool:
        goal = LiftDolly.Goal()
        goal.amr_id = self.amr_id
        goal.task_id = self.current_task_id
        goal.command = int(command)
        if command == LiftDolly.Goal.LIFT_UP:
            label = "LIFT_UP"
        else:
            label = "LIFT_DOWN"
        outcome = self._run_action(
            self.lift_client,
            goal,
            self.lift_timeout_s,
            f"LiftDolly({label})",
            feedback_callback=self._lift_feedback_callback,
        )
        if outcome is None:
            return False
        status, result = outcome
        succeeded = status == GoalStatus.STATUS_SUCCEEDED
        if succeeded and result is not None and result.success:
            self.get_logger().info(
                f"[LIFT] {result.message}; final={result.final_position:.3f}"
            )
            return True
        return False

    def _lift_feedback_callback(self, feedback_message) -> None:
        now = time.monotonic()
        last_log = getattr(self, "_last_lift_feedback_log_time", 0.0)
        if now - last_log < 0.75:
            return
        self._last_lift_feedback_log_time = now
        try:
            feedback = feedback_message.feedback
            self.get_logger().info(
                "[LIFT] "
                f"state={feedback.state}, "
                f"position={feedback.current_position:.3f}, "
                f"target={feedback.target_position:.3f}, "
                f"error={feedback.error:.3f}"
            )
        except Exception:
            return

    def _run_action(
        self,
        client,
        goal,
        result_timeout_s: float,
        label: str,
        feedback_callback=None,
        accepted_callback=None,
    ):
        if not client.wait_for_server(
            timeout_sec=self.action_server_wait_timeout_s
        ):
            self.get_logger().error(f"{label} Action Server is unavailable.")
            return None

        goal_response_event = threading.Event()
        result_event = threading.Event()
        goal_handle_box = {"handle": None}
        result_box = {"status": None, "result": None, "error": None}

        try:
            future = client.send_goal_async(
                goal,
                feedback_callback=feedback_callback,
            )
        except Exception as error:
            self.get_logger().error(f"{label} Goal send failed: {error}")
            return None

        def goal_response_callback(done_future) -> None:
            try:
                goal_handle = done_future.result()
                goal_handle_box["handle"] = goal_handle
                if goal_handle is None or not goal_handle.accepted:
                    self.get_logger().error(f"{label} Goal was rejected.")
                    goal_response_event.set()
                    result_event.set()
                    return
                if accepted_callback is not None:
                    accepted_callback()
                goal_response_event.set()
                result_future = goal_handle.get_result_async()

                def result_callback(completed_future) -> None:
                    try:
                        wrapped_result = completed_future.result()
                        result_box["status"] = wrapped_result.status
                        result_box["result"] = wrapped_result.result
                    except Exception as error:
                        result_box["error"] = error
                    finally:
                        result_event.set()

                result_future.add_done_callback(result_callback)
            except Exception as error:
                result_box["error"] = error
                goal_response_event.set()
                result_event.set()

        future.add_done_callback(goal_response_callback)
        if not goal_response_event.wait(
            timeout=self.action_server_wait_timeout_s
        ):
            self.get_logger().error(f"{label} Goal response timeout.")
            return None

        goal_handle = goal_handle_box["handle"]
        if goal_handle is None or not goal_handle.accepted:
            return None
        if not result_event.wait(timeout=result_timeout_s):
            self.get_logger().error(f"{label} Result timeout.")
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return None
        if result_box["error"] is not None:
            self.get_logger().error(
                f"{label} Action error: {result_box['error']}"
            )
            return None
        return result_box["status"], result_box["result"]

    def _return_to_waypoint(self) -> bool:
        start_pose = self._current_pose()
        if start_pose is None:
            self.get_logger().error("Reverse return requires odometry.")
            self._publish_stop()
            return False

        start_xy = start_pose[:2]
        reverse_command = Twist()
        reverse_command.linear.x = -self.return_speed_mps
        period = 1.0 / self.reverse_control_hz
        deadline = time.monotonic() + self.return_timeout_s

        try:
            while time.monotonic() < deadline:
                if self._shutdown_requested.is_set() or not rclpy.ok():
                    return False
                current_pose = self._current_pose()
                if current_pose is None:
                    return False
                distance = traveled_distance(start_xy, current_pose[:2])
                if distance >= self.return_distance_m:
                    self.get_logger().info(
                        f"[RETURN] odom distance={distance:.3f}m"
                    )
                    return True
                self.cmd_vel_publisher.publish(reverse_command)
                time.sleep(period)

            self.get_logger().error(
                f"[RETURN] Timeout before {self.return_distance_m:.3f}m"
            )
            return False
        finally:
            self._publish_stop()

    def _publish_stop(self) -> None:
        self.cmd_vel_publisher.publish(Twist())

    def _handle_task_failure(self, message: str) -> None:
        self._publish_stop()
        with self.task_lock:
            self.state = "ERROR"
            self.task_running = False
        self._publish_status("TASK_FAILED")
        self.get_logger().error(f"[TASK FAILED] {message}")

    def _finish_task(self) -> None:
        with self.task_lock:
            self.current_task_id = ""
            self.current_kit_id = ""
            self.state = "IDLE"
        self._publish_status("IDLE")
        with self.task_lock:
            self.task_running = False

    def destroy_node(self) -> bool:
        """Stop direct motion before releasing ROS entities."""
        self._shutdown_requested.set()
        try:
            self._publish_stop()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    """Run AMRNode in a multithreaded executor."""
    rclpy.init(args=args)
    node = AMRNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
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
