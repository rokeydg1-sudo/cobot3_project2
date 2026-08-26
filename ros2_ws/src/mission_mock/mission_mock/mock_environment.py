"""ROS fake servers and odometry used by the GPU-free mission runner."""

import math
import threading
import time

from geometry_msgs.msg import Twist
from interfaces.action import DockDolly, LiftDolly, VisualizeRoute
from interfaces.srv import RequestTask
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from amr_control.mission_utils import quaternion_yaw, quaternion_z_w


VALID_SCENARIOS = (
    'success',
    'dock_failure',
    'lift_up_failure',
    'reverse_timeout',
)


class MockMissionEnvironment(Node):
    """Provide deterministic FMS, Action, odom, and kinematic fakes."""

    def __init__(self, scenario='success', hold_first_response=False) -> None:
        """Create all fake ROS endpoints for one test scenario."""
        if scenario not in VALID_SCENARIOS:
            raise ValueError(f'Unknown mock scenario: {scenario}')
        super().__init__('mission_mock_environment')
        self.scenario = scenario
        self.hold_first_response = bool(hold_first_response)
        self.callback_group = ReentrantCallbackGroup()
        self.data_lock = threading.Lock()
        self.completion_event = threading.Event()
        self.first_request_received_event = threading.Event()
        self.release_first_response_event = threading.Event()

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.last_update_time = time.monotonic()
        self.linear_command = 0.0
        self.simulate_reverse_motion = scenario != 'reverse_timeout'
        self.docking_advance_m = 0.30

        self.task_request_count = 0
        self.task_assignment_count = 0
        self.event_sequence = 0
        self.request_history = []
        self.visualized_routes = []
        self.nav_goals = []
        self.dock_call_count = 0
        self.lift_commands = []
        self.negative_cmd_seen = False
        self.zero_after_negative_seen = False
        self.status_sequence = []
        self.status_history = []
        self.last_amr_state = ''
        self.mission_complete_seen = False
        self.idle_after_mission_seen = False

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/amr/odom',
            10,
        )
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10,
            callback_group=self.callback_group,
        )
        self.status_subscription = self.create_subscription(
            String,
            '/amr/status',
            self._status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.task_service = self.create_service(
            RequestTask,
            '/fms/request_task',
            self._request_task_callback,
            callback_group=self.callback_group,
        )

        self.visualize_server = ActionServer(
            self,
            VisualizeRoute,
            '/visualize_route',
            execute_callback=self._execute_visualize,
            callback_group=self.callback_group,
        )
        self.nav_server = ActionServer(
            self,
            NavigateThroughPoses,
            '/navigate_through_poses',
            execute_callback=self._execute_navigation,
            callback_group=self.callback_group,
        )
        self.dock_server = ActionServer(
            self,
            DockDolly,
            '/dock_dolly',
            execute_callback=self._execute_dock,
            callback_group=self.callback_group,
        )
        self.lift_server = ActionServer(
            self,
            LiftDolly,
            '/lift_dolly',
            execute_callback=self._execute_lift,
            callback_group=self.callback_group,
        )
        self.kinematic_timer = self.create_timer(
            0.02,
            self._kinematic_step,
            callback_group=self.callback_group,
        )

    def _request_task_callback(self, request, response):
        with self.data_lock:
            self.event_sequence += 1
            sequence = self.task_request_count + 1
            request_record = {
                'sequence': sequence,
                'state': str(request.state),
                'current_task_id': str(request.current_task_id),
                'load_state': str(request.load_state),
                'x': float(request.x),
                'y': float(request.y),
                'timestamp': time.monotonic(),
                'event_order': self.event_sequence,
                'mission_complete_observed': self.mission_complete_seen,
                'idle_observed': self.idle_after_mission_seen,
            }
            self.request_history.append(request_record)
            self.task_request_count = sequence
            assign_task = self.task_assignment_count == 0
            if assign_task:
                self.task_assignment_count += 1

        if sequence == 1:
            self.first_request_received_event.set()
            if self.hold_first_response:
                self.release_first_response_event.wait(timeout=2.0)

        if not assign_task:
            response.has_task = False
            response.message = 'No waiting task after completed mock mission.'
            return response

        response.has_task = True
        response.task_id = 'mock_task_001'
        response.kit_id = 'MOCK_DOLLY'
        response.processing_time = 0.0
        response.pickup_id = 'C'
        response.pickup_x = 1.0
        response.pickup_y = 1.0
        response.delivery_id = 'E'
        response.delivery_x = 2.0
        response.delivery_y = 2.0
        response.node_map_revision = 1

        response.approach_route_node_ids = [10, 11, 12]
        response.approach_route_x = [0.0, 1.0, 1.0]
        response.approach_route_y = [0.0, 0.0, 1.0]
        response.approach_route_z = [0.0, 0.0, 0.0]
        response.approach_route_total_cost = 2.0

        response.route_node_ids = [12, 13, 14]
        response.route_x = [1.0, 2.0, 2.0]
        response.route_y = [1.0, 1.0, 2.0]
        response.route_z = [0.0, 0.0, 0.0]
        response.route_total_cost = 2.0
        response.message = 'Assigned deterministic mock mission.'
        return response

    def _execute_visualize(self, goal_handle):
        with self.data_lock:
            self.visualized_routes.append(
                tuple(int(value) for value in goal_handle.request.node_ids)
            )
        time.sleep(0.02)
        result = VisualizeRoute.Result()
        result.success = True
        result.message = 'Mock route visualized.'
        goal_handle.succeed()
        return result

    def _execute_navigation(self, goal_handle):
        poses = list(goal_handle.request.poses)
        with self.data_lock:
            self.nav_goals.append(poses)
        time.sleep(0.02)

        if poses:
            final_pose = poses[-1].pose
            orientation = final_pose.orientation
            with self.data_lock:
                self.pose_x = float(final_pose.position.x)
                self.pose_y = float(final_pose.position.y)
                self.pose_yaw = quaternion_yaw(
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )
            self._publish_odom()
            time.sleep(0.04)

        result = NavigateThroughPoses.Result()
        result.error_code = NavigateThroughPoses.Result.NONE
        result.error_msg = ''
        goal_handle.succeed()
        return result

    def _execute_dock(self, goal_handle):
        with self.data_lock:
            self.dock_call_count += 1
        time.sleep(0.02)

        result = DockDolly.Result()
        if self.scenario == 'dock_failure':
            result.success = False
            result.message = 'Injected DockDolly failure.'
            goal_handle.succeed()
            return result

        with self.data_lock:
            self.pose_x += self.docking_advance_m * math.cos(self.pose_yaw)
            self.pose_y += self.docking_advance_m * math.sin(self.pose_yaw)
        self._publish_odom()
        time.sleep(0.04)
        result.success = True
        result.message = 'DOCKING_COMPLETE'
        goal_handle.succeed()
        return result

    def _execute_lift(self, goal_handle):
        command = int(goal_handle.request.command)
        with self.data_lock:
            self.lift_commands.append(command)
        time.sleep(0.02)

        result = LiftDolly.Result()
        lift_up_failure = self.scenario == 'lift_up_failure'
        if lift_up_failure and command == LiftDolly.Goal.LIFT_UP:
            result.success = False
            result.message = 'Injected Lift Up failure.'
            result.final_position = 0.0
            goal_handle.succeed()
            return result

        result.success = True
        if command == LiftDolly.Goal.LIFT_UP:
            result.message = 'LIFT_UP_COMPLETE'
            result.final_position = 1.0
        else:
            result.message = 'LIFT_DOWN_COMPLETE'
            result.final_position = 0.0
        goal_handle.succeed()
        return result

    def _cmd_vel_callback(self, message: Twist) -> None:
        linear_x = float(message.linear.x)
        with self.data_lock:
            self.linear_command = linear_x
            if linear_x < -1.0e-6:
                self.negative_cmd_seen = True
            if abs(linear_x) <= 1.0e-6 and self.negative_cmd_seen:
                self.zero_after_negative_seen = True

    def _status_callback(self, message: String) -> None:
        fields = {}
        for item in message.data.split(','):
            if '=' in item:
                key, value = item.split('=', 1)
                fields[key.strip()] = value.strip()
        status = fields.get('status', '')
        with self.data_lock:
            self.event_sequence += 1
            self.status_sequence.append(status)
            self.last_amr_state = fields.get('state', '')
            self.status_history.append(
                {
                    'status': status,
                    'state': self.last_amr_state,
                    'task_id': fields.get('task_id', ''),
                    'load_state': fields.get('load_state', ''),
                    'timestamp': time.monotonic(),
                    'event_order': self.event_sequence,
                }
            )
            if status == 'MISSION_COMPLETE':
                self.mission_complete_seen = True
            if status == 'TASK_FAILED':
                self.completion_event.set()
            if status == 'IDLE' and self.mission_complete_seen:
                self.idle_after_mission_seen = True
                self.completion_event.set()

    def _kinematic_step(self) -> None:
        now = time.monotonic()
        with self.data_lock:
            delta_time = min(now - self.last_update_time, 0.10)
            self.last_update_time = now
            if self.simulate_reverse_motion:
                self.pose_x += (
                    self.linear_command * math.cos(self.pose_yaw) * delta_time
                )
                self.pose_y += (
                    self.linear_command * math.sin(self.pose_yaw) * delta_time
                )
        self._publish_odom()

    def _publish_odom(self) -> None:
        with self.data_lock:
            x = self.pose_x
            y = self.pose_y
            yaw = self.pose_yaw
        message = Odometry()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        z, w = quaternion_z_w(yaw)
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        self.odom_publisher.publish(message)

    def snapshot(self):
        """Return a thread-safe copy of all observed test evidence."""
        with self.data_lock:
            return {
                'task_request_count': self.task_request_count,
                'task_assignment_count': self.task_assignment_count,
                'request_history': tuple(
                    dict(item) for item in self.request_history
                ),
                'visualized_routes': tuple(self.visualized_routes),
                'nav_goals': tuple(tuple(goal) for goal in self.nav_goals),
                'dock_call_count': self.dock_call_count,
                'lift_commands': tuple(self.lift_commands),
                'negative_cmd_seen': self.negative_cmd_seen,
                'zero_after_negative_seen': self.zero_after_negative_seen,
                'status_sequence': tuple(self.status_sequence),
                'status_history': tuple(
                    dict(item) for item in self.status_history
                ),
                'last_amr_state': self.last_amr_state,
                'idle_after_mission_seen': self.idle_after_mission_seen,
            }

    def destroy_node(self) -> bool:
        """Release mock ROS entities through the owning Node."""
        self.release_first_response_event.set()
        return super().destroy_node()
