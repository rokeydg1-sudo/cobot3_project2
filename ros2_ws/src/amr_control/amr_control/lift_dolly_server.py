#!/usr/bin/env python3
"""Measured-position LiftDolly action server for the Isaac IW Hub."""

import math
import threading
import time

from geometry_msgs.msg import Twist
from interfaces.action import LiftDolly  # noqa: I201
import rclpy  # noqa: I201
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState  # noqa: I201


class LiftDollyServer(Node):
    """Bridge the LiftDolly action to Isaac JointState commands."""

    def __init__(self):
        """Create ROS endpoints and measured lift state."""
        super().__init__('lift_dolly_server')
        self.declare_parameter('action_name', '/lift_dolly')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('joint_command_topic', '/joint_commands')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_name', 'lift_joint')
        self.declare_parameter('up_target', 0.04)
        self.declare_parameter('down_target', 0.0)
        self.declare_parameter('tolerance', 0.001)
        self.declare_parameter('timeout_s', 15.0)
        self.declare_parameter('command_hz', 20.0)
        self.declare_parameter('settled_samples', 3)

        self.action_name = str(self.get_parameter('action_name').value)
        self.joint_state_topic = str(
            self.get_parameter('joint_state_topic').value
        )
        self.joint_command_topic = str(
            self.get_parameter('joint_command_topic').value
        )
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.joint_name = str(self.get_parameter('joint_name').value)
        self.up_target = float(self.get_parameter('up_target').value)
        self.down_target = float(self.get_parameter('down_target').value)
        self.tolerance = float(self.get_parameter('tolerance').value)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.command_period = 1.0 / max(
            1.0,
            float(self.get_parameter('command_hz').value),
        )
        self.settled_samples = max(
            1,
            int(self.get_parameter('settled_samples').value),
        )

        self.state_lock = threading.Lock()
        self.goal_active = False
        self.current_position = None
        self.callback_group = ReentrantCallbackGroup()
        self.command_pub = self.create_publisher(
            JointState,
            self.joint_command_topic,
            10,
        )
        self.zero_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_sub = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            LiftDolly,
            self.action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            f'LiftDolly ready | action={self.action_name} '
            f'joint={self.joint_name} limits=[{self.down_target:.3f}, '
            f'{self.up_target:.3f}]'
        )
        self.get_logger().info(
            f'joint_state={self.joint_state_topic} '
            f'joint_command={self.joint_command_topic}'
        )

    def joint_state_callback(self, message):
        """Store the measured position for the configured lift DOF."""
        try:
            index = list(message.name).index(self.joint_name)
            position = float(message.position[index])
        except (ValueError, IndexError):
            return
        if math.isfinite(position):
            with self.state_lock:
                self.current_position = position

    def goal_callback(self, goal_request):
        """Accept one valid lift command at a time."""
        if goal_request.command not in (
            LiftDolly.Goal.LIFT_UP,
            LiftDolly.Goal.LIFT_DOWN,
        ):
            self.get_logger().error(
                f'Rejecting invalid lift command={goal_request.command}'
            )
            return GoalResponse.REJECT
        with self.state_lock:
            if self.goal_active:
                self.get_logger().warning('Rejecting concurrent lift goal')
                return GoalResponse.REJECT
            self.goal_active = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        """Allow cancellation; execute_callback performs the safe hold."""
        return CancelResponse.ACCEPT

    def publish_zero(self):
        """Keep the mobile base stationary during lift operation."""
        self.zero_pub.publish(Twist())

    def publish_target(self, target):
        """Publish a named position target for Isaac's controller."""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [self.joint_name]
        message.position = [float(target)]
        self.command_pub.publish(message)

    def measured_position(self):
        """Return the most recent measured joint position."""
        with self.state_lock:
            return self.current_position

    def safe_hold(self):
        """Hold the measured lift position and stop the mobile base."""
        self.publish_zero()
        current = self.measured_position()
        if current is not None:
            self.publish_target(current)

    def execute_callback(self, goal_handle):
        """Drive until the measured joint reaches the configured target."""
        command = int(goal_handle.request.command)
        target = (
            self.up_target
            if command == LiftDolly.Goal.LIFT_UP
            else self.down_target
        )
        state = (
            'LIFTING_UP'
            if command == LiftDolly.Goal.LIFT_UP
            else 'LIFTING_DOWN'
        )
        deadline = time.monotonic() + self.timeout_s
        settled = 0
        result = LiftDolly.Result()
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.safe_hold()
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'LIFT_CANCELED'
                    current = self.measured_position()
                    result.final_position = float(current or 0.0)
                    return result

                current = self.measured_position()
                self.publish_zero()
                self.publish_target(target)
                feedback = LiftDolly.Feedback()
                feedback.state = (
                    state
                    if current is not None
                    else 'WAITING_FOR_JOINT_STATE'
                )
                feedback.current_position = float(current or 0.0)
                feedback.target_position = float(target)
                feedback.error = float(
                    target - current if current is not None else target
                )
                goal_handle.publish_feedback(feedback)

                if (
                    current is not None
                    and abs(target - current) <= self.tolerance
                ):
                    settled += 1
                    if settled >= self.settled_samples:
                        self.safe_hold()
                        goal_handle.succeed()
                        result.success = True
                        result.message = (
                            'LIFT_UP_COMPLETE'
                            if command == LiftDolly.Goal.LIFT_UP
                            else 'LIFT_DOWN_COMPLETE'
                        )
                        result.final_position = float(current)
                        return result
                else:
                    settled = 0

                if time.monotonic() >= deadline:
                    self.safe_hold()
                    goal_handle.abort()
                    result.success = False
                    result.message = 'LIFT_TIMEOUT'
                    result.final_position = float(current or 0.0)
                    return result
                time.sleep(self.command_period)
        finally:
            with self.state_lock:
                self.goal_active = False


def main(args=None):
    """Run the LiftDolly server with concurrent ROS callbacks."""
    rclpy.init(args=args)
    node = LiftDollyServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.safe_hold()
        node.action_server.destroy()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
