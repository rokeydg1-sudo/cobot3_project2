#!/usr/bin/env python3
"""Run only the production AMR odometry-based reverse stage."""

import argparse
import math
import threading
import time

from amr_control.amr_node import AMRNode

from geometry_msgs.msg import Twist

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter


class ReverseGateNode(AMRNode):
    """Disable task acquisition and observe the production reverse command."""

    def __init__(self, parameter_overrides) -> None:
        """Create a node with automatic task acquisition disabled."""
        super().__init__(parameter_overrides=parameter_overrides)
        self.task_request_timer.cancel()
        with self.task_lock:
            self.state = 'BUSY'
            self.task_running = True
        self.last_command = None
        self.command_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._command_callback,
            10,
        )

    def _command_callback(self, message: Twist) -> None:
        self.last_command = (
            float(message.linear.x),
            float(message.angular.z),
        )


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--odom-topic', default='/amr1/odom')
    parser.add_argument('--cmd-vel-topic', default='/amr1/cmd_vel')
    parser.add_argument('--distance', type=float, default=3.0)
    parser.add_argument('--speed', type=float, default=0.20)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--control-hz', type=float, default=20.0)
    parser.add_argument('--odom-wait', type=float, default=10.0)
    return parser.parse_args()


def _wait_for_pose(node, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pose = node._current_pose()
        if pose is not None:
            return pose
        time.sleep(0.05)
    return None


def main() -> int:
    """Execute one reverse gate and return a shell status code."""
    args = _arguments()
    overrides = [
        Parameter('odom_topic', value=args.odom_topic),
        Parameter('cmd_vel_topic', value=args.cmd_vel_topic),
        Parameter('return_distance_m', value=args.distance),
        Parameter('return_speed_mps', value=args.speed),
        Parameter('return_timeout_s', value=args.timeout),
        Parameter('reverse_control_hz', value=args.control_hz),
    ]
    rclpy.init()
    node = ReverseGateNode(overrides)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    result = False
    try:
        start_pose = _wait_for_pose(node, args.odom_wait)
        if start_pose is None:
            print('FAIL: odometry was not received')
            return 2

        print(
            'START: '
            f'x={start_pose[0]:.6f} y={start_pose[1]:.6f} '
            f'yaw={start_pose[2]:.6f}'
        )
        reverse_start_time = time.monotonic()
        result = node._return_to_waypoint()
        elapsed_s = time.monotonic() - reverse_start_time
        time.sleep(0.25)
        end_pose = node._current_pose()
        if end_pose is None:
            print('FAIL: odometry disappeared after reverse')
            return 3

        dx = end_pose[0] - start_pose[0]
        dy = end_pose[1] - start_pose[1]
        distance = math.hypot(dx, dy)
        yaw_delta = math.atan2(
            math.sin(end_pose[2] - start_pose[2]),
            math.cos(end_pose[2] - start_pose[2]),
        )
        stopped = node.last_command == (0.0, 0.0)
        print(
            'END: '
            f'x={end_pose[0]:.6f} y={end_pose[1]:.6f} '
            f'yaw={end_pose[2]:.6f}'
        )
        print(f'ODOM_DISTANCE={distance:.6f}')
        print(f'ELAPSED_S={elapsed_s:.6f}')
        print(f'AVERAGE_SPEED_MPS={distance / max(elapsed_s, 1e-6):.6f}')
        print(f'DX={dx:.6f} DY={dy:.6f} YAW_DELTA={yaw_delta:.6f}')
        print(f'LAST_CMD_ZERO={stopped}')
        if result and stopped:
            print('PASS: production odom reverse and zero Twist')
            return 0
        print('FAIL: production odom reverse or zero Twist')
        return 1
    finally:
        node._publish_stop()
        node._shutdown_requested.set()
        time.sleep(0.1)
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    raise SystemExit(main())
