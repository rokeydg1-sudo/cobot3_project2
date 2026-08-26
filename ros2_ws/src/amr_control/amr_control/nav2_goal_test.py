#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose


class Nav2GoalTest(Node):

    def __init__(self):
        super().__init__('nav2_goal_test')

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.points = [
            {
                'name': 'POINT_A',
                'x': -31.007,
                'y': 15.229,
                'yaw_deg': 0.169,
            },
            {
                'name': 'POINT_B',
                'x': -2.290,
                'y': 20.956,
                'yaw_deg': 20.779,
            },
]

        self.current_index = 0

    def yaw_to_quaternion(self, yaw_deg):
        yaw = math.radians(yaw_deg)

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        return qz, qw

    def start_mission(self):
        self.get_logger().info(
            'Nav2 NavigateToPose 서버를 기다리는 중...'
        )

        self._action_client.wait_for_server()

        self.get_logger().info(
            'Nav2 서버 연결 완료!'
        )

        self.send_current_goal()

    def send_current_goal(self):
        if self.current_index >= len(self.points):
            self.get_logger().info(
                '모든 목표 지점 이동 완료! Mission Complete'
            )
            rclpy.shutdown()
            return

        point = self.points[self.current_index]

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = point['x']
        goal_msg.pose.pose.position.y = point['y']
        goal_msg.pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(point['yaw_deg'])

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"{point['name']} Goal 전송 "
            f"x={point['x']:.3f}, "
            f"y={point['y']:.3f}, "
            f"yaw={point['yaw_deg']:.3f}°"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Nav2가 Goal을 거절했습니다.'
            )
            rclpy.shutdown()
            return

        point = self.points[self.current_index]

        self.get_logger().info(
            f"{point['name']} Goal ACCEPTED"
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'남은 거리: {feedback.distance_remaining:.2f} m'
        )

    def result_callback(self, future):
        wrapped_result = future.result()
        status = wrapped_result.status

        point = self.points[self.current_index]

        if status == 4:
            self.get_logger().info(
                f"{point['name']} 도착 성공!"
            )

            self.current_index += 1
            self.send_current_goal()

        elif status == 5:
            self.get_logger().warning(
                f"{point['name']} Goal 취소"
            )
            rclpy.shutdown()

        elif status == 6:
            self.get_logger().error(
                f"{point['name']} 이동 실패 / Abort"
            )
            rclpy.shutdown()

        else:
            self.get_logger().warning(
                f"{point['name']} 종료 status={status}"
            )
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = Nav2GoalTest()
    node.start_mission()

    rclpy.spin(node)

    node.destroy_node()


if __name__ == '__main__':
    main()