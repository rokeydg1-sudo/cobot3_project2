#!/usr/bin/env python3
"""Normalize Isaac LaserScan angle_max to the published ranges count."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class MappingScanNormalizer(Node):
    """Publish a mapping-only scan with internally consistent metadata."""

    def __init__(self) -> None:
        super().__init__("mapping_scan_normalizer")
        self.publisher = self.create_publisher(
            LaserScan,
            "/amr1/scan_mapping",
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            LaserScan,
            "/amr1/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )

    def _scan_callback(self, message: LaserScan) -> None:
        if message.ranges:
            message.angle_max = (
                message.angle_min
                + message.angle_increment * (len(message.ranges) - 1)
            )
            message.ranges = [
                value
                if math.isfinite(value) and value >= message.range_min
                else math.inf
                for value in message.ranges
            ]
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = MappingScanNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
