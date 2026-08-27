#!/usr/bin/env python3
"""Capture one ROS Image message to a PNG for runtime diagnostics."""

import argparse
import threading

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageCapture(Node):
    """Save the first received image and stop spinning."""

    def __init__(self, topic, output):
        super().__init__("runtime_image_capture")
        self.output = output
        self.done = threading.Event()
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            topic,
            self._callback,
            10,
        )

    def _callback(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if not cv2.imwrite(self.output, image):
            raise RuntimeError(f"Failed to write image: {self.output}")
        self.get_logger().info(f"Saved image: {self.output}")
        self.done.set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = ImageCapture(args.topic, args.output)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    try:
        while not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds >= deadline:
                raise TimeoutError(f"No image received from {args.topic}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
