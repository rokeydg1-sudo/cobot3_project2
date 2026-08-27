#!/usr/bin/env python3
"""Record camera frames with the robot pose sampled at the same instant.

Pairing frames against the mission controller's log does not work: the
controller prints once a second while the AMR travels at up to 1.5 m/s, so a
frame can be matched to a pose that is over a metre stale. That was enough to
report a Dolly at 3.0 m when the wheels in the picture were clearly 0.8 m away,
which in turn made the detection statistics meaningless.

Subscribing to odometry here and writing a sidecar JSON per frame removes the
guesswork: both come from the same moment.

Odometry is reported relative to where the robot spawned, not in factory
coordinates, so the spawn pose from the inventory is applied here exactly as the
mission controller does it. Skipping that step put the AMR fourteen metres from
the nearest Dolly for an entire run.

    python3 scripts/capture_frames.py <image_topic> <odom_topic> <out_dir> <hz> [amr]
"""
import json
import math
import sys
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def spawn_pose(amr_name):
    """Where the robot's odometry origin sits in factory coordinates."""
    inventory = json.loads(
        (
            Path(__file__).resolve().parents[1]
            / "simulation"
            / "isaac_sim"
            / "factory_inventory.json"
        ).read_text(encoding="utf-8")
    )
    for robot in inventory["amrs"]:
        if robot["name"] == amr_name:
            start = robot["amr_start"]
            return (
                float(start["x"]),
                float(start["y"]),
                math.radians(float(start["yaw_deg"])),
            )
    raise SystemExit(f"{amr_name} not in inventory")


class Capture(Node):
    def __init__(self, image_topic, odom_topic, out_dir, hz, amr_name):
        super().__init__("capture_frames")
        self.start_x, self.start_y, self.start_yaw = spawn_pose(amr_name)
        self.bridge = CvBridge()
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.count = 0
        self.image = None
        self.odom = None
        self.create_subscription(Image, image_topic, self.on_image, 1)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 1)
        self.create_timer(1.0 / hz, self.tick)

    def on_image(self, msg):
        self.image = msg

    def on_odom(self, msg):
        self.odom = msg

    def tick(self):
        if self.image is None or self.odom is None:
            return
        image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="bgr8")
        stem = f"f_{self.count:04d}"
        cv2.imwrite(str(self.out_dir / f"{stem}.png"), image)

        pose = self.odom.pose.pose
        ox, oy = pose.position.x, pose.position.y
        cos_start, sin_start = math.cos(self.start_yaw), math.sin(self.start_yaw)
        meta = {
            "x": self.start_x + cos_start * ox - sin_start * oy,
            "y": self.start_y + sin_start * ox + cos_start * oy,
            "yaw_rad": self.start_yaw + yaw_from_quaternion(pose.orientation),
            "max_level": int(image.max()),
            "sharpness": float(
                cv2.Laplacian(
                    cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.CV_64F
                ).var()
            ),
        }
        (self.out_dir / f"{stem}.json").write_text(json.dumps(meta), encoding="utf-8")
        print(
            f"{stem} pose=({meta['x']:.3f}, {meta['y']:.3f}, "
            f"{math.degrees(meta['yaw_rad']):+.1f} deg) "
            f"max={meta['max_level']} sharp={meta['sharpness']:.0f}",
            flush=True,
        )
        self.count += 1


def main():
    image_topic, odom_topic, out_dir, hz = (
        sys.argv[1],
        sys.argv[2],
        sys.argv[3],
        float(sys.argv[4]),
    )
    amr_name = sys.argv[5] if len(sys.argv) > 5 else "amr1"
    rclpy.init()
    node = Capture(image_topic, odom_topic, out_dir, hz, amr_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


main()
