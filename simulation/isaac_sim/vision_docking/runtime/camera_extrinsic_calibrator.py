#!/usr/bin/env python3
"""Score real Isaac camera frames while the bridge cycles extrinsic candidates."""

import importlib.util
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "vision_config.py"
MODEL = ROOT / "outputs" / "weights" / "dolly_pose_v1_best.pt"
INTRINSICS = ROOT / "config" / "camera_intrinsics.npz"
CANDIDATE_FILE = Path("/tmp/docking_camera_candidate")


def load_config():
    spec = importlib.util.spec_from_file_location("vision_config", CONFIG)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Calibrator(Node):
    def __init__(self):
        super().__init__("camera_extrinsic_calibrator")
        self.bridge = CvBridge()
        self.model = YOLO(str(MODEL))
        intrinsics = np.load(INTRINSICS)
        self.K = intrinsics["K"].astype(np.float64)
        self.dist = intrinsics["distortion"].astype(np.float64)
        self.config = load_config()
        self.object_points = np.asarray(
            [self.config.DOLLY_KEYPOINTS_LOCAL[n] for n in self.config.KEYPOINT_NAMES],
            dtype=np.float64,
        )
        self.frames = []
        self.sub = self.create_subscription(
            Image, "/vision/front_camera/image_raw", self.image_callback, 10
        )

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        result = self.model.predict(
            image, imgsz=640, conf=0.25, device="cpu", verbose=False
        )[0]
        best_conf = 0.0
        keypoints = 0
        pnp_ok = False
        if len(result.boxes) and result.keypoints is not None:
            index = int(result.boxes.conf.argmax().item())
            best_conf = float(result.boxes.conf[index].item())
            xy = result.keypoints.xy[index].cpu().numpy()
            kp_conf = result.keypoints.conf[index].cpu().numpy()
            keep = np.where(kp_conf >= 0.25)[0]
            keypoints = int(len(keep))
            if len(keep) >= 4 and len(xy) == len(self.object_points):
                pnp_ok, _, _, _ = cv2.solvePnPRansac(
                    self.object_points[keep], xy[keep].astype(np.float64),
                    self.K, self.dist, iterationsCount=100,
                    reprojectionError=12.0, confidence=0.99,
                    flags=cv2.SOLVEPNP_EPNP,
                )
        self.frames.append(
            (float(image.mean()), float(image.std()), best_conf, keypoints, pnp_ok)
        )

    def evaluate(self, candidate):
        CANDIDATE_FILE.write_text(f"{candidate}\n", encoding="ascii")
        self.frames.clear()
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.frames:
            return (0.0, 0, 0.0, False, 0.0)
        stable = self.frames[-3:] if len(self.frames) >= 3 else self.frames
        brightness = float(np.mean([f[0] for f in stable]))
        confidence = float(np.mean([f[2] for f in stable]))
        keypoints = int(max(f[3] for f in stable))
        pnp = any(f[4] for f in stable)
        black_ratio = float(np.mean([f[1] < 2.0 for f in stable]))
        return brightness, keypoints, confidence, pnp, black_ratio


def main():
    rclpy.init()
    node = Calibrator()
    results = []
    try:
        for candidate in range(6):
            metrics = node.evaluate(candidate)
            brightness, keypoints, confidence, pnp, black_ratio = metrics
            score = (
                (1000.0 if pnp else 0.0)
                + 100.0 * min(keypoints, 8)
                + 100.0 * confidence
                + min(brightness, 255.0)
                - 500.0 * black_ratio
            )
            results.append((score, candidate, metrics))
            print(
                f"CALIBRATION candidate={candidate} score={score:.2f} "
                f"brightness={brightness:.1f} kp={keypoints} "
                f"conf={confidence:.3f} pnp={pnp} black={black_ratio:.2f}",
                flush=True,
            )
        best = max(results, key=lambda item: item[0])
        CANDIDATE_FILE.write_text(f"{best[1]}\n", encoding="ascii")
        print(f"CALIBRATION SELECTED candidate={best[1]} score={best[0]:.2f}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
