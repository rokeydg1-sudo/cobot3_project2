#!/usr/bin/env python3
"""Draw where the Dolly should appear, using only known geometry.

This is the sanity check that has to pass before any labelling or retraining is
worth doing. Every term is known independently of the detector: the robot pose
comes from odometry, the camera offset from the bridge constants, the Dolly pose
from the inventory, the keypoints from vision_config and the intrinsics from the
calibration file. If the projected points do not land on the Dolly in the
picture, the fault is in the camera model, not in the network - and no amount of
training will fix it.

    python3 scripts/project_dolly.py <capture_dir> <frame_stem> [out.png]
"""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]
VISION = REPO / "simulation" / "isaac_sim" / "vision_docking"
sys.path.insert(0, str(VISION / "config"))
import vision_config  # noqa: E402

CAMERA_FORWARD_M = float(sys.argv[4]) if len(sys.argv) > 4 else 0.55
CAMERA_HEIGHT_M = float(sys.argv[5]) if len(sys.argv) > 5 else 0.45
CAMERA_PITCH_DEG = float(sys.argv[6]) if len(sys.argv) > 6 else 4.0


def dolly_world_points(dolly_x, dolly_y, dolly_yaw_deg):
    """Keypoints P1..P8 lifted from Dolly-local into factory coordinates."""
    yaw = math.radians(dolly_yaw_deg)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    points = []
    for name in vision_config.KEYPOINT_NAMES:
        lx, ly, lz = vision_config.DOLLY_KEYPOINTS_LOCAL[name]
        points.append(
            (
                dolly_x + cos_yaw * lx - sin_yaw * ly,
                dolly_y + sin_yaw * lx + cos_yaw * ly,
                lz,
            )
        )
    return np.asarray(points, dtype=float)


def world_to_camera(points, cam_x, cam_y, cam_z, cam_yaw, pitch_deg):
    """Factory frame -> OpenCV camera frame (x right, y down, z forward)."""
    translated = points - np.array([cam_x, cam_y, cam_z])

    cos_yaw, sin_yaw = math.cos(-cam_yaw), math.sin(-cam_yaw)
    yawed = np.stack(
        [
            cos_yaw * translated[:, 0] - sin_yaw * translated[:, 1],
            sin_yaw * translated[:, 0] + cos_yaw * translated[:, 1],
            translated[:, 2],
        ],
        axis=1,
    )

    # Tilt about the robot's left axis, positive meaning nose-down.
    tilt = math.radians(pitch_deg)
    cos_tilt, sin_tilt = math.cos(tilt), math.sin(tilt)
    forward = cos_tilt * yawed[:, 0] - sin_tilt * yawed[:, 2]
    up = sin_tilt * yawed[:, 0] + cos_tilt * yawed[:, 2]

    # Robot axes (forward, left, up) into OpenCV axes (right, down, forward).
    return np.stack([-yawed[:, 1], -up, forward], axis=1)


def main():
    capture_dir = Path(sys.argv[1])
    stem = sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else "/tmp/projected.png"

    image = cv2.imread(str(capture_dir / f"{stem}.png"))
    height, width = image.shape[:2]
    meta = json.loads((capture_dir / f"{stem}.json").read_text(encoding="utf-8"))

    intrinsics = np.load(
        VISION / "config" / (
            "camera_intrinsics.npz" if width == 1280
            else f"camera_intrinsics_{width}.npz"
        )
    )
    K = np.asarray(intrinsics["K"], dtype=float)

    inventory = json.loads(
        (REPO / "simulation" / "isaac_sim" / "factory_inventory.json").read_text(
            encoding="utf-8"
        )
    )

    yaw = meta["yaw_rad"]
    cam_x = meta["x"] + CAMERA_FORWARD_M * math.cos(yaw)
    cam_y = meta["y"] + CAMERA_FORWARD_M * math.sin(yaw)

    canvas = image.copy()
    print(f"{stem}: robot=({meta['x']:.2f}, {meta['y']:.2f}, "
          f"{math.degrees(yaw):+.1f} deg)  camera=({cam_x:.2f}, {cam_y:.2f}, "
          f"{CAMERA_HEIGHT_M:.2f})")

    for dolly in inventory["dollies"]:
        points = dolly_world_points(dolly["x"], dolly["y"], dolly["yaw_deg"])
        camera_points = world_to_camera(
            points, cam_x, cam_y, CAMERA_HEIGHT_M, yaw, CAMERA_PITCH_DEG
        )
        if np.median(camera_points[:, 2]) <= 0.2:
            continue
        distance = float(np.median(camera_points[:, 2]))
        if distance > 8.0:
            continue

        uv = (K @ camera_points.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        inside = sum(
            1 for u, v in uv if 0 <= u < width and 0 <= v < height
        )
        print(
            f"  {dolly['path']:28s} d={distance:5.2f}  "
            f"u=[{uv[:, 0].min():7.0f},{uv[:, 0].max():7.0f}]  "
            f"v=[{uv[:, 1].min():7.0f},{uv[:, 1].max():7.0f}]  in-frame={inside}/8"
        )
        for index, (u, v) in enumerate(uv):
            if -2000 < u < 2000 and -2000 < v < 2000:
                cv2.circle(canvas, (int(u), int(v)), 5, (0, 165, 255), -1)
                cv2.putText(
                    canvas, str(index + 1), (int(u) + 6, int(v) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1
                )

    cv2.imwrite(out, canvas)
    print(f"wrote {out}")


main()
