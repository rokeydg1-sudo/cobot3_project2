#!/usr/bin/env python3
"""Report detection quality only over frames that could contain a Dolly.

Two things make a naive hit rate misleading, and both are corrected here:

* Most frames in a mission point at empty aisle. Scoring those punishes the
  detector for something it was never asked to do, so frames are kept only when
  a Dolly was inside the lens cone at that instant.
* Isaac sometimes emits a blank or barely-converged frame. The model happily
  scored one all-black frame as a Dolly at 0.25 confidence, so counting those
  either way is worse than useless. They are reported separately.

    python3 scripts/eval_detection.py <capture_dir> [weights]
"""
import glob
import json
import math
import os
import sys
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

REPO = Path(__file__).resolve().parents[1]
VISION = REPO / "simulation" / "isaac_sim" / "vision_docking"
DEFAULT_WEIGHTS = VISION / "outputs" / "weights" / "dolly_pose_v1_best.pt"

# Every Dolly in the factory, from factory_inventory.json. The AMR only ever
# targets one at a time, but any of them in shot is a fair detection chance.
DOLLIES = [
    (-24.832190, 22.560954),
    (-27.835348, 25.521380),
    (-24.348740, -20.839027),
    (-27.364198, -23.874470),
    (-5.974600, 23.477560),
    (-8.976570, 20.473740),
    (-8.938910, -22.604786),
    (-5.905780, -25.538490),
]

# Camera sits this far ahead of the chassis origin; see CAMERA_FORWARD_M.
CAMERA_FORWARD_M = 0.55

NEAR_M, FAR_M = 1.2, 5.0
BLANK_LEVEL = 2


def load_intrinsics(width):
    path = VISION / "config" / (
        "camera_intrinsics.npz" if width == 1280
        else f"camera_intrinsics_{width}.npz"
    )
    data = np.load(path)
    return np.asarray(data["K"], dtype=float)


def main():
    capture_dir = sys.argv[1]
    weights = sys.argv[2] if len(sys.argv) > 2 else str(DEFAULT_WEIGHTS)

    frames = sorted(glob.glob(os.path.join(capture_dir, "*.png")))
    if not frames:
        raise SystemExit(f"no frames in {capture_dir}")

    height, width = cv2.imread(frames[0]).shape[:2]
    K = load_intrinsics(width)
    half_fov = math.degrees(math.atan(width / 2 / K[0, 0]))
    print(f"{len(frames)} frames at {width}x{height}, half-FOV {half_fov:.1f} deg")

    model = YOLO(weights)
    in_view, blank, hits = [], 0, 0
    rows = []

    for path in frames:
        meta_path = Path(path).with_suffix(".json")
        if not meta_path.exists():
            continue
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        yaw = meta["yaw_rad"]
        # Measure from the lens, not the chassis origin.
        cx = meta["x"] + CAMERA_FORWARD_M * math.cos(yaw)
        cy = meta["y"] + CAMERA_FORWARD_M * math.sin(yaw)

        best = None
        for dx, dy in DOLLIES:
            distance = math.hypot(dx - cx, dy - cy)
            if not (NEAR_M <= distance <= FAR_M):
                continue
            delta = math.atan2(dy - cy, dx - cx) - yaw
            bearing = math.degrees(math.atan2(math.sin(delta), math.cos(delta)))
            if abs(bearing) <= half_fov and (best is None or distance < best[0]):
                best = (distance, bearing)
        if best is None:
            continue

        if meta["max_level"] <= BLANK_LEVEL:
            blank += 1
            continue

        in_view.append(path)
        result = model.predict(source=path, imgsz=640, conf=0.10, verbose=False)[0]
        boxes = result.boxes
        distance, bearing = best
        if boxes is None or len(boxes) == 0:
            rows.append((path, distance, bearing, meta["sharpness"], None, False))
            continue
        hits += 1
        index = int(boxes.conf.argmax())
        x1, y1, x2, y2 = boxes.xyxy[index].tolist()
        clipped = x1 <= 1 or y1 <= 1 or x2 >= width - 1 or y2 >= height - 1
        rows.append(
            (path, distance, bearing, meta["sharpness"],
             float(boxes.conf[index]), clipped)
        )

    for path, distance, bearing, sharpness, conf, clipped in rows:
        verdict = "MISS" if conf is None else (
            f"conf={conf:.3f} {'CLIPPED' if clipped else 'whole'}"
        )
        print(
            f"{os.path.basename(path):12s} d={distance:.2f} "
            f"bear={bearing:+6.1f} sharp={sharpness:6.0f}  {verdict}"
        )

    total = len(in_view)
    print(f"\nblank while in view: {blank}")
    if total:
        print(f"in-view: {total}   detected: {hits}   rate: {100 * hits / total:.0f}%")
        near = [r for r in rows if r[1] <= 3.5]
        if near:
            near_hits = sum(1 for r in near if r[4] is not None)
            print(
                f"  within 3.5 m: {len(near)} frames, {near_hits} detected "
                f"({100 * near_hits / len(near):.0f}%)"
            )


main()
