#!/usr/bin/env python3
"""
Fast YOLO Pose inference smoke test for the Dolly model.

- Uses the runtime best.pt
- Tests 20 evenly spaced validation images
- Saves prediction overlays
- Compares predicted keypoints against GT visible keypoints
- Prints one compact PASS/WARN summary

Run in the vision .venv from repository root:
    python vision/isaac_sim/vision_docking/inference/validate_yolo_pose.py
"""

from pathlib import Path
import csv
import math
import shutil

import cv2
import numpy as np
from ultralytics import YOLO


REPO_ROOT = Path(__file__).resolve().parents[4]
VISION_DIR = REPO_ROOT / "vision" / "isaac_sim" / "vision_docking"

MODEL_PATH = (
    VISION_DIR
    / "outputs"
    / "weights"
    / "dolly_pose_v1_best.pt"
)

IMAGE_DIR = (
    VISION_DIR
    / "outputs"
    / "dataset"
    / "images"
    / "val"
)

LABEL_DIR = (
    VISION_DIR
    / "outputs"
    / "dataset"
    / "labels"
    / "val"
)

OUTPUT_DIR = (
    VISION_DIR
    / "outputs"
    / "inference_smoke"
)

SUMMARY_CSV = OUTPUT_DIR / "summary.csv"

NUM_SAMPLES = 20
IMG_SIZE = 640
CONF = 0.25
DEVICE = 0

# Compact smoke thresholds; not final acceptance criteria.
PASS_DETECTION_RATE = 0.95
WARN_MEAN_KP_ERROR_PX = 30.0


def choose_evenly_spaced(paths, n):
    if len(paths) <= n:
        return paths

    indices = np.linspace(
        0,
        len(paths) - 1,
        n,
        dtype=int,
    )

    return [paths[i] for i in indices]


def read_gt_keypoints(label_path, width, height):
    values = [
        float(v)
        for v in label_path.read_text(
            encoding="utf-8"
        ).strip().split()
    ]

    # class + bbox(4) + 8 * (x,y,v)
    kp = values[5:]

    points = []

    for i in range(8):
        x = kp[i * 3 + 0]
        y = kp[i * 3 + 1]
        visibility = int(
            kp[i * 3 + 2]
        )

        points.append(
            (
                x * width,
                y * height,
                visibility,
            )
        )

    return points


def main():

    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"Model not found: {MODEL_PATH}"
        )

    images = sorted(
        IMAGE_DIR.glob("*.png")
    )

    if not images:
        raise RuntimeError(
            f"No validation images: {IMAGE_DIR}"
        )

    selected = choose_evenly_spaced(
        images,
        NUM_SAMPLES,
    )

    if OUTPUT_DIR.exists():
        shutil.rmtree(
            OUTPUT_DIR
        )

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    model = YOLO(
        str(MODEL_PATH)
    )

    rows = []

    detected_count = 0
    bbox_confidences = []
    all_kp_errors = []

    for index, image_path in enumerate(
        selected
    ):

        image = cv2.imread(
            str(image_path)
        )

        if image is None:
            raise RuntimeError(
                f"Failed to read: {image_path}"
            )

        height, width = (
            image.shape[:2]
        )

        result = model.predict(
            source=image,
            imgsz=IMG_SIZE,
            conf=CONF,
            device=DEVICE,
            verbose=False,
        )[0]

        detected = (
            result.boxes is not None
            and
            len(result.boxes) > 0
        )

        bbox_conf = None
        kp_mean_error = None
        visible_gt_count = 0

        if detected:

            detected_count += 1

            confidences = (
                result.boxes.conf
                .detach()
                .cpu()
                .numpy()
            )

            best_index = int(
                np.argmax(confidences)
            )

            bbox_conf = float(
                confidences[best_index]
            )

            bbox_confidences.append(
                bbox_conf
            )

            if (
                result.keypoints is not None
                and
                result.keypoints.xy is not None
            ):

                pred_kp = (
                    result.keypoints.xy[
                        best_index
                    ]
                    .detach()
                    .cpu()
                    .numpy()
                )

                label_path = (
                    LABEL_DIR
                    / f"{image_path.stem}.txt"
                )

                gt_points = (
                    read_gt_keypoints(
                        label_path,
                        width,
                        height,
                    )
                )

                errors = []

                for kp_index, (
                    gt_x,
                    gt_y,
                    visibility,
                ) in enumerate(
                    gt_points
                ):

                    # Compare only fully visible GT keypoints.
                    if visibility != 2:
                        continue

                    visible_gt_count += 1

                    if kp_index >= len(
                        pred_kp
                    ):
                        continue

                    pred_x = float(
                        pred_kp[
                            kp_index,
                            0,
                        ]
                    )

                    pred_y = float(
                        pred_kp[
                            kp_index,
                            1,
                        ]
                    )

                    error = math.hypot(
                        pred_x - gt_x,
                        pred_y - gt_y,
                    )

                    errors.append(
                        error
                    )

                    all_kp_errors.append(
                        error
                    )

                if errors:
                    kp_mean_error = float(
                        np.mean(errors)
                    )

        overlay = result.plot()

        output_path = (
            OUTPUT_DIR
            / f"{index:02d}_{image_path.name}"
        )

        cv2.imwrite(
            str(output_path),
            overlay,
        )

        rows.append(
            {
                "image": image_path.name,
                "detected": int(detected),
                "bbox_conf": (
                    ""
                    if bbox_conf is None
                    else f"{bbox_conf:.6f}"
                ),
                "visible_gt_keypoints": (
                    visible_gt_count
                ),
                "mean_visible_kp_error_px": (
                    ""
                    if kp_mean_error is None
                    else f"{kp_mean_error:.3f}"
                ),
            }
        )

    with SUMMARY_CSV.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=rows[0].keys(),
        )

        writer.writeheader()
        writer.writerows(
            rows
        )

    detection_rate = (
        detected_count
        / len(selected)
    )

    mean_bbox_conf = (
        float(
            np.mean(
                bbox_confidences
            )
        )
        if bbox_confidences
        else float("nan")
    )

    mean_kp_error = (
        float(
            np.mean(
                all_kp_errors
            )
        )
        if all_kp_errors
        else float("nan")
    )

    median_kp_error = (
        float(
            np.median(
                all_kp_errors
            )
        )
        if all_kp_errors
        else float("nan")
    )

    print(
        "===== YOLO POSE SMOKE TEST ====="
    )

    print(
        f"Model             : {MODEL_PATH}"
    )

    print(
        f"Samples           : {len(selected)}"
    )

    print(
        f"Detection         : "
        f"{detected_count}/{len(selected)} "
        f"({100.0 * detection_rate:.1f}%)"
    )

    print(
        f"Mean bbox conf     : "
        f"{mean_bbox_conf:.3f}"
    )

    print(
        f"Mean visible KP err: "
        f"{mean_kp_error:.2f} px"
    )

    print(
        f"Median KP err      : "
        f"{median_kp_error:.2f} px"
    )

    print(
        f"Overlays           : {OUTPUT_DIR}"
    )

    print(
        f"Summary CSV        : {SUMMARY_CSV}"
    )

    if detection_rate < PASS_DETECTION_RATE:
        print(
            "RESULT             : FAIL "
            "(detection rate)"
        )
        return 1

    if (
        np.isfinite(
            mean_kp_error
        )
        and
        mean_kp_error
        > WARN_MEAN_KP_ERROR_PX
    ):
        print(
            "RESULT             : PASS WITH WARN "
            "(keypoint pixel error)"
        )
        return 0

    print(
        "RESULT             : PASS"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
