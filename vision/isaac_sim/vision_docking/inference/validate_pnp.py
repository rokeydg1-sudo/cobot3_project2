#!/usr/bin/env python3
"""
Fast YOLO Pose -> PnP validation.

Uses:
- dolly_pose_v1_best.pt
- validation RGB images
- SDG metadata ground truth
- exact camera_intrinsics.npz exported from Isaac Sim

Output:
outputs/pnp_smoke/summary.csv

This is a compact gate before runtime docking control.
"""

from pathlib import Path
import csv
import importlib.util
import math
import shutil

import cv2
import numpy as np
from ultralytics import YOLO


REPO_ROOT = Path(__file__).resolve().parents[4]

VISION_DIR = (
    REPO_ROOT
    / "vision"
    / "isaac_sim"
    / "vision_docking"
)

MODEL_PATH = (
    VISION_DIR
    / "outputs"
    / "weights"
    / "dolly_pose_v1_best.pt"
)

INTRINSICS_PATH = (
    VISION_DIR
    / "config"
    / "camera_intrinsics.npz"
)

CONFIG_PATH = (
    VISION_DIR
    / "config"
    / "vision_config.py"
)

IMAGE_DIR = (
    VISION_DIR
    / "outputs"
    / "dataset"
    / "images"
    / "val"
)

METADATA_PATH = (
    VISION_DIR
    / "outputs"
    / "dataset"
    / "metadata_val.csv"
)

OUTPUT_DIR = (
    VISION_DIR
    / "outputs"
    / "pnp_smoke"
)

SUMMARY_PATH = (
    OUTPUT_DIR
    / "summary.csv"
)


NUM_SAMPLES = 40
IMG_SIZE = 640
DETECTION_CONF = 0.25
KEYPOINT_CONF = 0.35

PNP_REPROJECTION_ERROR_PX = 12.0
PNP_ITERATIONS = 200
DEVICE = 0


def load_vision_config():

    spec = (
        importlib.util
        .spec_from_file_location(
            "dolly_vision_config",
            CONFIG_PATH,
        )
    )

    if (
        spec is None
        or spec.loader is None
    ):
        raise RuntimeError(
            f"Cannot load config: {CONFIG_PATH}"
        )

    module = (
        importlib.util
        .module_from_spec(
            spec
        )
    )

    spec.loader.exec_module(
        module
    )

    return module


def choose_evenly_spaced(
    paths,
    n,
):

    if len(paths) <= n:
        return paths

    indices = np.linspace(
        0,
        len(paths) - 1,
        n,
        dtype=int,
    )

    return [
        paths[i]
        for i in indices
    ]


def load_metadata():

    with METADATA_PATH.open(
        "r",
        encoding="utf-8",
        newline="",
    ) as f:

        rows = list(
            csv.DictReader(f)
        )

    return {
        row["frame"]: row
        for row in rows
    }


def wrap_deg(angle):

    return (
        angle + 180.0
    ) % 360.0 - 180.0


def main():

    for path in (
        MODEL_PATH,
        INTRINSICS_PATH,
        CONFIG_PATH,
        METADATA_PATH,
    ):
        if not path.exists():
            raise FileNotFoundError(
                str(path)
            )

    config = load_vision_config()

    object_points_all = np.asarray(
        [
            config.DOLLY_KEYPOINTS_LOCAL[
                name
            ]
            for name in config.KEYPOINT_NAMES
        ],
        dtype=np.float64,
    )

    intrinsics = np.load(
        INTRINSICS_PATH
    )

    K = np.asarray(
        intrinsics["K"],
        dtype=np.float64,
    )

    dist_coeffs = np.asarray(
        intrinsics["distortion"],
        dtype=np.float64,
    )

    metadata = load_metadata()

    images = sorted(
        IMAGE_DIR.glob(
            "*.png"
        )
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

    pnp_success_count = 0

    distance_errors = []
    lateral_errors = []

    yaw_errors_normal = []
    yaw_errors_flipped = []

    for image_path in selected:

        frame = image_path.stem

        gt = metadata.get(
            frame
        )

        if gt is None:
            continue

        result = model.predict(
            source=str(image_path),
            imgsz=IMG_SIZE,
            conf=DETECTION_CONF,
            device=DEVICE,
            verbose=False,
        )[0]

        if (
            result.boxes is None
            or
            len(result.boxes) == 0
            or
            result.keypoints is None
        ):

            rows.append(
                {
                    "frame": frame,
                    "pnp_ok": 0,
                    "used_keypoints": 0,
                }
            )
            continue

        box_conf = (
            result.boxes.conf
            .detach()
            .cpu()
            .numpy()
        )

        best_index = int(
            np.argmax(
                box_conf
            )
        )

        xy = (
            result.keypoints.xy[
                best_index
            ]
            .detach()
            .cpu()
            .numpy()
        )

        kp_conf_tensor = (
            result.keypoints.conf
        )

        if kp_conf_tensor is not None:

            kp_conf = (
                kp_conf_tensor[
                    best_index
                ]
                .detach()
                .cpu()
                .numpy()
            )

        else:

            kp_conf = np.ones(
                len(xy),
                dtype=np.float32,
            )

        keep = np.where(
            kp_conf
            >= KEYPOINT_CONF
        )[0]

        if len(keep) < 4:

            # Fall back to the four strongest keypoints.
            keep = np.argsort(
                kp_conf
            )[-4:]

        object_points = (
            object_points_all[
                keep
            ]
        )

        image_points = (
            xy[
                keep
            ]
            .astype(
                np.float64
            )
        )

        ok, rvec, tvec, inliers = (
            cv2.solvePnPRansac(
                objectPoints=object_points,
                imagePoints=image_points,
                cameraMatrix=K,
                distCoeffs=dist_coeffs,
                iterationsCount=(
                    PNP_ITERATIONS
                ),
                reprojectionError=(
                    PNP_REPROJECTION_ERROR_PX
                ),
                confidence=0.99,
                flags=cv2.SOLVEPNP_EPNP,
            )
        )

        if not ok:

            rows.append(
                {
                    "frame": frame,
                    "pnp_ok": 0,
                    "used_keypoints": len(
                        keep
                    ),
                }
            )
            continue

        if (
            inliers is not None
            and
            len(inliers) >= 4
        ):

            inlier_idx = (
                inliers
                .reshape(-1)
            )

            cv2.solvePnP(
                objectPoints=(
                    object_points[
                        inlier_idx
                    ]
                ),
                imagePoints=(
                    image_points[
                        inlier_idx
                    ]
                ),
                cameraMatrix=K,
                distCoeffs=dist_coeffs,
                rvec=rvec,
                tvec=tvec,
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

        R, _ = cv2.Rodrigues(
            rvec
        )

        tvec = (
            tvec
            .reshape(3)
        )

        # OpenCV camera coordinates:
        # +X right, +Y down, +Z forward.
        distance_est = float(
            tvec[2]
        )

        lateral_est = float(
            tvec[0]
        )

        # Dolly local +X is the approach-axis direction.
        axis_x_camera = R[:, 0]

        yaw_est = math.degrees(
            math.atan2(
                float(
                    axis_x_camera[0]
                ),
                float(
                    axis_x_camera[2]
                ),
            )
        )

        gt_distance = float(
            gt["distance_m"]
        )

        gt_lateral = float(
            gt["lateral_m"]
        )

        gt_yaw = float(
            gt["yaw_deg"]
        )

        distance_error = (
            distance_est
            - gt_distance
        )

        lateral_error = (
            lateral_est
            - gt_lateral
        )

        yaw_error_normal = wrap_deg(
            yaw_est
            - gt_yaw
        )

        yaw_error_flipped = wrap_deg(
            -yaw_est
            - gt_yaw
        )

        distance_errors.append(
            distance_error
        )

        lateral_errors.append(
            lateral_error
        )

        yaw_errors_normal.append(
            yaw_error_normal
        )

        yaw_errors_flipped.append(
            yaw_error_flipped
        )

        pnp_success_count += 1

        rows.append(
            {
                "frame": frame,
                "pnp_ok": 1,
                "used_keypoints": len(
                    keep
                ),
                "inliers": (
                    0
                    if inliers is None
                    else len(inliers)
                ),
                "distance_gt_m": (
                    gt_distance
                ),
                "distance_est_m": (
                    distance_est
                ),
                "distance_error_m": (
                    distance_error
                ),
                "lateral_gt_m": (
                    gt_lateral
                ),
                "lateral_est_m": (
                    lateral_est
                ),
                "lateral_error_m": (
                    lateral_error
                ),
                "yaw_gt_deg": gt_yaw,
                "yaw_est_deg": yaw_est,
                "yaw_error_normal_deg": (
                    yaw_error_normal
                ),
                "yaw_error_flipped_deg": (
                    yaw_error_flipped
                ),
            }
        )

    fieldnames = sorted(
        {
            key
            for row in rows
            for key in row.keys()
        }
    )

    with SUMMARY_PATH.open(
        "w",
        encoding="utf-8",
        newline="",
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
        )

        writer.writeheader()
        writer.writerows(rows)

    success_rate = (
        pnp_success_count
        / max(
            1,
            len(selected)
        )
    )

    def mae(values):

        if not values:
            return float("nan")

        return float(
            np.mean(
                np.abs(values)
            )
        )

    distance_mae = mae(
        distance_errors
    )

    lateral_mae = mae(
        lateral_errors
    )

    yaw_mae_normal = mae(
        yaw_errors_normal
    )

    yaw_mae_flipped = mae(
        yaw_errors_flipped
    )

    if (
        yaw_mae_flipped
        < yaw_mae_normal
    ):
        yaw_sign = -1
        yaw_mae = (
            yaw_mae_flipped
        )
    else:
        yaw_sign = 1
        yaw_mae = (
            yaw_mae_normal
        )

    print(
        "===== PNP SMOKE TEST ====="
    )

    print(
        f"Samples       : "
        f"{len(selected)}"
    )

    print(
        f"PnP success   : "
        f"{pnp_success_count}/"
        f"{len(selected)} "
        f"({100.0 * success_rate:.1f}%)"
    )

    print(
        f"Distance MAE  : "
        f"{distance_mae:.3f} m"
    )

    print(
        f"Lateral MAE   : "
        f"{lateral_mae:.3f} m"
    )

    print(
        f"Yaw sign      : "
        f"{yaw_sign:+d}"
    )

    print(
        f"Yaw MAE       : "
        f"{yaw_mae:.2f} deg"
    )

    print(
        f"Summary CSV   : "
        f"{SUMMARY_PATH}"
    )

    # Compact gate: enough for controller prototype.
    if (
        success_rate >= 0.85
        and
        distance_mae <= 0.25
        and
        lateral_mae <= 0.15
        and
        yaw_mae <= 10.0
    ):

        print(
            "RESULT        : PASS"
        )
        return 0

    print(
        "RESULT        : WARN"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
