#!/usr/bin/env python3
"""
Train the Dolly YOLO Pose model.

Run with a normal Python environment (NOT Isaac Sim Script Editor).

Outputs:
- Ultralytics training run:
  vision/isaac_sim/vision_docking/outputs/training/dolly_pose_yolo26n_v1/
- Runtime model copy:
  vision/isaac_sim/vision_docking/outputs/weights/dolly_pose_v1_best.pt
- Small training metadata text file
"""

from pathlib import Path
import shutil
import sys
from datetime import datetime


REPO_ROOT = Path(__file__).resolve().parents[4]
VISION_DIR = REPO_ROOT / "vision" / "isaac_sim" / "vision_docking"

DATASET_YAML = (
    VISION_DIR
    / "outputs"
    / "dataset"
    / "dataset.yaml"
)

TRAIN_PROJECT = (
    VISION_DIR
    / "outputs"
    / "training"
)

RUN_NAME = "dolly_pose_yolo26n_v1"

FINAL_WEIGHTS_DIR = (
    VISION_DIR
    / "outputs"
    / "weights"
)

FINAL_MODEL = (
    FINAL_WEIGHTS_DIR
    / "dolly_pose_v1_best.pt"
)

METADATA_PATH = (
    FINAL_WEIGHTS_DIR
    / "dolly_pose_v1_metadata.txt"
)


# Compact first training configuration.
BASE_MODEL = "yolo26n-pose.pt"
EPOCHS = 80
IMAGE_SIZE = 640
PATIENCE = 15
DEVICE = 0
BATCH = -1  # Ultralytics AutoBatch on a single GPU


def main():

    if not DATASET_YAML.exists():
        raise FileNotFoundError(
            f"dataset.yaml not found: {DATASET_YAML}"
        )

    try:
        import torch
    except ImportError as exc:
        raise RuntimeError(
            "PyTorch is not installed in this Python environment."
        ) from exc

    try:
        from ultralytics import YOLO
        import ultralytics
    except ImportError as exc:
        raise RuntimeError(
            "Ultralytics is not installed. "
            "Install it in the training Python environment first."
        ) from exc

    print("===== YOLO POSE TRAINING PRECHECK =====")
    print("Python       :", sys.executable)
    print("PyTorch      :", torch.__version__)
    print("Ultralytics  :", ultralytics.__version__)
    print("CUDA         :", torch.cuda.is_available())

    if not torch.cuda.is_available():
        raise RuntimeError(
            "CUDA GPU is not available in this Python environment. "
            "Do not start CPU training for this project."
        )

    print("GPU          :", torch.cuda.get_device_name(DEVICE))
    print("Dataset      :", DATASET_YAML)
    print("Base model   :", BASE_MODEL)
    print("Epochs       :", EPOCHS)
    print("Image size   :", IMAGE_SIZE)
    print("Batch        :", BATCH)
    print("")

    model = YOLO(BASE_MODEL)

    model.train(
        data=str(DATASET_YAML),
        epochs=EPOCHS,
        imgsz=IMAGE_SIZE,
        batch=BATCH,
        device=DEVICE,
        patience=PATIENCE,
        amp=True,
        workers=8,
        project=str(TRAIN_PROJECT),
        name=RUN_NAME,
        exist_ok=True,
        verbose=True,
        plots=True,
        save=True,
    )

    best_path = (
        TRAIN_PROJECT
        / RUN_NAME
        / "weights"
        / "best.pt"
    )

    if not best_path.exists():
        raise RuntimeError(
            f"Training finished but best.pt was not found: {best_path}"
        )

    FINAL_WEIGHTS_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    shutil.copy2(
        best_path,
        FINAL_MODEL,
    )

    METADATA_PATH.write_text(
        "\n".join(
            [
                "Dolly YOLO Pose model",
                f"created={datetime.now().isoformat()}",
                f"base_model={BASE_MODEL}",
                f"dataset={DATASET_YAML}",
                f"epochs={EPOCHS}",
                f"imgsz={IMAGE_SIZE}",
                f"batch={BATCH}",
                f"patience={PATIENCE}",
                f"source_best={best_path}",
                f"runtime_model={FINAL_MODEL}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    print("")
    print("===== YOLO POSE TRAINING COMPLETE =====")
    print("best.pt      :", best_path)
    print("runtime model:", FINAL_MODEL)
    print("metadata     :", METADATA_PATH)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
