#!/usr/bin/env python3
"""
Compact post-SDG check for the Dolly YOLO Pose dataset.

Hard checks:
- train/val image and label counts
- metadata row counts
- label length = 29 values for 8 keypoints
- dataset.yaml exists

Reports:
- bbox touching image border ratio
- visible/occluded keypoint statistics
- sampled image brightness

This script does NOT modify the dataset.
"""

from pathlib import Path
import csv
import statistics
import sys

from PIL import Image, ImageStat


REPO_ROOT = Path(__file__).resolve().parents[4]
VISION_DIR = REPO_ROOT / "vision" / "isaac_sim" / "vision_docking"
DATASET = VISION_DIR / "outputs" / "dataset"

WIDTH = 1280
HEIGHT = 720
EXPECTED_VALUES = 29

TARGETS = {
    "train": 1000,
    "val": 200,
}


def load_metadata(split):
    path = DATASET / f"metadata_{split}.csv"
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def sample_brightness(paths, max_samples=40):
    if not paths:
        return None
    step = max(1, len(paths) // max_samples)
    selected = paths[::step][:max_samples]
    values = []
    for p in selected:
        with Image.open(p).convert("RGB") as im:
            stat = ImageStat.Stat(im)
            values.append(sum(stat.mean) / 3.0)
    return statistics.mean(values) if values else None


def main():
    errors = []
    warnings = []

    print("===== PRODUCTION DATASET QUICK CHECK =====")

    if not (DATASET / "dataset.yaml").exists():
        errors.append("dataset.yaml missing")

    for split, target in TARGETS.items():
        image_dir = DATASET / "images" / split
        label_dir = DATASET / "labels" / split

        images = sorted(image_dir.glob("*.png"))
        labels = sorted(label_dir.glob("*.txt"))
        rows = load_metadata(split)

        print(f"\n[{split}]")
        print(f"images   : {len(images)} / {target}")
        print(f"labels   : {len(labels)} / {target}")
        print(f"metadata : {len(rows)} / {target}")

        if len(images) != target:
            errors.append(f"{split}: image count {len(images)} != {target}")
        if len(labels) != target:
            errors.append(f"{split}: label count {len(labels)} != {target}")
        if len(rows) != target:
            errors.append(f"{split}: metadata count {len(rows)} != {target}")

        bad_label_count = 0
        for p in labels:
            if len(p.read_text(encoding="utf-8").split()) != EXPECTED_VALUES:
                bad_label_count += 1

        print(f"bad labels: {bad_label_count}")
        if bad_label_count:
            errors.append(f"{split}: {bad_label_count} malformed labels")

        if rows:
            touch_count = 0
            visible = []
            occluded = []

            for row in rows:
                xmin = float(row["bbox_xmin"])
                ymin = float(row["bbox_ymin"])
                xmax = float(row["bbox_xmax"])
                ymax = float(row["bbox_ymax"])

                # BBox clipped to the render edge => Dolly is truncated/touching edge.
                if (
                    xmin <= 1.0
                    or ymin <= 1.0
                    or xmax >= WIDTH - 1.0
                    or ymax >= HEIGHT - 1.0
                ):
                    touch_count += 1

                visible.append(int(row["visible_keypoints"]))
                occluded.append(int(row["occluded_keypoints"]))

            touch_ratio = 100.0 * touch_count / len(rows)

            print(f"bbox touches frame edge : {touch_count}/{len(rows)} ({touch_ratio:.1f}%)")
            print(
                "visible keypoints       : "
                f"min={min(visible)}, avg={statistics.mean(visible):.2f}, max={max(visible)}"
            )
            print(
                "occluded keypoints      : "
                f"min={min(occluded)}, avg={statistics.mean(occluded):.2f}, max={max(occluded)}"
            )

            # This is a warning, not a failure. Some truncation is useful.
            if touch_ratio > 30.0:
                warnings.append(
                    f"{split}: {touch_ratio:.1f}% of bboxes touch the image border; "
                    "this is rather high for the current docking use case."
                )

        brightness = sample_brightness(images)
        if brightness is not None:
            print(f"sample mean brightness  : {brightness:.1f} / 255")
            if brightness < 30.0:
                warnings.append(
                    f"{split}: sampled images are very dark (mean={brightness:.1f})."
                )

    print("\n===== RESULT =====")
    if errors:
        print("FAIL")
        for e in errors:
            print("ERROR:", e)
        for w in warnings:
            print("WARN :", w)
        return 1

    print("PASS")
    for w in warnings:
        print("WARN :", w)

    if not warnings:
        print("No critical post-SDG issues detected.")

    print("\nNext: YOLO Pose training.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
