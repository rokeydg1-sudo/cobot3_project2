"""
Fast validation for Dolly Pose Pilot SDG.

Checks
------
1. images / labels / debug = 20 / 20 / 20
2. each YOLO Pose label has 29 values
3. class id = 0
4. bbox values are normalized to [0, 1]
5. visible keypoints have normalized coordinates in [0, 1]
6. at least 4 keypoints are visible
7. metadata has 20 rows
8. select representative debug frames for quick visual review

Result
------
PASS / FAIL printed to terminal.
Detailed results saved to:
outputs/logs/pilot_validation.log
"""

import csv
from pathlib import Path


# ============================================================
# Paths
# ============================================================

SCRIPT_PATH = Path(__file__).resolve()

VISION_DOCKING_DIR = SCRIPT_PATH.parents[1]

PILOT_DIR = (
    VISION_DOCKING_DIR
    / "outputs"
    / "pilot"
)

IMAGE_DIR = PILOT_DIR / "images"
LABEL_DIR = PILOT_DIR / "labels"
DEBUG_DIR = PILOT_DIR / "debug"

METADATA_PATH = (
    PILOT_DIR
    / "metadata.csv"
)

LOG_PATH = (
    VISION_DOCKING_DIR
    / "outputs"
    / "logs"
    / "pilot_validation.log"
)


# ============================================================
# Expected dataset configuration
# ============================================================

EXPECTED_FRAMES = 20
EXPECTED_KEYPOINTS = 8

# class + bbox(4) + keypoints(8 * 3)
EXPECTED_LABEL_VALUES = (
    1
    + 4
    + EXPECTED_KEYPOINTS * 3
)

MIN_VISIBLE_KEYPOINTS = 4


# ============================================================
# Logging
# ============================================================

details = []


def log(message):
    details.append(str(message))


def write_log():

    LOG_PATH.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    LOG_PATH.write_text(
        "\n".join(details) + "\n",
        encoding="utf-8",
    )


# ============================================================
# Validation
# ============================================================

def main():

    errors = []

    log(
        "===== PILOT DATASET VALIDATION ====="
    )

    # --------------------------------------------------------
    # 1. File counts
    # --------------------------------------------------------

    images = sorted(
        IMAGE_DIR.glob("frame_*.png")
    )

    labels = sorted(
        LABEL_DIR.glob("frame_*.txt")
    )

    debug_images = sorted(
        DEBUG_DIR.glob(
            "frame_*_overlay.png"
        )
    )

    log("")
    log("===== FILE COUNTS =====")
    log(f"Images : {len(images)}")
    log(f"Labels : {len(labels)}")
    log(f"Debug  : {len(debug_images)}")

    if len(images) != EXPECTED_FRAMES:
        errors.append(
            f"Expected {EXPECTED_FRAMES} images, "
            f"found {len(images)}"
        )

    if len(labels) != EXPECTED_FRAMES:
        errors.append(
            f"Expected {EXPECTED_FRAMES} labels, "
            f"found {len(labels)}"
        )

    if len(debug_images) != EXPECTED_FRAMES:
        errors.append(
            f"Expected {EXPECTED_FRAMES} debug images, "
            f"found {len(debug_images)}"
        )


    # --------------------------------------------------------
    # 2. YOLO Pose labels
    # --------------------------------------------------------

    log("")
    log("===== LABEL VALIDATION =====")

    for label_path in labels:

        text = (
            label_path
            .read_text(
                encoding="utf-8"
            )
            .strip()
        )

        values = text.split()

        if len(values) != EXPECTED_LABEL_VALUES:

            errors.append(
                f"{label_path.name}: "
                f"expected {EXPECTED_LABEL_VALUES} values, "
                f"found {len(values)}"
            )

            continue

        try:
            numbers = [
                float(v)
                for v in values
            ]

        except ValueError:

            errors.append(
                f"{label_path.name}: "
                "non-numeric value detected"
            )

            continue


        # ----------------------------------------------------
        # Class ID
        # ----------------------------------------------------

        class_id = int(
            numbers[0]
        )

        if class_id != 0:

            errors.append(
                f"{label_path.name}: "
                f"class id = {class_id}"
            )


        # ----------------------------------------------------
        # Bounding box
        # ----------------------------------------------------

        bbox = numbers[1:5]

        for i, value in enumerate(bbox):

            if not (
                0.0 <= value <= 1.0
            ):

                errors.append(
                    f"{label_path.name}: "
                    f"bbox[{i}] out of range "
                    f"({value})"
                )


        # ----------------------------------------------------
        # Keypoints
        # ----------------------------------------------------

        kp_values = numbers[5:]

        visible_count = 0

        for kp_index in range(
            EXPECTED_KEYPOINTS
        ):

            offset = (
                kp_index * 3
            )

            x = kp_values[offset]
            y = kp_values[offset + 1]

            visibility = int(
                kp_values[offset + 2]
            )

            if visibility == 2:

                visible_count += 1

                if not (
                    0.0 <= x <= 1.0
                    and
                    0.0 <= y <= 1.0
                ):

                    errors.append(
                        f"{label_path.name}: "
                        f"P{kp_index + 1} "
                        "visible but coordinate "
                        "out of range "
                        f"(x={x}, y={y})"
                    )

            elif visibility == 0:

                if (
                    abs(x) > 1e-8
                    or
                    abs(y) > 1e-8
                ):

                    errors.append(
                        f"{label_path.name}: "
                        f"P{kp_index + 1} "
                        "visibility=0 but "
                        f"coordinate=({x}, {y})"
                    )

            else:

                errors.append(
                    f"{label_path.name}: "
                    f"P{kp_index + 1} "
                    f"unexpected visibility="
                    f"{visibility}"
                )


        if (
            visible_count
            < MIN_VISIBLE_KEYPOINTS
        ):

            errors.append(
                f"{label_path.name}: "
                f"only {visible_count} "
                "visible keypoints"
            )

        log(
            f"{label_path.name}: "
            f"visible={visible_count}"
        )


    # --------------------------------------------------------
    # 3. Metadata
    # --------------------------------------------------------

    log("")
    log("===== METADATA =====")

    metadata_rows = []

    if not METADATA_PATH.exists():

        errors.append(
            "metadata.csv not found"
        )

    else:

        with open(
            METADATA_PATH,
            "r",
            encoding="utf-8",
        ) as f:

            reader = csv.DictReader(f)

            metadata_rows = list(
                reader
            )

        log(
            f"Rows: {len(metadata_rows)}"
        )

        if (
            len(metadata_rows)
            != EXPECTED_FRAMES
        ):

            errors.append(
                f"metadata rows: "
                f"expected {EXPECTED_FRAMES}, "
                f"found {len(metadata_rows)}"
            )


    # --------------------------------------------------------
    # 4. Representative frames
    #
    # Quick manual visual review:
    # nearest / farthest / left / right / max yaw
    # --------------------------------------------------------

    representative_frames = []

    if metadata_rows:

        def value(
            row,
            column,
        ):
            return float(
                row[column]
            )


        nearest = min(
            metadata_rows,
            key=lambda r: value(
                r,
                "distance_m",
            ),
        )

        farthest = max(
            metadata_rows,
            key=lambda r: value(
                r,
                "distance_m",
            ),
        )

        leftmost = min(
            metadata_rows,
            key=lambda r: value(
                r,
                "lateral_m",
            ),
        )

        rightmost = max(
            metadata_rows,
            key=lambda r: value(
                r,
                "lateral_m",
            ),
        )

        max_yaw = max(
            metadata_rows,
            key=lambda r: abs(
                value(
                    r,
                    "yaw_deg",
                )
            ),
        )

        representative_frames = [
            (
                "nearest",
                nearest["frame"],
            ),
            (
                "farthest",
                farthest["frame"],
            ),
            (
                "leftmost",
                leftmost["frame"],
            ),
            (
                "rightmost",
                rightmost["frame"],
            ),
            (
                "max_abs_yaw",
                max_yaw["frame"],
            ),
        ]

        log("")
        log(
            "===== REPRESENTATIVE DEBUG FRAMES ====="
        )

        for role, frame in (
            representative_frames
        ):

            debug_path = (
                DEBUG_DIR
                / f"{frame}_overlay.png"
            )

            log(
                f"{role:12s}: "
                f"{debug_path}"
            )


    # --------------------------------------------------------
    # Result
    # --------------------------------------------------------

    log("")
    log("===== RESULT =====")

    if errors:

        log("FAIL")

        log("")
        log("===== ERRORS =====")

        for error in errors:
            log(error)

        write_log()

        print("")
        print(
            "PILOT VALIDATION: FAIL"
        )

        print(
            f"Errors : {len(errors)}"
        )

        print(
            f"Log    : {LOG_PATH}"
        )

        return 1


    log("PASS")

    write_log()

    print("")
    print(
        "PILOT VALIDATION: PASS"
    )

    print(
        f"Images / Labels / Debug : "
        f"{len(images)} / "
        f"{len(labels)} / "
        f"{len(debug_images)}"
    )

    print(
        f"Log : {LOG_PATH}"
    )

    print("")
    print(
        "Quick visual review:"
    )

    for role, frame in (
        representative_frames
    ):

        print(
            f"  {role:12s} "
            f"{frame}_overlay.png"
        )

    return 0


if __name__ == "__main__":

    raise SystemExit(
        main()
    )