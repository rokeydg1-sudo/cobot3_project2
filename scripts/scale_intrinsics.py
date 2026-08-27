#!/usr/bin/env python3
"""Write the camera intrinsics the bridge should render with.

Dropping the render resolution is the cheapest way to cut Isaac's per-frame
render cost, but changing CAMERA_WIDTH on its own silently narrows the field of
view: the bridge derives the USD aperture as focal * width / fx, so halving the
width while leaving fx alone halves the aperture too. Scaling fx, fy, cx and cy
by the same factor keeps the aperture - and therefore the field of view - the
same as the authored camera.

    python3 scripts/scale_intrinsics.py 640 360

The authored camera is a 30.6 degree telephoto, and that is too tight to dock
with. A 1.24 m wide Dolly fills the whole frame at 2.6 m and overflows it below
that, so every detection inside docking range came back clipped and the pinhole
distance estimate had no width left to measure. Passing a target horizontal
field of view rebuilds fx and fy for that angle instead of copying the authored
one:

    python3 scripts/scale_intrinsics.py 640 360 60

At 60 degrees the Dolly spans 54% of the frame at 2 m and 36% at 3 m, which puts
the docking approach inside the 0.08-0.75 bbox area band the model was trained
on (see vision_config.MIN/MAX_BBOX_AREA_RATIO) instead of past the top of it.

Writes camera_intrinsics_<width>.npz next to the original and leaves the
original untouched.
"""
import math
import sys
from pathlib import Path

import numpy as np

# Dolly footprint from vision_config.DOLLY_KEYPOINTS_LOCAL: P5..P8 span
# +/-0.43 m along x and +/-0.62 m along y, so the widest face is 1.24 m across
# and the body stands 0.47 m tall. Used only to report apparent size.
DOLLY_WIDTH_M = 1.24
DOLLY_HEIGHT_M = 0.47

CONFIG_DIR = (
    Path(__file__).resolve().parents[1]
    / "simulation"
    / "isaac_sim"
    / "vision_docking"
    / "config"
)
SOURCE = CONFIG_DIR / "camera_intrinsics.npz"


def main():
    width, height = int(sys.argv[1]), int(sys.argv[2])
    hfov_deg = float(sys.argv[3]) if len(sys.argv) > 3 else None
    data = np.load(SOURCE)
    K = np.asarray(data["K"], dtype=float)
    source_width = int(data["width"])
    source_height = int(data["height"])

    scale_x = width / source_width
    scale_y = height / source_height
    if abs(scale_x - scale_y) > 1e-9:
        raise SystemExit(
            f"aspect ratio would change: {source_width}x{source_height} "
            f"-> {width}x{height}"
        )

    scaled = K.copy()
    scaled[0, 0] *= scale_x   # fx
    scaled[1, 1] *= scale_y   # fy
    scaled[0, 2] *= scale_x   # cx
    scaled[1, 2] *= scale_y   # cy

    if hfov_deg is not None:
        if not 5.0 < hfov_deg < 175.0:
            raise SystemExit(f"implausible horizontal fov: {hfov_deg}")
        # Square pixels: fy follows fx, so the vertical angle comes out of the
        # aspect ratio on its own and does not need a second parameter.
        focal = width / 2.0 / math.tan(math.radians(hfov_deg) / 2.0)
        scaled[0, 0] = focal
        scaled[1, 1] = focal
        # Recentre too. An off-centre principal point copied from a different
        # focal length puts the optical axis somewhere the renderer will not
        # put it, and solvePnP then reports a lateral offset that is not real.
        scaled[0, 2] = width / 2.0
        scaled[1, 2] = height / 2.0

    destination = CONFIG_DIR / f"camera_intrinsics_{width}.npz"
    np.savez(
        destination,
        K=scaled,
        width=width,
        height=height,
        distortion=data["distortion"],
        camera_path=data["camera_path"],
    )

    def fov(matrix, w, h):
        return (
            2 * np.degrees(np.arctan(w / 2 / matrix[0, 0])),
            2 * np.degrees(np.arctan(h / 2 / matrix[1, 1])),
        )

    before = fov(K, source_width, source_height)
    after = fov(scaled, width, height)
    print(f"source  {source_width}x{source_height}  fx={K[0, 0]:.2f} cx={K[0, 2]:.1f}")
    print(f"output  {width}x{height}  fx={scaled[0, 0]:.2f} cx={scaled[0, 2]:.1f}")
    print(f"fov before  h={before[0]:.2f} deg  v={before[1]:.2f} deg")
    print(f"fov after   h={after[0]:.2f} deg  v={after[1]:.2f} deg")

    # What the detector will actually be looking at. A Dolly wider than the
    # frame is the failure this script exists to prevent, so say so plainly.
    print("\napparent Dolly size (fraction of frame)")
    for distance in (1.5, 2.0, 2.5, 3.0, 4.0, 5.0):
        span_w = scaled[0, 0] * DOLLY_WIDTH_M / distance / width
        span_h = scaled[1, 1] * DOLLY_HEIGHT_M / distance / height
        note = "  OVERFLOWS FRAME" if span_w > 1.0 else ""
        print(
            f"  {distance:4.1f} m  width={span_w * 100:5.1f}%  "
            f"area={span_w * span_h:5.3f}{note}"
        )
    print("\n(model trained on bbox area 0.08-0.75)")
    print(f"wrote {destination}")


main()
