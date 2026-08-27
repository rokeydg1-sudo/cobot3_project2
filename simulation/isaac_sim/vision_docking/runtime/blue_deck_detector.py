#!/usr/bin/env python3
"""Locate a Dolly by the colour of its deck, and say when not to trust it.

Why colour and not the trained network
--------------------------------------
`dolly_pose_v1_best.pt` was trained against keypoints that put the Dolly's deck
at z = 0.23-0.47 m. Measured off a live frame, the real deck in this factory
sits at z = 0.85 m and up, so every label the SDG generated was drawn around the
floor under the Dolly rather than the Dolly. The weights peak at 0.71 confidence
on a clean view and fire on maybe a quarter of frames, and a solvePnP built on
those keypoints would return a pose that is wrong by design. The deck, by
contrast, is the only saturated blue in a factory of white floor and grey steel:
one HSV range separates it at 80-100% of frames between 1 and 8 m.

What this returns and what it deliberately does not
---------------------------------------------------
Bearing only, plus the box for the operator overlay.

Bearing follows from fx and cx alone, and both are authored by
scripts/scale_intrinsics.py rather than estimated, so it carries no hidden
assumption. Range does not: recovering it from apparent width needs a deck
width, and fitting one over real frames returned 3.29 m because a second Dolly
parked behind the target merges into the same blue blob. Recovering it from the
floor plane needs the camera height and pitch, and projecting the known Dolly
geometry with the bridge's own constants lands visibly low, so those are not
trustworthy either. Range is already known from odometry, so the snapshot does
not need to guess at it - it only has to answer "how far off to the side is the
Dolly from where we assumed", which is exactly the lateral error that docking
cares about.

Rejections
----------
Every gate below came from a measured failure on the 465-frame capture:

* clipped horizontally - f_0038 masks the deck correctly, but it runs off the
  left edge, so the centroid sits 28 degrees left of the real bearing. A
  centroid of a cropped object is not the centroid of the object.
* too small - f_0029 catches a 2306-pixel sliver of a Dolly that is otherwise
  outside the frame, and reports a bearing 47 degrees off.
* wrong shape - the deck is a flat slab seen close to edge-on. Anything tall is
  background machinery.
* too low in frame - the deck is always above the floor line.

A rejection is not a failure. The caller keeps its coordinate-based target, and
the dock proceeds exactly as it did before vision existed.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np


# Saturated blue. The deck reads around H=105 in OpenCV's 0-179 hue scale; the
# window is wide enough to survive the specular highlight along the top edge and
# narrow enough to exclude the pale blue-grey floor reflections.
HSV_LOWER = (95, 80, 50)
HSV_UPPER = (130, 255, 255)

# Gates, as fractions of the frame unless stated otherwise.
# Systematic offset between the deck's blue centroid and the bearing to the
# Dolly's origin, measured over 51 unclipped frames at 1.5-6 m: the centroid
# reads 2.01 degrees to one side with a spread of 2.36. A constant that
# consistent is a mounting and geometry offset, not noise, so it is removed
# rather than averaged over. Re-measure this if the camera pose in
# create_docking_camera() changes.
BEARING_BIAS_DEG = 2.01

MIN_AREA_FRACTION = 0.010      # f_0029's false positive is 0.010 of a 640x360.
MIN_WIDTH_FRACTION = 0.12
MAX_WIDTH_FRACTION = 0.98
MIN_ASPECT_RATIO = 1.8         # deck seen edge-on is much wider than it is tall

# Upper bound on the same ratio, and a floor on absolute thickness.
#
# The factory floor carries painted lines in the same blue as the deck, and
# they were being reported as Dollies. Colour cannot separate them but shape
# can: the deck is a slab 0.257 m thick, which at the 3-5 m snapshot range
# covers 66-107 pixels, while a painted line is a handful of pixels tall and
# runs the width of the aisle. Anything that long and that thin is floor
# marking, not cargo.
MAX_ASPECT_RATIO = 14.0
MIN_HEIGHT_PX = 14
MAX_CENTRE_V_FRACTION = 0.75   # deck never sits at the bottom of the frame
EDGE_MARGIN_PX = 2


@dataclass
class Detection:
    """What was seen, separated into what is certain and what is merely likely.

    Two answers, because the two questions have very different reliabilities.

    `present` - is there a Dolly in front of the robot? Yes on 80-100% of
    frames between 1 and 8 m. A Dolly deck is the only large saturated blue
    thing at deck height in this factory, so its presence is not in doubt.

    `ok` - is the measured bearing good enough to steer by? Much rarer. The
    deck merges with the blue machinery along the aisle into a single
    component, 3.3 m of contiguous blue where the deck is 1.24 m, and no
    colour rule separates the two because they are the same colour at
    different depths. Attempts at edge-depth ordering and at an expected-width
    gate both worked on most frames and then produced 25-degree outliers, which
    is exactly the kind of error that must not reach the wheels.

    So presence gates the approach and bearing only ever trims it.
    """

    present: bool
    ok: bool
    reason: str
    bearing_deg: float = 0.0
    box: tuple = ()            # (x, y, w, h) for the overlay
    area_px: int = 0
    area_fraction: float = 0.0

    @property
    def label(self):
        if self.ok:
            return f"DOLLY CONFIRMED  bearing {self.bearing_deg:+.1f} deg"
        if self.present:
            return f"DOLLY CONFIRMED  (bearing unusable: {self.reason})"
        return f"NO DOLLY  ({self.reason})"


def blue_mask(image):
    """Binary mask of the deck colour, with speckle and gaps cleaned up."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1
    )
    # Closing spans the grey cross-braces that interrupt the deck lengthwise.
    return cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2
    )


def detect(image, fx, cx, expected_width_px=None, width_tolerance=0.45):
    """Find the Dolly deck. Returns a Detection, never raises.

    `expected_width_px` is how wide the deck should look, computed by the
    caller from the deck width the bridge measured off the asset and the range
    it is standing at. Both are known exactly at snapshot time, and supplying
    them turns blob selection from a guess into a test.

    It matters because "largest blue blob" is wrong here. The factory has blue
    machinery along the aisles, and it merges with the deck into one component:
    a Dolly measured at 2.84 m produced 3.16 m of contiguous blue, two and a
    half times the 1.24 m the deck actually is. Picking the blob whose width
    matches prediction rejects the merge instead of averaging over it.
    """
    height, width = image.shape[:2]

    if int(image.max()) <= 2:
        # Isaac drops the occasional dead frame. The model once scored one of
        # these as a Dolly at 0.25, so a black frame is refused outright rather
        # than measured.
        return Detection(False, False, "blank frame")

    mask = blue_mask(image)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)

    frame_area = float(width * height)
    candidates = []
    for index in range(1, count):
        candidates.append(
            (
                int(stats[index, cv2.CC_STAT_AREA]),
                int(stats[index, cv2.CC_STAT_LEFT]),
                int(stats[index, cv2.CC_STAT_TOP]),
                int(stats[index, cv2.CC_STAT_WIDTH]),
                int(stats[index, cv2.CC_STAT_HEIGHT]),
                float(centroids[index][0]),
                float(centroids[index][1]),
            )
        )

    if not candidates:
        return Detection(False, False, "no blue region")

    # Presence is decided on the blue that is there, before any of the gates
    # that decide whether it can be measured. The widest blob is the right
    # thing to ask about here even when it is a merge of deck and machinery:
    # a merge still means a deck is in it.
    widest = max(candidates, key=lambda c: c[0])
    present = (
        widest[0] / frame_area >= MIN_AREA_FRACTION
        and widest[3] >= MIN_WIDTH_FRACTION * width
        and widest[6] <= MAX_CENTRE_V_FRACTION * height
        # Presence is gated on shape as well, not only size. Without this the
        # overlay announced DOLLY DETECTED over a painted floor line, which is
        # worse than a missed detection: it is a confident wrong answer.
        and widest[4] >= MIN_HEIGHT_PX
        and widest[3] <= MAX_ASPECT_RATIO * max(1, widest[4])
    )
    present_box = (widest[1], widest[2], widest[3], widest[4])

    def refuse(reason, blob=None):
        chosen = blob or widest
        return Detection(
            present,
            False,
            reason,
            box=(chosen[1], chosen[2], chosen[3], chosen[4]),
            area_px=chosen[0],
            area_fraction=chosen[0] / frame_area,
        )

    if expected_width_px:
        low = expected_width_px * (1.0 - width_tolerance)
        high = expected_width_px * (1.0 + width_tolerance)
        matching = [c for c in candidates if low <= c[3] <= high]
        if not matching:
            return refuse(
                f"no blob near {expected_width_px:.0f} px wide "
                f"(widest was {widest[3]} px)"
            )
        # Among plausible widths, the biggest is the deck; the rest are
        # fragments of the same object or distant machinery.
        best = max(matching, key=lambda c: c[0])
    else:
        best = max(candidates, key=lambda c: c[0])

    area, x, y, w, h, centroid_u, centroid_v = best
    box = (x, y, w, h)
    fraction = area / frame_area

    if fraction < MIN_AREA_FRACTION:
        return refuse("too small", best)
    if x <= EDGE_MARGIN_PX or x + w >= width - EDGE_MARGIN_PX:
        return refuse("clipped at frame edge", best)
    if not MIN_WIDTH_FRACTION * width <= w <= MAX_WIDTH_FRACTION * width:
        return refuse("implausible width", best)
    if w < MIN_ASPECT_RATIO * max(1, h):
        return refuse("not deck-shaped", best)
    if h < MIN_HEIGHT_PX:
        return refuse("too thin - floor marking", best)
    if w > MAX_ASPECT_RATIO * max(1, h):
        return refuse("too elongated - floor marking", best)
    if centroid_v > MAX_CENTRE_V_FRACTION * height:
        return refuse("too low in frame", best)

    bearing = math.degrees(math.atan((centroid_u - cx) / fx)) - BEARING_BIAS_DEG
    return Detection(
        present, True, "ok", bearing_deg=bearing, box=box, area_px=area,
        area_fraction=fraction,
    )


def draw(image, detection):
    """Overlay for rqt_image_view.

    Written to be read across a room during a demonstration, so the headline is
    whether a Dolly was found and nothing else competes with it. The reason a
    bearing was not usable is diagnostic detail and goes in small text at the
    bottom; it is not a failure and should not read like one, because the
    approach proceeds either way.
    """
    canvas = image.copy()
    height, width = canvas.shape[:2]

    # Green for any detection, including one whose bearing was not used.
    #
    # These were two colours at first, green when the bearing was steered on
    # and amber when only presence was confirmed. But that distinction is about
    # what the controller did with the number, not about whether the Dolly was
    # recognised, and an amber box in front of a Dolly that had just been
    # detected on 5 of 5 frames read as a failure to anyone watching. The
    # distinction still appears, in the small line at the bottom, where it
    # belongs.
    if detection.present:
        colour = (0, 220, 0)
        headline = "DOLLY DETECTED"
    else:
        colour = (0, 0, 230)
        headline = "NO DOLLY"

    if detection.box:
        x, y, w, h = detection.box
        cv2.rectangle(canvas, (x, y), (x + w, y + h), colour, 3)
        # Corner ticks read as a detection box rather than as a stray
        # rectangle when the deck runs to the frame edge.
        tick = max(8, min(w, h) // 4)
        for cx0, cy0, dx, dy in (
            (x, y, 1, 1), (x + w, y, -1, 1),
            (x, y + h, 1, -1), (x + w, y + h, -1, -1),
        ):
            cv2.line(canvas, (cx0, cy0), (cx0 + dx * tick, cy0), colour, 5)
            cv2.line(canvas, (cx0, cy0), (cx0, cy0 + dy * tick), colour, 5)
        if detection.ok:
            centre = x + w // 2
            cv2.line(canvas, (centre, y), (centre, y + h), colour, 2)

    # Banner behind the headline so it stays legible over a bright factory.
    cv2.rectangle(canvas, (0, 0), (width, 34), (0, 0, 0), -1)
    cv2.putText(
        canvas, headline, (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2, cv2.LINE_AA,
    )
    if detection.present:
        cv2.putText(
            canvas,
            f"deck {detection.box[2]}x{detection.box[3]} px  "
            f"{detection.area_fraction * 100:.1f}% of frame",
            (width - 330, 23),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1, cv2.LINE_AA,
        )
        if detection.ok:
            cv2.putText(
                canvas, f"bearing {detection.bearing_deg:+.1f} deg",
                (10, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2, cv2.LINE_AA,
            )
        else:
            cv2.putText(
                canvas, f"bearing not used: {detection.reason}",
                (10, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA,
            )
    else:
        cv2.putText(
            canvas, detection.reason, (10, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA,
        )
    return canvas
