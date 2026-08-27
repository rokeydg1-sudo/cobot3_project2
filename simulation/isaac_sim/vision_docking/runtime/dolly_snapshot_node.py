#!/usr/bin/env python3
"""Look at the Dolly once, from a standstill, and report what is there.

Why a snapshot and not a control loop
-------------------------------------
The previous node steered the robot from the camera at frame rate. That put a
detector with an intermittent hit rate directly in the control path, and because
its open-loop fallback published on the same topic, a run in which vision never
recognised anything still logged "VISION DOCKING engaged". The failure was
invisible from the outside.

Measuring once, while stopped, removes both problems. The robot is stationary so
there is no motion blur and no pose staleness; the answer is a single value that
is either produced or not; and the controller applies it as a bounded correction
to a target it already had, so a refusal changes nothing rather than stranding
the robot mid-dock.

Protocol
--------
    /dock/snapshot_request   std_msgs/String   {"amr":..,"mission":..,"seq":..}
    /dock/snapshot_result    std_msgs/String   {"seq":.., "ok":.., ...}

The request carries a sequence number and the reply echoes it, so a late answer
to an abandoned request cannot be mistaken for the current one.

The debug image publishes continuously, not just on request, so
`rqt_image_view` shows a live feed with the box drawn on it throughout the run
rather than a frame that only updates twice a mission.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parent))
import blue_deck_detector as detector  # noqa: E402

VISION_DIR = Path(__file__).resolve().parents[1]


class DollySnapshotNode(Node):
    def __init__(self):
        super().__init__("dolly_snapshot_node")

        self.declare_parameter("image_topic", "/vision/front_camera/image_raw")
        self.declare_parameter("request_topic", "/dock/snapshot_request")
        self.declare_parameter("result_topic", "/dock/snapshot_result")
        self.declare_parameter("debug_image_topic",
                               "/vision/dolly_docking/debug_image")
        self.declare_parameter("intrinsics", "")
        # Frames to average over once a request arrives. The robot is stopped,
        # so these are near-identical; the point is to ride out the occasional
        # dead frame Isaac emits rather than to average away noise.
        self.declare_parameter("snapshot_frames", 5)
        self.declare_parameter("snapshot_window_sec", 2.0)
        # Refuse to answer with a frame older than this. A stale image taken
        # before the robot stopped would describe the wrong pose.
        self.declare_parameter("max_frame_age_sec", 1.0)
        # Write the frame and its overlay whenever a measurement is refused.
        # A rejection reason on its own does not say whether the detector was
        # wrong or the view was: "clipped at frame edge" is the correct answer
        # to a Dolly that really is too close, and the wrong answer to a blue
        # wall behind it. Only the picture distinguishes the two.
        self.declare_parameter("save_rejected_dir", "/tmp/snapshot_rejects")

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_at = 0.0

        self.pending = None
        self.samples = []

        intrinsics_param = str(self.get_parameter("intrinsics").value)
        self.fx, self.cx = self.load_intrinsics(intrinsics_param)

        self.result_publisher = self.create_publisher(
            String, str(self.get_parameter("result_topic").value), 10
        )
        self.debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_image_topic").value), 1
        )
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value),
            self.image_callback, 1,
        )
        self.create_subscription(
            String, str(self.get_parameter("request_topic").value),
            self.request_callback, 10,
        )
        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            "snapshot node ready: fx=%.1f cx=%.1f, waiting on %s"
            % (self.fx, self.cx,
               str(self.get_parameter("request_topic").value))
        )

    # ------------------------------------------------------------- setup

    def load_intrinsics(self, override):
        """fx and cx for the resolution actually being published.

        The bridge renders at whatever CAMERA_WIDTH says and derives its USD
        aperture from the matching intrinsics file, so reading the same file
        here is what keeps the two consistent. Guessing a default would silently
        reintroduce the field-of-view mismatch that made the first attempt at
        this fail.
        """
        if override:
            path = Path(override)
        else:
            path = VISION_DIR / "config" / "camera_intrinsics.npz"
        data = np.load(path)
        K = np.asarray(data["K"], dtype=float)
        self.intrinsics_width = int(data["width"])
        self.get_logger().info(
            "intrinsics %s (%dx%d)" % (path.name, data["width"], data["height"])
        )
        return float(K[0, 0]), float(K[0, 2])

    # --------------------------------------------------------- callbacks

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )
        self.latest_image_at = time.monotonic()

    def request_callback(self, msg):
        try:
            request = json.loads(msg.data)
        except ValueError:
            self.get_logger().warning("unparseable snapshot request, ignored")
            return

        # The controller knows how far away it stopped and how wide the bridge
        # measured the deck, so it can say how wide the deck should look. That
        # turns "which blue blob" from a guess into an arithmetic check.
        range_m = float(request.get("range_m", 0.0))
        deck_width_m = float(request.get("deck_width_m", 0.0))
        expected_width_px = (
            self.fx * deck_width_m / range_m
            if range_m > 0.1 and deck_width_m > 0.0
            else None
        )

        self.pending = {
            "seq": int(request.get("seq", 0)),
            "amr": str(request.get("amr", "?")),
            "mission": int(request.get("mission", -1)),
            "expected_width_px": expected_width_px,
            "deadline": time.monotonic()
            + float(self.get_parameter("snapshot_window_sec").value),
        }
        self.samples = []
        self.get_logger().info(
            "SNAPSHOT requested by %s mission %d (seq %d), expecting a deck "
            "%s wide at %.2f m"
            % (self.pending["amr"], self.pending["mission"] + 1,
               self.pending["seq"],
               f"{expected_width_px:.0f} px" if expected_width_px else "of any size",
               range_m)
        )

    # -------------------------------------------------------------- loop

    def tick(self):
        image = self.latest_image
        if image is None:
            return

        fresh = (
            time.monotonic() - self.latest_image_at
            <= float(self.get_parameter("max_frame_age_sec").value)
        )
        expected = self.pending["expected_width_px"] if self.pending else None
        result = detector.detect(image, self.fx, self.cx, expected)

        # Live overlay regardless of whether anyone asked for a measurement.
        self.debug_publisher.publish(
            self.bridge.cv2_to_imgmsg(detector.draw(image, result), "bgr8")
        )

        if self.pending is None:
            return

        if fresh:
            self.samples.append(result)

        wanted = int(self.get_parameter("snapshot_frames").value)
        if len(self.samples) >= wanted:
            self.answer()
        elif time.monotonic() > self.pending["deadline"]:
            self.answer(timed_out=True)

    def save_rejected(self, seq, reason):
        directory = str(self.get_parameter("save_rejected_dir").value)
        if not directory or self.latest_image is None:
            return
        try:
            import cv2

            out = Path(directory)
            out.mkdir(parents=True, exist_ok=True)
            stem = f"seq{seq:03d}"
            cv2.imwrite(str(out / f"{stem}_raw.png"), self.latest_image)
            cv2.imwrite(
                str(out / f"{stem}_mask.png"),
                detector.blue_mask(self.latest_image),
            )
            self.get_logger().info(
                "wrote rejected frame to %s/%s_raw.png (%s)"
                % (directory, stem, reason)
            )
        except Exception as error:            # diagnostics must never break a run
            self.get_logger().warning("could not save rejected frame: %s" % error)

    def answer(self, timed_out=False):
        """Reply with the median bearing of the accepted samples."""
        accepted = [s for s in self.samples if s.ok]
        request = self.pending
        self.pending = None

        # Presence and measurability are answered separately: a Dolly seen on
        # most frames but never cleanly measurable still means it is safe to
        # drive in, which is the question the controller actually has to answer
        # before committing.
        seen = sum(1 for s in self.samples if s.present)
        present = bool(self.samples) and seen * 2 >= len(self.samples)

        if not accepted:
            reasons = [s.reason for s in self.samples] or ["no frames"]
            reason = max(set(reasons), key=reasons.count)
            self.save_rejected(request["seq"], reason)
            payload = {
                "seq": request["seq"],
                "ok": False,
                "present": present,
                "seen": seen,
                "of": len(self.samples),
                "reason": f"{reason} ({len(self.samples)} frames)"
                          + (" [timeout]" if timed_out else ""),
            }
            if present:
                self.get_logger().info(
                    "SNAPSHOT seq %d: DOLLY CONFIRMED on %d/%d frames, "
                    "bearing not usable (%s)"
                    % (request["seq"], seen, len(self.samples), reason)
                )
            else:
                self.get_logger().warning(
                    "SNAPSHOT seq %d: NO DOLLY - %s"
                    % (request["seq"], payload["reason"])
                )
        else:
            # Median, not mean: one bad sample among five should not drag the
            # answer, and with the robot stopped the spread is small anyway.
            bearing = float(np.median([s.bearing_deg for s in accepted]))
            area = float(np.median([s.area_fraction for s in accepted]))
            spread = float(
                np.max([s.bearing_deg for s in accepted])
                - np.min([s.bearing_deg for s in accepted])
            ) if len(accepted) > 1 else 0.0
            payload = {
                "seq": request["seq"],
                "ok": True,
                "present": True,
                "bearing_deg": bearing,
                "area_fraction": area,
                "samples": len(accepted),
                "of": len(self.samples),
                "spread_deg": spread,
            }
            self.get_logger().info(
                "SNAPSHOT seq %d: DOLLY bearing=%+.2f deg area=%.1f%% "
                "(%d/%d frames, spread %.2f deg)"
                % (request["seq"], bearing, area * 100.0,
                   len(accepted), len(self.samples), spread)
            )

        message = String()
        message.data = json.dumps(payload)
        # Best-effort transport, one-shot event: repeat so a single drop cannot
        # cost the whole measurement.
        for _ in range(5):
            self.result_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = DollySnapshotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
