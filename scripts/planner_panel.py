#!/usr/bin/env python3
"""Publish the fleet plan comparison as an image, for rqt_image_view.

    ./.venv_vision/bin/python scripts/planner_panel.py
    ros2 run rqt_image_view rqt_image_view /planner/comparison

Why an image and not a log line
-------------------------------
"cuOpt solved this in 686 ms" is only convincing next to what the alternatives
produced. The bridge already prints that comparison, but a terminal scrollback
is not something an audience reads, and the evaluation asks for the integration
to be visible. Drawing it into a ROS image topic puts it in the same rqt window
as the camera feed, so the recognition and the planning are on screen together.

The numbers are read from factory_inventory.json, which the bridge writes on
startup, rather than recomputed here. Re-solving would risk showing a different
answer from the one the robots are actually executing.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

REPO = Path(__file__).resolve().parents[1]
INVENTORY = REPO / "simulation" / "isaac_sim" / "factory_inventory.json"

WIDTH, HEIGHT = 900, 520
BACKGROUND = (28, 28, 30)
TEXT = (235, 235, 235)
DIM = (150, 150, 150)
HIGHLIGHT = (90, 220, 90)
BAR_OTHER = (110, 110, 115)

# Matches the AMR colours the bridge assigns, so a robot in the viewport and its
# row in the panel are the same colour.
FALLBACK_COLOURS = [(90, 120, 240), (90, 220, 120), (240, 180, 90)]


def put(canvas, text, origin, scale=0.5, colour=TEXT, thickness=1):
    cv2.putText(
        canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, colour,
        thickness, cv2.LINE_AA,
    )


class PlannerPanel(Node):
    def __init__(self):
        super().__init__("planner_panel")
        self.declare_parameter("topic", "/planner/comparison")
        self.declare_parameter("inventory", str(INVENTORY))
        self.declare_parameter("period_sec", 1.0)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, str(self.get_parameter("topic").value), 1
        )
        self.create_timer(
            float(self.get_parameter("period_sec").value), self.tick
        )
        self.warned = False
        self.get_logger().info(
            "publishing plan comparison on %s"
            % str(self.get_parameter("topic").value)
        )

    def load(self):
        path = Path(str(self.get_parameter("inventory").value))
        if not path.exists():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        candidates = data.get("plan_candidates")
        if not candidates:
            return None
        return candidates, data.get("amrs", [])

    def tick(self):
        loaded = self.load()
        if loaded is None:
            if not self.warned:
                self.warned = True
                self.get_logger().warning(
                    "no 'plan_candidates' in the inventory yet; start the "
                    "bridge so it writes the fleet plan"
                )
            return
        self.warned = False
        candidates, amrs = loaded
        self.publisher.publish(
            self.bridge.cv2_to_imgmsg(self.render(candidates, amrs), "bgr8")
        )

    def render(self, candidates, amrs):
        canvas = np.full((HEIGHT, WIDTH, 3), BACKGROUND, dtype=np.uint8)
        chosen = next(
            (c for c in candidates if c["label"] == "selected"), candidates[-1]
        )

        put(canvas, "FLEET TASK ASSIGNMENT", (24, 40), 0.85, TEXT, 2)
        put(
            canvas,
            f"solver: {chosen['solver']}    "
            f"solve time: {chosen['solve_seconds'] * 1000.0:.1f} ms",
            (24, 70), 0.6, HIGHLIGHT, 1,
        )

        # ---- comparison bars. Makespan is the number that matters for a
        # fleet: it is when the last robot finishes, not how far they drove.
        top = 110
        put(canvas, "makespan (m)  -  lower is better", (24, top - 12), 0.5, DIM)
        widest = max(c["makespan_m"] for c in candidates) or 1.0
        for row, candidate in enumerate(candidates):
            y = top + row * 42
            selected = candidate is chosen
            colour = HIGHLIGHT if selected else BAR_OTHER
            length = int(candidate["makespan_m"] / widest * 520)
            cv2.rectangle(canvas, (150, y), (150 + length, y + 26), colour, -1)
            # Name the solver, not the internal label. "selected" told a
            # viewer nothing about what did the selecting, and drawing it bold
            # at this size smeared the glyphs together.
            name = candidate["solver"] if selected else candidate["label"]
            put(
                canvas, name, (24, y + 19), 0.5,
                HIGHLIGHT if selected else DIM, 1,
            )
            put(
                canvas,
                f"{candidate['makespan_m']:.1f} m   "
                f"total {candidate['total_m']:.1f} m",
                (160 + length, y + 19), 0.5,
                TEXT if selected else DIM,
            )

        baseline = next(
            (c for c in candidates if c["label"] == "manual"), None
        )
        if baseline and baseline["makespan_m"] > 0:
            gain = 100.0 * (1.0 - chosen["makespan_m"] / baseline["makespan_m"])
            distance_gain = 100.0 * (
                1.0 - chosen["total_m"] / baseline["total_m"]
            )
            put(
                canvas,
                f"vs manual:  makespan {gain:+.1f}%    "
                f"distance {distance_gain:+.1f}%",
                (24, top + len(candidates) * 42 + 22), 0.6, HIGHLIGHT, 2,
            )

        # ---- per robot assignment
        base = top + len(candidates) * 42 + 60
        put(canvas, "assignment", (24, base), 0.5, DIM)
        colours = {}
        for index, robot in enumerate(amrs):
            raw = robot.get("colour")
            if raw and len(raw) >= 3:
                # Inventory stores 0..1 RGB; OpenCV wants 0..255 BGR.
                colours[robot["name"]] = (
                    int(raw[2] * 255), int(raw[1] * 255), int(raw[0] * 255)
                )
            else:
                colours[robot["name"]] = FALLBACK_COLOURS[
                    index % len(FALLBACK_COLOURS)
                ]

        costs = chosen.get("cost_m", {})
        for index, (name, task_ids) in enumerate(
            sorted(chosen["assignment"].items())
        ):
            y = base + 30 + index * 34
            colour = colours.get(name, FALLBACK_COLOURS[index % 3])
            cv2.rectangle(canvas, (24, y - 14), (44, y + 4), colour, -1)
            put(canvas, name, (54, y), 0.6, TEXT, 2)
            put(
                canvas,
                f"{len(task_ids)} tasks   {', '.join(task_ids) or '-'}",
                (130, y), 0.55, TEXT,
            )
            if name in costs:
                put(canvas, f"{costs[name]:.1f} m", (WIDTH - 120, y), 0.55, DIM)

        return canvas


def main():
    rclpy.init()
    node = PlannerPanel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
