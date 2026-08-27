#!/usr/bin/env python3
"""Draw the fleet plan on a map of the factory, for rqt_image_view.

    ./.venv_vision/bin/python scripts/planner_map.py
    ros2 run rqt_image_view rqt_image_view /planner/map

The bar chart says cuOpt produced a shorter makespan. It does not show what
cuOpt decided. A viewer cannot tell from "107.2 m" whether the assignment is
sensible, whether the robots cover the factory or crowd one aisle, or which
robot goes where - and those are the things an audience can judge at a glance
if they are drawn.

Everything here comes out of factory_inventory.json, which the bridge writes on
startup: node positions, the corridor edges, each robot's colour, and the route
its missions actually follow. Nothing is recomputed, so the picture cannot
disagree with what the robots are doing.
"""
import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

REPO = Path(__file__).resolve().parents[1]
INVENTORY = REPO / "simulation" / "isaac_sim" / "factory_inventory.json"

WIDTH, HEIGHT = 1000, 620
MARGIN = 70
BACKGROUND = (26, 26, 28)
CORRIDOR = (70, 70, 76)
NODE = (150, 150, 158)
TEXT = (235, 235, 235)
DIM = (140, 140, 145)
# Dolly markers, in BGR. Deliberately not the deck's own blue: drawn in blue
# they were indistinguishable from amr3's route colour, and a viewer counting
# robots would have counted cargo. Amber reads as "waiting to be collected".
DOLLY = (60, 170, 235)

FALLBACK_COLOURS = [(90, 120, 240), (90, 220, 120), (240, 180, 90)]


def put(canvas, text, origin, scale=0.45, colour=TEXT, thickness=1):
    cv2.putText(
        canvas, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, colour,
        thickness, cv2.LINE_AA,
    )


class PlannerMap(Node):
    def __init__(self):
        super().__init__("planner_map")
        self.declare_parameter("topic", "/planner/map")
        self.declare_parameter("inventory", str(INVENTORY))
        self.declare_parameter("period_sec", 2.0)
        # Draw the rejected plan beside the chosen one.
        self.declare_parameter("compare_topic", "/planner/map_compare")

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, str(self.get_parameter("topic").value), 1
        )

        # Live robot positions. Odometry is reported relative to where each
        # robot spawned, not in factory coordinates, so the spawn pose has to
        # be applied - the same conversion capture_frames.py does. Skipping it
        # once put a robot fourteen metres from where it actually was.
        self.poses = {}
        self.subscriptions_by_robot = {}
        self.subscribe_to_robots()
        self.create_timer(
            float(self.get_parameter("period_sec").value), self.tick
        )
        self.compare_publisher = self.create_publisher(
            Image, str(self.get_parameter("compare_topic").value), 1
        )
        self.warned = False
        self.get_logger().info(
            "publishing plan map on %s"
            % str(self.get_parameter("topic").value)
        )

    def subscribe_to_robots(self):
        path = Path(str(self.get_parameter("inventory").value))
        if not path.exists():
            return
        data = json.loads(path.read_text(encoding="utf-8"))
        for robot in data.get("amrs", []):
            name = robot["name"]
            if name in self.subscriptions_by_robot:
                continue
            topic = (robot.get("topics") or {}).get("odom")
            if not topic:
                continue
            start = robot.get("amr_start") or {}
            origin = (
                float(start.get("x", 0.0)),
                float(start.get("y", 0.0)),
                math.radians(float(start.get("yaw_deg", 0.0))),
            )
            self.subscriptions_by_robot[name] = self.create_subscription(
                Odometry, topic,
                lambda msg, n=name, o=origin: self.on_odom(msg, n, o),
                1,
            )
            self.get_logger().info("tracking %s on %s" % (name, topic))

    def on_odom(self, msg, name, origin):
        start_x, start_y, start_yaw = origin
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        cos_yaw, sin_yaw = math.cos(start_yaw), math.sin(start_yaw)
        yaw = start_yaw + math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.poses[name] = (
            start_x + cos_yaw * p.x - sin_yaw * p.y,
            start_y + sin_yaw * p.x + cos_yaw * p.y,
            yaw,
        )

    def tick(self):
        self.subscribe_to_robots()
        path = Path(str(self.get_parameter("inventory").value))
        if not path.exists():
            return
        data = json.loads(path.read_text(encoding="utf-8"))
        if not data.get("nodes") or not data.get("amrs"):
            if not self.warned:
                self.warned = True
                self.get_logger().warning(
                    "inventory has no nodes or amrs yet; start the bridge"
                )
            return
        self.warned = False
        self.publisher.publish(
            self.bridge.cv2_to_imgmsg(self.render(data), "bgr8")
        )
        comparison = self.render_comparison(data)
        if comparison is not None:
            self.compare_publisher.publish(
                self.bridge.cv2_to_imgmsg(comparison, "bgr8")
            )

    # ------------------------------------------------------------ drawing

    def make_projection(self, nodes, dollies):
        """World metres to pixels, preserving aspect ratio.

        Distorting the factory to fill the canvas would make a balanced plan
        look lopsided, so the scale is the same on both axes and the shorter
        one is centred.
        """
        xs = [p[0] for p in nodes.values()] + [d[0] for d in dollies]
        ys = [p[1] for p in nodes.values()] + [d[1] for d in dollies]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        span_x = max(1e-6, max_x - min_x)
        span_y = max(1e-6, max_y - min_y)
        scale = min(
            (WIDTH - 2 * MARGIN) / span_x, (HEIGHT - 2 * MARGIN - 60) / span_y
        )
        offset_x = (WIDTH - span_x * scale) / 2.0 - min_x * scale
        offset_y = (HEIGHT - 60 - span_y * scale) / 2.0 - min_y * scale + 50

        def project(x, y):
            # Flip y so north in the factory is up on screen.
            return (
                int(round(x * scale + offset_x)),
                int(round(HEIGHT - (y * scale + offset_y))),
            )

        return project

    def render(self, data, heading=None, live=True):
        canvas = np.full((HEIGHT, WIDTH, 3), BACKGROUND, dtype=np.uint8)

        nodes = {
            int(n["id"]): (float(n["x"]), float(n["y"]))
            for n in data["nodes"]
            if n.get("id") is not None
        }
        dollies = [
            (float(d["x"]), float(d["y"])) for d in data.get("dollies", [])
        ]
        project = self.make_projection(nodes, dollies)

        # ---- corridors first, so routes draw over them
        for edge in data.get("edges", []):
            a, b = int(edge[0]), int(edge[1])
            if a in nodes and b in nodes:
                cv2.line(
                    canvas, project(*nodes[a]), project(*nodes[b]),
                    CORRIDOR, 2, cv2.LINE_AA,
                )

        # ---- dollies waiting to be collected
        for x, y in dollies:
            cv2.circle(canvas, project(x, y), 6, DOLLY, -1, cv2.LINE_AA)

        # ---- one coloured path per robot, per mission
        legend = []
        for index, robot in enumerate(data["amrs"]):
            raw = robot.get("colour")
            if raw and len(raw) >= 3:
                colour = (
                    int(raw[2] * 255), int(raw[1] * 255), int(raw[0] * 255)
                )
            else:
                colour = FALLBACK_COLOURS[index % len(FALLBACK_COLOURS)]

            task_ids = []
            for mission in robot.get("missions", []):
                task_ids.append(mission.get("task_id", "?"))
                route = [
                    int(n) for n in mission.get("mission_route", [])
                    if int(n) in nodes
                ]
                # Offset each robot's line slightly so shared corridors show
                # all of them rather than whichever drew last.
                shift = (index - 1) * 4
                points = []
                for node_id in route:
                    px, py = project(*nodes[node_id])
                    points.append((px + shift, py + shift))
                for start, end in zip(points, points[1:]):
                    cv2.line(canvas, start, end, colour, 3, cv2.LINE_AA)
                if points:
                    cv2.circle(canvas, points[0], 7, colour, 2, cv2.LINE_AA)
                    cv2.drawMarker(
                        canvas, points[-1], colour, cv2.MARKER_TILTED_CROSS,
                        14, 2, cv2.LINE_AA,
                    )

            start = robot.get("amr_start") or {}
            if "x" in start:
                home = project(float(start["x"]), float(start["y"]))
                cv2.rectangle(
                    canvas,
                    (home[0] - 7, home[1] - 7), (home[0] + 7, home[1] + 7),
                    colour, -1,
                )
            legend.append((robot["name"], colour, task_ids))

        # ---- live robot positions, drawn over the routes they are following
        for index, robot in enumerate(data["amrs"]):
            pose = self.poses.get(robot["name"]) if live else None
            if pose is None:
                continue
            raw = robot.get("colour")
            colour = (
                (int(raw[2] * 255), int(raw[1] * 255), int(raw[0] * 255))
                if raw and len(raw) >= 3
                else FALLBACK_COLOURS[index % len(FALLBACK_COLOURS)]
            )
            x, y, yaw = pose
            centre = project(x, y)
            cv2.circle(canvas, centre, 11, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(canvas, centre, 9, colour, -1, cv2.LINE_AA)
            # A heading tick, so a robot turning in place is visibly doing
            # something rather than looking stalled.
            nose = (
                int(centre[0] + 17 * math.cos(yaw)),
                int(centre[1] - 17 * math.sin(yaw)),
            )
            cv2.line(canvas, centre, nose, (255, 255, 255), 2, cv2.LINE_AA)

        # ---- node dots and numbers last, so they stay readable
        for node_id, (x, y) in nodes.items():
            px, py = project(x, y)
            cv2.circle(canvas, (px, py), 3, NODE, -1, cv2.LINE_AA)
            put(canvas, str(node_id), (px + 6, py - 6), 0.38, DIM)

        # ---- header and legend
        cv2.rectangle(canvas, (0, 0), (WIDTH, 44), (0, 0, 0), -1)
        plan = data.get("fleet_plan") or {}
        if heading is not None:
            put(canvas, heading, (16, 30), 0.8, TEXT, 2)
        else:
            put(canvas, "FLEET PLAN", (16, 30), 0.75, TEXT, 2)
            put(
                canvas,
                f"solver {plan.get('solver', '?')}   "
                f"makespan {plan.get('makespan_m', 0.0):.1f} m   "
                f"solve {plan.get('solve_seconds', 0.0) * 1000.0:.1f} ms",
                (190, 29), 0.55, (90, 220, 90), 1,
            )

        x = 16
        for name, colour, task_ids in legend:
            cv2.rectangle(
                canvas, (x, HEIGHT - 26), (x + 18, HEIGHT - 10), colour, -1
            )
            label = f"{name}  {','.join(task_ids) or '-'}"
            put(canvas, label, (x + 26, HEIGHT - 13), 0.5, TEXT)
            # Advance by the rendered width rather than a per-character guess,
            # which ran the last entry off the canvas.
            (text_w, _), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            x += 46 + text_w

        put(
            canvas,
            "circle = pickup    cross = dropoff    square = spawn    "
            "orange = Dolly",
            (16, HEIGHT - 38), 0.42, DIM,
        )
        return canvas


    def render_comparison(self, data):
        """The naive assignment and cuOpt's, side by side on the same map.

        The strongest argument for the solver is not a number. Drawn together,
        the manual plan's routes are long and cross the factory while cuOpt's
        are compact and separated, and that reads without anyone parsing
        "180.2 m" against "107.2 m". Both panes use the same projection so a
        difference in the picture is a difference in the plan.
        """
        candidates = data.get("plan_candidates") or []
        manual = next((c for c in candidates if c["label"] == "manual"), None)
        chosen = next((c for c in candidates if c["label"] == "selected"), None)
        if manual is None or chosen is None:
            return None

        panes = []
        for candidate in (manual, chosen):
            # Re-key the robots onto this candidate's assignment. Mission
            # routes come from the plan the bridge is executing, so the other
            # candidate's routes are rebuilt from its task list instead.
            pane = self.render(
                self.reassign(data, candidate["assignment"]),
                heading=(
                    f"{candidate['solver']}   "
                    f"makespan {candidate['makespan_m']:.1f} m"
                ),
                live=False,
            )
            panes.append(cv2.resize(pane, (WIDTH // 2, HEIGHT // 2 + 60)))

        canvas = np.hstack(panes)
        cv2.line(
            canvas, (WIDTH // 2, 0), (WIDTH // 2, canvas.shape[0]),
            (70, 70, 76), 2,
        )
        return canvas

    @staticmethod
    def reassign(data, assignment):
        """A copy of the inventory with each robot given another plan's tasks."""
        by_task = {
            mission["task_id"]: mission
            for robot in data["amrs"]
            for mission in robot.get("missions", [])
        }
        clone = dict(data)
        clone["amrs"] = [
            dict(
                robot,
                missions=[
                    by_task[task_id]
                    for task_id in assignment.get(robot["name"], [])
                    if task_id in by_task
                ],
            )
            for robot in data["amrs"]
        ]
        return clone


def main():
    rclpy.init()
    node = PlannerMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


main()
