#!/usr/bin/env python3
"""World-coordinate transport controller for one IW Hub.

Reads the plan PC1 wrote into factory_inventory.json and drives a single AMR
through its list of missions back to back:

    approach waypoints -> freeze Dolly -> pre-dock -> final dock -> lift up
    -> attach -> carry waypoints -> lift down -> detach -> undock -> next

Docking uses world coordinates only; the Dolly handling itself happens on PC1
and is triggered over the /dolly_cmd topic so no shared filesystem is needed.
"""

import json
import math
import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def _default_inventory():
    """Locate factory_inventory.json without hard-coding anyone's home directory.

    Order: FACTORY_INVENTORY env var, then a repository checkout found by walking
    up from this file, then the current working directory.
    """
    override = os.environ.get("FACTORY_INVENTORY")
    if override:
        return override

    here = Path(__file__).resolve()
    relative = Path("simulation/isaac_sim/factory_inventory.json")
    for parent in here.parents:
        candidate = parent / relative
        if candidate.is_file():
            return str(candidate)
    return str(Path.cwd() / relative)


DEFAULT_INVENTORY = _default_inventory()


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def edge_key(a, b):
    """Stable text form of an undirected graph edge."""
    lo, hi = sorted((int(a), int(b)))
    return f"{lo}-{hi}"


class AMRMissionController(Node):
    # Must match the codes in standalone_factory_bridge.py on PC1.
    DOLLY_CMD_FREEZE = 1
    DOLLY_CMD_ATTACH = 2
    DOLLY_CMD_LIFT = 3
    DOLLY_CMD_LOWER = 4
    DOLLY_CMD_RELEASE = 5

    def __init__(self):
        super().__init__("amr_mission_controller")

        self.declare_parameter("inventory", str(DEFAULT_INVENTORY))
        # Which robot in inventory["amrs"] this controller drives.
        self.declare_parameter("amr", "amr1")
        self.declare_parameter("position_tolerance", 0.25)
        self.declare_parameter("dock_tolerance", 0.12)
        self.declare_parameter("yaw_tolerance_rad", 0.08)
        self.declare_parameter("timeout_sec", 300.0)
        self.declare_parameter("max_linear_speed", 4.05)
        self.declare_parameter("max_angular_speed", 6.30)
        self.declare_parameter("dock_linear_speed", 0.36)
        self.declare_parameter("dock_angular_speed", 0.75)
        self.declare_parameter("lift_down_dwell_sec", 2.0)
        # Short reverse: some drop nodes sit in tight corners.
        self.declare_parameter("undock_distance", 1.0)
        self.declare_parameter("undock_speed", 0.5)
        self.declare_parameter("lift_ramp_sec", 1.2)
        self.declare_parameter("pickup_lift_position", 0.04)
        # Delay before this robot starts, used to stagger AMRs that must share
        # an aisle because the waypoint graph offers no disjoint alternative.
        self.declare_parameter("start_delay_sec", 0.0)
        # Runtime arbitration for aisles two robots must share.
        self.declare_parameter("traffic_topic", "/traffic/claims")
        self.declare_parameter("traffic_timeout_sec", 60.0)
        self.declare_parameter("claim_stale_sec", 2.0)

        inventory_path = Path(self.get_parameter("inventory").value)
        with inventory_path.open(encoding="utf-8") as stream:
            inventory = json.load(stream)

        amr_name = str(self.get_parameter("amr").value)
        robots = {r["name"]: r for r in inventory.get("amrs", [])}
        if amr_name not in robots:
            raise RuntimeError(
                "amr '%s' not in inventory (available: %s). Start the PC1 bridge "
                "first so it regenerates factory_inventory.json."
                % (amr_name, sorted(robots))
            )
        robot = robots[amr_name]

        self.amr_name = amr_name
        self.topics = robot["topics"]
        self.missions = robot["missions"]
        self.mission_index = 0

        self.start_x = float(robot["amr_start"]["x"])
        self.start_y = float(robot["amr_start"]["y"])
        self.start_yaw = math.radians(float(robot["amr_start"]["yaw_deg"]))
        self.nodes_by_id = {int(item["id"]): item for item in inventory["nodes"]}

        self.tolerance = float(self.get_parameter("position_tolerance").value)
        self.dock_tolerance = float(self.get_parameter("dock_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance_rad").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.dock_linear_speed = float(self.get_parameter("dock_linear_speed").value)
        self.dock_angular_speed = float(self.get_parameter("dock_angular_speed").value)
        self.lift_down_dwell_sec = float(
            self.get_parameter("lift_down_dwell_sec").value
        )
        self.undock_distance = float(self.get_parameter("undock_distance").value)
        self.undock_speed = float(self.get_parameter("undock_speed").value)
        self.lift_ramp_sec = float(self.get_parameter("lift_ramp_sec").value)
        self.pickup_lift_position = float(
            self.get_parameter("pickup_lift_position").value
        )
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.traffic_timeout_sec = float(
            self.get_parameter("traffic_timeout_sec").value
        )
        self.claim_stale_sec = float(self.get_parameter("claim_stale_sec").value)
        self.locked_edges = set(inventory.get("shared_edges", []))

        self.publisher = self.create_publisher(Twist, self.topics["cmd_vel"], 10)
        self.subscription = self.create_subscription(
            Odometry, self.topics["odom"], self.odom_callback, 10
        )
        self.lift_publisher = self.create_publisher(
            JointState, self.topics["lift_cmd"], 10
        )
        self.lift_state_subscription = self.create_subscription(
            JointState, self.topics["lift_state"], self.lift_state_callback, 10
        )
        self.dolly_cmd_publisher = self.create_publisher(
            JointState, self.topics["dolly_cmd"], 10
        )
        traffic_topic = str(self.get_parameter("traffic_topic").value)
        self.traffic_publisher = self.create_publisher(String, traffic_topic, 10)
        self.traffic_subscription = self.create_subscription(
            String, traffic_topic, self.traffic_callback, 10
        )
        self.timer = self.create_timer(0.05, self.control_loop)

        self.odom_pose = None
        self.lift_position = None
        self.dolly_cmd_seq = 0
        self.state = "WAIT_ODOM"
        self.started_at = None
        self.release_at = None
        self.last_log_at = 0.0
        self.phase_start = 0.0
        self.phase_deadline = None
        self.lift_cmd_sent = True
        self.undock_from = (0.0, 0.0)

        # Measured metrics for the planner comparison.
        self.travelled_m = 0.0
        self.last_metric_pose = None
        self.run_started_at = None
        self.mission_started_at = None
        self.mission_stats = []

        # Waypoint queue for the current leg.
        self.waypoints = []
        self.waypoint_index = 0
        self.leg = "APPROACH"
        self.target_x = self.start_x
        self.target_y = self.start_y
        self.target_yaw = None
        self.reached = True
        self.dock_info = None

        # Traffic state.
        self.other_claims = {}
        self.held_edge = None
        self.wanted_edge = None
        self.pending_edge = None
        self.wait_started_at = None
        self.last_node = None

        self.get_logger().info(
            "AMR %s | %d mission(s)" % (amr_name, len(self.missions))
        )
        for mission in self.missions:
            self.get_logger().info(
                "  mission%d Node_%d -> Node_%d | approach %s | carry %s | dolly %s"
                % (
                    mission["index"] + 1,
                    mission["start_node"],
                    mission["goal_node"],
                    " -> ".join(f"Node_{n}" for n in mission["approach_route"]),
                    " -> ".join(f"Node_{n}" for n in mission["mission_route"]),
                    mission["dolly_path"],
                )
            )
        self.get_logger().info(
            "topics cmd_vel=%s odom=%s lift_cmd=%s lift_state=%s dolly_cmd=%s"
            % (
                self.topics["cmd_vel"],
                self.topics["odom"],
                self.topics["lift_cmd"],
                self.topics["lift_state"],
                self.topics["dolly_cmd"],
            )
        )
        if self.start_delay_sec > 0.0:
            self.get_logger().info(
                "start delayed by %.1f s" % self.start_delay_sec
            )
        self.get_logger().info(
            "traffic-locked edges: %s"
            % (sorted(self.locked_edges) if self.locked_edges else "none")
        )

    # ---------------------------------------------------------------- inputs

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            odom_yaw,
        )
        if self.release_at is None:
            self.release_at = time.monotonic() + self.start_delay_sec

        pose = self.world_pose()
        if self.last_metric_pose is not None and self.run_started_at is not None:
            self.travelled_m += math.hypot(
                pose[0] - self.last_metric_pose[0],
                pose[1] - self.last_metric_pose[1],
            )
        self.last_metric_pose = pose

    def lift_state_callback(self, msg):
        if not msg.position:
            return
        if msg.name and "lift_joint" in msg.name:
            self.lift_position = float(msg.position[msg.name.index("lift_joint")])
        else:
            self.lift_position = float(msg.position[0])

    def traffic_callback(self, msg):
        try:
            claim = json.loads(msg.data)
        except ValueError:
            return
        name = claim.get("amr")
        if not name or name == self.amr_name:
            return
        claim["received_at"] = time.monotonic()
        self.other_claims[name] = claim

    def publish_claim(self):
        payload = {
            "amr": self.amr_name,
            "held": self.held_edge,
            "wanted": self.wanted_edge,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.traffic_publisher.publish(msg)

    def edge_blocker(self, now, key):
        """Return the name of a robot that outranks us for this edge."""
        for name, claim in self.other_claims.items():
            if now - claim.get("received_at", 0.0) > self.claim_stale_sec:
                continue
            if claim.get("held") == key:
                return name
            # Both want it: the lexicographically smaller name goes first.
            if claim.get("wanted") == key and name < self.amr_name:
                return name
        return None

    def acquire_edge(self, now, key):
        """Reserve a shared aisle before entering it."""
        self.wanted_edge = key
        blocker = self.edge_blocker(now, key)
        if blocker is None:
            self.held_edge = key
            self.wanted_edge = None
            if self.wait_started_at is not None:
                self.get_logger().info(
                    "TRAFFIC clear on %s after %.1f s"
                    % (key, now - self.wait_started_at)
                )
                self.wait_started_at = None
            return True

        if self.wait_started_at is None:
            self.wait_started_at = now
            self.get_logger().info(
                "TRAFFIC waiting for %s to clear segment %s" % (blocker, key)
            )
        elif now - self.wait_started_at > self.traffic_timeout_sec:
            self.get_logger().warning(
                "TRAFFIC timeout after %.1f s on %s, proceeding"
                % (now - self.wait_started_at, key)
            )
            self.held_edge = key
            self.wanted_edge = None
            self.wait_started_at = None
            return True
        return False

    def release_edge(self):
        self.held_edge = None
        self.wanted_edge = None
        self.wait_started_at = None

    # --------------------------------------------------------------- helpers

    def world_pose(self):
        ox, oy, oyaw = self.odom_pose
        c = math.cos(self.start_yaw)
        s = math.sin(self.start_yaw)
        return (
            self.start_x + c * ox - s * oy,
            self.start_y + s * ox + c * oy,
            normalize_angle(self.start_yaw + oyaw),
        )

    def publish_stop(self):
        self.publisher.publish(Twist())

    def publish_lift(self, position):
        lift = JointState()
        lift.name = ["lift_joint"]
        lift.position = [position]
        self.lift_publisher.publish(lift)

    def send_dolly_command(self, code, label):
        self.dolly_cmd_seq += 1
        msg = JointState()
        # ROS2SubscribeJointState drops the message unless name and position
        # have equal length, so both fields carry two entries.
        msg.name = ["dolly_cmd", "dolly_seq"]
        msg.position = [float(code), float(self.dolly_cmd_seq)]
        # Repeat: this is a one-shot event on a best-effort link.
        for _ in range(5):
            self.dolly_cmd_publisher.publish(msg)
        self.get_logger().info(
            "DOLLY CMD %s (code=%d seq=%d)" % (label, code, self.dolly_cmd_seq)
        )

    def enter_phase(self, now, duration):
        self.phase_start = now
        self.phase_deadline = now + duration

    def aim_at(self, x, y, yaw_deg, state):
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_yaw = None if yaw_deg is None else math.radians(float(yaw_deg))
        self.reached = False
        self.started_at = time.monotonic()
        self.state = state

    def aim_at_waypoint(self, index):
        self.waypoint_index = index
        target_node = self.waypoints[index]
        node = self.nodes_by_id[target_node]
        self.release_edge()
        self.pending_edge = None
        if self.last_node is not None:
            key = edge_key(self.last_node, target_node)
            if key in self.locked_edges:
                self.pending_edge = key
        self.aim_at(
            node["x"],
            node["y"],
            None,
            "APPROACH" if self.leg == "APPROACH" else "CARRY",
        )

    def begin_mission(self, index):
        mission = self.missions[index]
        self.mission_index = index
        self.dock_info = mission["mission_dock"]
        self.last_node = (
            self.missions[index - 1]["goal_node"] if index else None
        )
        self.leg = "APPROACH"
        self.mission_started_at = time.monotonic()
        self.mission_travelled_at_start = self.travelled_m
        self.waypoints = [int(n) for n in mission["approach_route"]]
        self.get_logger().info(
            "MISSION %d START | Node_%d -> Node_%d"
            % (index + 1, mission["start_node"], mission["goal_node"])
        )
        if self.waypoints:
            self.aim_at_waypoint(0)
        else:
            self.begin_docking()

    def begin_docking(self):
        self.send_dolly_command(self.DOLLY_CMD_FREEZE, "FREEZE")
        pre = self.dock_info["pre_dock"]
        self.aim_at(pre["x"], pre["y"], pre["yaw_deg"], "GO_TO_PRE_DOCK")

    def current_mission(self):
        return self.missions[self.mission_index]

    # ------------------------------------------------------------ main cycle

    def control_loop(self):
        now = time.monotonic()

        if self.state == "WAIT_ODOM":
            self.publish_stop()
            if self.odom_pose is not None and now >= self.release_at:
                self.run_started_at = now
                self.begin_mission(0)
            return

        if self.state == "LIFT_UP":
            self.publish_stop()
            if not self.lift_cmd_sent and now >= self.phase_start + 0.4:
                self.send_dolly_command(self.DOLLY_CMD_LIFT, "LIFT")
                self.lift_cmd_sent = True
                self.enter_phase(now, self.lift_ramp_sec + 6.0)
            if not self.lift_cmd_sent:
                return
            progress = min(1.0, (now - self.phase_start) / self.lift_ramp_sec)
            self.publish_lift(self.pickup_lift_position * progress)
            done = (
                progress >= 1.0
                and self.lift_position is not None
                and self.lift_position >= 0.035
            )
            if done or now >= self.phase_deadline:
                self.get_logger().info(
                    "LIFT_UP done, lift_joint_state=%s (timeout=%s)"
                    % (self.lift_position, not done)
                )
                self.state = "ATTACH_DOLLY"
                self.enter_phase(now, 1.0)
            return

        if self.state == "ATTACH_DOLLY":
            self.publish_stop()
            if now >= self.phase_deadline:
                self.get_logger().info("ATTACH_DOLLY done, follower active")
                self.leg = "CARRY"
                self.last_node = self.current_mission()["start_node"]
                self.waypoints = [
                    int(n) for n in self.current_mission()["mission_route"]
                ]
                # First entry is the node we are already docked at.
                self.aim_at_waypoint(1 if len(self.waypoints) > 1 else 0)
            return

        if self.state == "LIFT_DOWN":
            self.publish_stop()
            progress = min(1.0, (now - self.phase_start) / self.lift_ramp_sec)
            self.publish_lift(self.pickup_lift_position * (1.0 - progress))
            down = (
                progress >= 1.0
                and self.lift_position is not None
                and self.lift_position <= 0.005
            )
            settled = now >= self.phase_start + self.lift_down_dwell_sec
            if (down and settled) or now >= self.phase_deadline:
                self.get_logger().info(
                    "LIFT_DOWN done, lift_joint_state=%s (timeout=%s)"
                    % (self.lift_position, not down)
                )
                self.send_dolly_command(self.DOLLY_CMD_RELEASE, "RELEASE")
                self.state = "DETACH_DOLLY"
                self.enter_phase(now, 1.0)
            return

        if self.state == "DETACH_DOLLY":
            self.publish_stop()
            if now >= self.phase_deadline:
                self.undock_from = self.world_pose()[:2]
                self.get_logger().info(
                    "DETACH_DOLLY done, Dolly left at Node%d"
                    % self.current_mission()["goal_node"]
                )
                self.state = "UNDOCK"
                self.enter_phase(now, self.undock_distance / self.undock_speed)
            return

        if self.state == "UNDOCK":
            x, y, _ = self.world_pose()
            backed = math.hypot(x - self.undock_from[0], y - self.undock_from[1])
            command = Twist()
            command.linear.x = -self.undock_speed
            self.publisher.publish(command)
            if backed >= self.undock_distance or now >= self.phase_deadline:
                self.publish_stop()
                mission = self.current_mission()
                elapsed = now - self.mission_started_at
                distance = self.travelled_m - self.mission_travelled_at_start
                self.mission_stats.append(
                    {
                        "task": mission.get("task_id", "?"),
                        "seconds": elapsed,
                        "metres": distance,
                    }
                )
                self.get_logger().info(
                    "MISSION %d COMPLETE (%s) | %.1f s | %.1f m | backed out %.3f m"
                    % (
                        self.mission_index + 1,
                        mission.get("task_id", "?"),
                        elapsed,
                        distance,
                        backed,
                    )
                )
                if self.mission_index + 1 < len(self.missions):
                    self.begin_mission(self.mission_index + 1)
                else:
                    self.state = "DONE"
                    x, y, yaw = self.world_pose()
                    total_seconds = now - self.run_started_at
                    breakdown = " ".join(
                        "%s=%.0fs/%.0fm" % (m["task"], m["seconds"], m["metres"])
                        for m in self.mission_stats
                    )
                    self.get_logger().info(
                        "ALL MISSIONS DONE | %s final world=(%.3f, %.3f, %.1f deg)"
                        % (self.amr_name, x, y, math.degrees(yaw))
                    )
                    self.get_logger().info(
                        "METRICS %s | total %.1f s | travelled %.1f m | %s"
                        % (self.amr_name, total_seconds, self.travelled_m, breakdown)
                    )
            return

        if self.state == "DONE":
            self.publish_stop()
            return

        self.publish_claim()

        if self.odom_pose is None or self.reached:
            self.publish_stop()
            return

        if self.pending_edge is not None:
            if not self.acquire_edge(now, self.pending_edge):
                self.publish_stop()
                return
            self.pending_edge = None

        if now - self.started_at > self.timeout_sec:
            self.publish_stop()
            self.reached = True
            self.state = "TIMEOUT"
            self.get_logger().error(
                "Waypoint timeout in %s, stopping safely" % self.state
            )
            return

        self.drive_towards_target(now)

    # ------------------------------------------------------------- steering

    def drive_towards_target(self, now):
        docking = self.state == "FINAL_DOCK"
        tolerance = self.dock_tolerance if docking else self.tolerance
        angular_limit = (
            self.dock_angular_speed if docking else self.max_angular_speed
        )
        linear_limit = self.dock_linear_speed if docking else self.max_linear_speed

        x, y, yaw = self.world_pose()
        dx = self.target_x - x
        dy = self.target_y - y
        distance = math.hypot(dx, dy)

        if distance <= tolerance and self.target_yaw is not None:
            yaw_error = normalize_angle(self.target_yaw - yaw)
            if abs(yaw_error) > self.yaw_tolerance:
                command = Twist()
                command.angular.z = max(
                    -angular_limit, min(angular_limit, 1.2 * yaw_error)
                )
                self.publisher.publish(command)
                return

        if distance <= tolerance:
            self.publish_stop()
            self.reached = True
            self.on_target_reached(now, x, y, yaw, distance)
            return

        desired_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_yaw - yaw)
        command = Twist()
        if abs(heading_error) > 0.18:
            command.angular.z = max(
                -angular_limit, min(angular_limit, 1.2 * heading_error)
            )
        else:
            command.linear.x = linear_limit
            command.angular.z = max(
                -angular_limit, min(angular_limit, 0.8 * heading_error)
            )
        self.publisher.publish(command)

        if now - self.last_log_at >= 1.0:
            self.last_log_at = now
            self.get_logger().info(
                "%s world=(%.3f, %.3f, %.3f) target=(%.3f, %.3f) "
                "distance=%.3f heading_error=%.3f"
                % (
                    self.state,
                    x,
                    y,
                    yaw,
                    self.target_x,
                    self.target_y,
                    distance,
                    heading_error,
                )
            )

    def on_target_reached(self, now, x, y, yaw, distance):
        state = self.state

        if state in ("APPROACH", "CARRY"):
            node_id = self.waypoints[self.waypoint_index]
            self.last_node = node_id
            self.release_edge()
            self.get_logger().info(
                "REACHED Node%d world=(%.3f, %.3f, %.1f deg), error=%.3f m"
                % (node_id, x, y, math.degrees(yaw), distance)
            )
            if self.waypoint_index + 1 < len(self.waypoints):
                self.aim_at_waypoint(self.waypoint_index + 1)
            elif state == "APPROACH":
                self.begin_docking()
            else:
                self.send_dolly_command(self.DOLLY_CMD_LOWER, "LOWER")
                self.state = "LIFT_DOWN"
                self.enter_phase(now, self.lift_ramp_sec + 6.0)
            return

        if state == "GO_TO_PRE_DOCK":
            self.get_logger().info(
                "REACHED PRE_DOCK world=(%.3f, %.3f, %.1f deg), error=%.3f m"
                % (x, y, math.degrees(yaw), distance)
            )
            dock = self.dock_info["dock"]
            self.aim_at(dock["x"], dock["y"], dock["yaw_deg"], "FINAL_DOCK")
            return

        if state == "FINAL_DOCK":
            dock = self.dock_info["dock"]
            yaw_error = math.degrees(
                normalize_angle(math.radians(dock["yaw_deg"]) - yaw)
            )
            self.get_logger().info(
                "DOCK OK | target=(%.3f, %.3f, %.2f deg) actual=(%.3f, %.3f, %.2f deg) "
                "position_error=%.3f m yaw_error=%.2f deg"
                % (
                    dock["x"],
                    dock["y"],
                    dock["yaw_deg"],
                    x,
                    y,
                    math.degrees(yaw),
                    distance,
                    yaw_error,
                )
            )
            self.send_dolly_command(self.DOLLY_CMD_ATTACH, "ATTACH")
            self.lift_cmd_sent = False
            self.state = "LIFT_UP"
            self.enter_phase(now, self.lift_ramp_sec + 6.0)
            return


def main(args=None):
    rclpy.init(args=args)
    node = AMRMissionController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
