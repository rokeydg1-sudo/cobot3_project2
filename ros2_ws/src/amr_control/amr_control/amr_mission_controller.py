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
        # Vision docking: which mission index (0-based) hands FINAL_DOCK over to
        # the YOLO/PnP node. -1 disables it. Coordinate docking always remains
        # the fallback, so a vision dropout cannot strand the robot.
        self.declare_parameter("vision_dock_mission", -1)
        # Use the camera on every mission this robot runs, not just one.
        #
        # One snapshot per run is not a sample. The detection rate in the run
        # report was 1/1, which is not evidence of anything; with three
        # missions on amr1 the same run yields three. Off by default because a
        # single docking camera exists and only the robot it is mounted on can
        # use it - a second subscriber would be measuring amr1's view.
        self.declare_parameter("vision_dock_all_missions", False)
        self.declare_parameter("vision_cmd_topic", "/dock/cmd_vel")
        self.declare_parameter("vision_status_topic", "/dock/status")
        self.declare_parameter("vision_timeout_sec", 1.0)

        # Snapshot docking: stop short of the Dolly, look once, then drive in.
        self.declare_parameter("snapshot_request_topic", "/dock/snapshot_request")
        self.declare_parameter("snapshot_result_topic", "/dock/snapshot_result")
        # Clear distance from the camera to the *near edge* of the Dolly, not
        # to the dock target. The dock target sits under the middle of the
        # Dolly, and the asset is about 3.2 m long, so measuring the standoff
        # from there left the leading edge roughly 1 m from the lens - close
        # enough to overflow the frame at 60 and at 80 degrees alike, and every
        # snapshot came back "clipped at frame edge". Measuring from the edge
        # is also what "stop 3 m before the Dolly" means to a person.
        self.declare_parameter("snapshot_standoff_m", 3.0)
        # Camera offset ahead of the chassis origin, matching CAMERA_FORWARD_M
        # in the bridge. The robot is commanded by its chassis pose but the
        # picture is taken from here.
        self.declare_parameter("camera_forward_m", 0.55)
        # Used only if the inventory predates the bounds being recorded.
        self.declare_parameter("dolly_half_length_fallback_m", 1.6)
        # How wide the deck really is across the approach, in metres.
        #
        # Measured off the asset by scripts/measure_dolly.py:
        # FOF_Mesh_Shelf_Cart_B_LOD0 is 1.242 x 0.865 x 0.257 m. An earlier
        # value of 3.29 came from fitting apparent width against a camera model
        # that was itself wrong by a factor of 2.6, and was wrong with it.
        # Set to 0 to fall back to the inventory bounding box.
        self.declare_parameter("dolly_deck_width_m", 1.242)
        self.declare_parameter("snapshot_timeout_sec", 6.0)
        # Hold still after the answer before driving in. The measurement itself
        # takes about half a second, which is too short to see: on the first
        # integrated run the robot appeared to drive straight through the
        # standoff without stopping. This dwell exists so the recognition is
        # visible - the overlay in rqt sits on the Dolly, unmoving, long enough
        # to read - and it costs nothing but the wait.
        self.declare_parameter("snapshot_dwell_sec", 1.4)
        # Distance over which to taper the approach speed to a target.
        self.declare_parameter("approach_ramp_m", 0.9)
        # About one degree. At the 5 m standoff a degree of heading moves
        # the Dolly 0.09 m across the frame, so this keeps it centred.
        self.declare_parameter("standoff_yaw_tolerance_rad", 0.022)
        # Turn shaping. See smooth_angular().
        # Reverted to the plain proportional term after the first attempt at
        # shaping the turn made it worse. A minimum speed floor of 0.13 rad/s
        # cannot settle inside a 0.022 rad tolerance: the robot overshoots, and
        # the acceleration limit then takes several ticks to reverse, so it
        # hunts instead of stopping. amr2 was observed rotating away from its
        # target, -1.873 to -2.029 rad, and amr1 never left SNAPSHOT_ALIGN.
        #
        # A floor and a slew limit are the right tools, but they need the floor
        # to scale down inside the final approach angle rather than being
        # constant, and that is worth doing carefully rather than under a
        # deadline. Gain 1.2 with no floor and no acceleration limit is the
        # behaviour that was measured working all day.
        self.declare_parameter("angular_gain", 1.2)
        self.declare_parameter("min_angular_speed", 0.0)
        self.declare_parameter("max_angular_accel", 1000.0)
        # Floor on the taper, so the last few centimetres are still covered
        # rather than crept through.
        self.declare_parameter("min_approach_speed_fraction", 0.50)
        # The vision bearing is worth roughly +/-3 degrees at 3 m, which is
        # 0.16 m of lateral error - the same order as the 0.09-0.12 m the
        # coordinate dock already achieves on its own. Clamping the correction
        # keeps a bad reading from making a working dock worse, while still
        # letting a real offset be corrected.
        self.declare_parameter("snapshot_max_correction_m", 0.05)
        # Beyond this the robot is not pointed at the Dolly and the reading is
        # far more likely to be background than target.
        self.declare_parameter("snapshot_max_bearing_deg", 12.0)

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
        self.vision_dock_mission = int(
            self.get_parameter("vision_dock_mission").value
        )
        self.vision_dock_all_missions = bool(
            self.get_parameter("vision_dock_all_missions").value
        )
        self.vision_timeout_sec = float(
            self.get_parameter("vision_timeout_sec").value
        )
        self.snapshot_standoff_m = float(
            self.get_parameter("snapshot_standoff_m").value
        )
        self.snapshot_timeout_sec = float(
            self.get_parameter("snapshot_timeout_sec").value
        )
        self.snapshot_dwell_sec = float(
            self.get_parameter("snapshot_dwell_sec").value
        )
        self.camera_forward_m = float(
            self.get_parameter("camera_forward_m").value
        )
        self.approach_ramp_m = float(
            self.get_parameter("approach_ramp_m").value
        )
        self.standoff_yaw_tolerance_rad = float(
            self.get_parameter("standoff_yaw_tolerance_rad").value
        )
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.min_angular_speed = float(
            self.get_parameter("min_angular_speed").value
        )
        self.max_angular_accel = float(
            self.get_parameter("max_angular_accel").value
        )
        self.last_angular_cmd = 0.0
        self.min_approach_speed_fraction = float(
            self.get_parameter("min_approach_speed_fraction").value
        )
        self.dolly_footprint_m = self.dolly_footprint(inventory, (1.24, 0.86))
        self.dolly_deck_width_m = float(
            self.get_parameter("dolly_deck_width_m").value
        )
        self.get_logger().info(
            "Dolly footprint from inventory: %.2f x %.2f m; deck width in use: "
            "%.2f m"
            % (
                self.dolly_footprint_m[0],
                self.dolly_footprint_m[1],
                self.dolly_deck_width_m
                if self.dolly_deck_width_m > 0
                else max(self.dolly_footprint_m),
            )
        )
        self.snapshot_max_correction_m = float(
            self.get_parameter("snapshot_max_correction_m").value
        )
        self.snapshot_max_bearing_deg = float(
            self.get_parameter("snapshot_max_bearing_deg").value
        )

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
        self.snapshot_request_publisher = self.create_publisher(
            String, str(self.get_parameter("snapshot_request_topic").value), 10
        )
        self.snapshot_result_subscription = self.create_subscription(
            String,
            str(self.get_parameter("snapshot_result_topic").value),
            self.snapshot_result_callback,
            10,
        )
        if self.vision_dock_mission >= 0:
            self.vision_cmd_subscription = self.create_subscription(
                Twist,
                str(self.get_parameter("vision_cmd_topic").value),
                self.vision_cmd_callback,
                10,
            )
            self.vision_status_subscription = self.create_subscription(
                String,
                str(self.get_parameter("vision_status_topic").value),
                self.vision_status_callback,
                10,
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

        # Vision docking state.
        self.vision_cmd = None
        self.vision_cmd_at = 0.0
        self.vision_complete = False
        self.vision_used = False
        self.vision_fallback_logged = False

        # Snapshot docking state. The offset is in factory coordinates and is
        # added to both dock targets of the current mission, then cleared.
        self.snapshot_seq = 0
        self.snapshot_result = None
        self.snapshot_deadline = None
        self.snapshot_hold_until = None
        self.snapshot_pose = None
        self.snapshot_done = False
        self.snapshot_resume_state = None
        self.dock_offset = (0.0, 0.0)

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

    def vision_cmd_callback(self, msg):
        self.vision_cmd = msg
        self.vision_cmd_at = time.monotonic()

    def vision_status_callback(self, msg):
        if msg.data == "DOCKING_COMPLETE":
            self.vision_complete = True
            self.get_logger().info("VISION reports DOCKING_COMPLETE")

    def snapshot_result_callback(self, msg):
        try:
            result = json.loads(msg.data)
        except ValueError:
            return
        # The vision node repeats each reply five times and an abandoned
        # request may still answer late, so match on the sequence number
        # rather than trusting whatever arrived most recently.
        if int(result.get("seq", -1)) != self.snapshot_seq:
            return
        if self.snapshot_result is None:
            self.snapshot_result = result

    def snapshot_enabled(self):
        if self.vision_dock_all_missions:
            return True
        return (
            self.vision_dock_mission >= 0
            and self.mission_index == self.vision_dock_mission
        )

    def request_snapshot(self, now):
        self.snapshot_seq += 1
        self.snapshot_result = None
        self.snapshot_deadline = now + self.snapshot_timeout_sec
        self.snapshot_pose = self.world_pose()
        x, y, _ = self.snapshot_pose
        dock = self.dock_info["dock"]
        # Range from the lens to the middle of the Dolly, which is what the
        # deck's apparent width is set by.
        range_m = max(
            0.1,
            math.hypot(dock["x"] - x, dock["y"] - y) - self.camera_forward_m,
        )
        message = String()
        message.data = json.dumps(
            {
                "seq": self.snapshot_seq,
                "amr": self.amr_name,
                "mission": self.mission_index,
                "range_m": range_m,
                "deck_width_m": self.deck_width_across(
                    math.radians(float(dock["yaw_deg"]))
                ),
            }
        )
        for _ in range(5):
            self.snapshot_request_publisher.publish(message)
        self.get_logger().info(
            "SNAPSHOT requested at %.2f m standoff (seq %d)"
            % (self.snapshot_standoff_m, self.snapshot_seq)
        )

    def apply_snapshot(self, result):
        """Turn a measured bearing into a clamped lateral shift of the dock.

        The bearing is measured from the robot's own heading, and at the moment
        it is taken the robot is stopped facing the dock target, so the
        difference between where the Dolly was seen and where the dock target
        lies is the lateral error. Everything else about the approach - the
        route, the speeds, the tolerances - is left exactly as it was.
        """
        self.dock_offset = (0.0, 0.0)

        if result is None:
            self.get_logger().warning(
                "SNAPSHOT timed out, docking on coordinates alone"
            )
            return
        if not result.get("ok"):
            if result.get("present"):
                # The useful half of the answer. Confirming the Dolly is there
                # before committing is what vision is reliable at here; the
                # trim is a bonus that this frame could not supply.
                self.get_logger().info(
                    "SNAPSHOT DOLLY CONFIRMED on %d/%d frames - entering dock "
                    "on coordinates (%s)"
                    % (
                        int(result.get("seen", 0)),
                        int(result.get("of", 0)),
                        result.get("reason", ""),
                    )
                )
            else:
                self.get_logger().warning(
                    "SNAPSHOT found no Dolly (%s), docking on coordinates alone"
                    % result.get("reason", "no reason given")
                )
            return

        bearing_deg = float(result["bearing_deg"])
        if abs(bearing_deg) > self.snapshot_max_bearing_deg:
            self.get_logger().warning(
                "SNAPSHOT bearing %+.2f deg exceeds %.1f deg limit, ignored"
                % (bearing_deg, self.snapshot_max_bearing_deg)
            )
            return

        x, y, yaw = self.snapshot_pose
        dock = self.dock_info["dock"]
        # Where the dock target sits relative to the heading the robot is
        # holding, so the two bearings are measured from the same place.
        expected = normalize_angle(
            math.atan2(dock["y"] - y, dock["x"] - x) - yaw
        )
        error_rad = math.radians(bearing_deg) - expected
        range_m = math.hypot(dock["x"] - x, dock["y"] - y)
        lateral = range_m * math.tan(error_rad)

        clamped = max(
            -self.snapshot_max_correction_m,
            min(self.snapshot_max_correction_m, lateral),
        )
        # Left of the robot's heading, which is the direction the lateral error
        # is measured along.
        self.dock_offset = (
            -math.sin(yaw) * clamped,
            math.cos(yaw) * clamped,
        )
        self.get_logger().info(
            "SNAPSHOT DOLLY seen at %+.2f deg (expected %+.2f deg, area %.1f%%, "
            "%d/%d frames) -> lateral %+.3f m, applying %+.3f m"
            % (
                bearing_deg,
                math.degrees(expected),
                100.0 * float(result.get("area_fraction", 0.0)),
                int(result.get("samples", 0)),
                int(result.get("of", 0)),
                lateral,
                clamped,
            )
        )

    # The docking camera is a 30-degree-FOV lens, so a Dolly only fits in the
    # frame from roughly 2 m out. FINAL_DOCK alone starts at about 1.5 m, by
    # which point the Dolly overflows the view. Handing over one leg earlier,
    # at GO_TO_PRE_DOCK, is where vision can actually see and correct.
    VISION_STATES = ("GO_TO_PRE_DOCK", "FINAL_DOCK")

    def vision_active(self, now):
        """True while the vision node is steering this docking approach."""
        if self.vision_dock_mission < 0:
            return False
        if self.mission_index != self.vision_dock_mission:
            return False
        if self.state not in self.VISION_STATES or self.vision_complete:
            return False
        if self.vision_cmd is None:
            return False
        return now - self.vision_cmd_at <= self.vision_timeout_sec

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
        # Clear the slew-rate state too. Leaving it set would make the next
        # turn ramp from a speed the robot is no longer travelling at, which
        # reintroduces exactly the step change smooth_angular() exists to
        # avoid.
        self.last_angular_cmd = 0.0
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
        self.vision_cmd = None
        self.vision_complete = False
        self.vision_used = False
        self.vision_fallback_logged = False
        # A correction belongs to the Dolly it was measured against. Carrying
        # one into the next mission would offset a dock that was never looked
        # at, so it is dropped here rather than at the end of the dock.
        self.snapshot_result = None
        self.dock_offset = (0.0, 0.0)
        self.snapshot_done = False
        self.snapshot_resume_state = None
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
        if self.snapshot_enabled() and not self.snapshot_done:
            # Only if the approach never passed the standoff, which happens
            # when the route already starts inside it.
            self.aim_at_standoff()
        else:
            self.aim_at_pre_dock()

    @staticmethod
    def longest_dolly_half_length(inventory, fallback):
        """Half the longest Dolly footprint the bridge measured.

        The longest rather than the matching one: a standoff that clears the
        biggest Dolly clears all of them, and being a little further back only
        costs a smaller Dolly in the frame, which the colour detector does not
        mind. Choosing per-Dolly would mean trusting a path lookup to stay in
        step with the inventory for no real gain.
        """
        lengths = [
            float(dolly["half_length_m"])
            for dolly in inventory.get("dollies", [])
            if "half_length_m" in dolly
        ]
        return max(lengths) if lengths else fallback

    @staticmethod
    def dolly_footprint(inventory, fallback):
        """Largest Dolly footprint (size_x, size_y) in world axes, in metres.

        Kept as the two axes rather than reduced to one number: which of them
        the camera sees across and which it sees into depends on the heading of
        the particular dock, and guessing wrong picks the 0.86 m side of a
        Dolly the camera is looking at 1.24 m of.
        """
        footprints = [
            (float(dolly["size"][0]), float(dolly["size"][1]))
            for dolly in inventory.get("dollies", [])
            if "size" in dolly
        ]
        return max(footprints, key=lambda s: s[0] * s[1]) if footprints else fallback

    def deck_width_across(self, heading_rad):
        """Upper bound on how wide the Dolly can look. Deliberately not exact.

        Predicting the exact width needs the Dolly's orientation relative to
        the approach, and an attempt at that made things worse: the footprint
        the bridge reports is already a world-axis bounding box, so applying
        the Dolly's yaw to it rotated the rectangle twice. It predicted the
        0.865 m side against a Dolly presenting 1.242 m, and the width gate
        refused two correct detections in a row.

        The gate does not need an exact figure. It exists to reject a blob
        three metres wide - background plant merged with the deck - not to
        measure. Predicting the longest side the Dolly could show, with the
        tolerance already wide enough to cover the shortest (0.865 / 1.242 =
        0.70, inside the +/-45% band), accepts either orientation and still
        refuses anything that is not a Dolly at all.
        """
        if self.dolly_deck_width_m > 0.0:
            return self.dolly_deck_width_m
        return max(self.dolly_footprint_m)

    def deck_depth_along(self, heading_rad):
        """How much Dolly lies between its near edge and the dock target."""
        size_x, size_y = self.dolly_footprint_m
        return (
            abs(size_x * math.cos(heading_rad))
            + abs(size_y * math.sin(heading_rad))
        ) / 2.0

    def aim_at_standoff(self):
        """Stop short of the Dolly, on the dock axis, facing it.

        Backing off along the dock heading rather than picking an arbitrary
        point keeps the robot pointed exactly where it will drive, so the
        bearing the camera measures is directly comparable to the bearing of
        the dock target and no extra frame conversion is needed.

        The chassis has to stop far enough back that the *camera* clears the
        Dolly's near edge by the requested distance, so the Dolly's own half
        length and the camera's forward offset both come off the total.
        """
        dock = self.dock_info["dock"]
        heading = math.radians(float(dock["yaw_deg"]))
        half_depth = self.deck_depth_along(heading)
        back_off = (
            self.snapshot_standoff_m + half_depth + self.camera_forward_m
        )
        self.get_logger().info(
            "STANDOFF %.2f m back from dock "
            "(%.2f m clear + %.2f m Dolly half-depth + %.2f m camera offset)"
            % (back_off, self.snapshot_standoff_m, half_depth,
               self.camera_forward_m)
        )
        self.aim_at(
            dock["x"] - back_off * math.cos(heading),
            dock["y"] - back_off * math.sin(heading),
            dock["yaw_deg"],
            "VISION_STANDOFF",
        )

    def aim_at_pre_dock(self):
        pre = self.dock_info["pre_dock"]
        offset_x, offset_y = self.dock_offset
        self.aim_at(
            pre["x"] + offset_x,
            pre["y"] + offset_y,
            pre["yaw_deg"],
            "GO_TO_PRE_DOCK",
        )

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

        if self.state == "WAIT_SNAPSHOT":
            # Stationary by design: this is the only moment in the run where
            # the camera is looking at the Dolly from a platform that is not
            # moving, which is what makes a single measurement trustworthy.
            self.publish_stop()
            if self.snapshot_result is not None:
                self.apply_snapshot(self.snapshot_result)
            elif now > self.snapshot_deadline:
                self.apply_snapshot(None)
            else:
                return
            self.snapshot_hold_until = now + self.snapshot_dwell_sec
            self.state = "SNAPSHOT_HOLD"
            return

        if self.state == "SNAPSHOT_ALIGN":
            _, _, yaw = self.world_pose()
            target = math.radians(float(self.dock_info["dock"]["yaw_deg"]))
            error = normalize_angle(target - yaw)
            if abs(error) > self.standoff_yaw_tolerance_rad:
                command = Twist()
                command.angular.z = self.smooth_angular(
                    error, self.dock_angular_speed
                )
                self.publisher.publish(command)
                return
            self.publish_stop()
            self.get_logger().info(
                "STANDOFF aligned to %.1f deg, taking snapshot"
                % math.degrees(target)
            )
            self.request_snapshot(now)
            self.state = "WAIT_SNAPSHOT"
            return

        if self.state == "SNAPSHOT_HOLD":
            # Deliberately doing nothing, so the recognition can be seen.
            self.publish_stop()
            if now >= self.snapshot_hold_until:
                if self.snapshot_resume_state is not None:
                    state, index = self.snapshot_resume_state
                    self.snapshot_resume_state = None
                    self.get_logger().info(
                        "SNAPSHOT hold done, resuming approach"
                    )
                    # Carry on to the waypoint the approach was interrupted at,
                    # rather than jumping to the dock: the route still has to
                    # reach the pickup node before docking begins.
                    self.aim_at_waypoint(index)
                else:
                    self.get_logger().info("SNAPSHOT hold done, entering dock")
                    self.aim_at_pre_dock()
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

        if self.maybe_snapshot_on_approach(now):
            return

        self.drive_towards_target(now)

    # ------------------------------------------------------------- steering

    def smooth_angular(self, error, limit):
        """Angular command that neither snaps nor crawls.

        A bare proportional term does both. `1.2 * error` starts at the speed
        cap the instant the turn begins, which pitches the chassis forward hard
        enough to see, and then decays towards zero so the last degree takes
        several seconds - the standoff alignment was measured at 10.2 s, almost
        all of it in the tail.

        Three changes fix it: a higher gain so the turn is decisive, a floor so
        the tail is covered at a useful rate instead of asymptotically, and a
        limit on how fast the command itself may change so the start is a ramp
        rather than a step. The acceleration limit is what removes the lurch;
        the floor is what removes the wait.
        """
        wanted = self.angular_gain * error
        magnitude = min(limit, max(self.min_angular_speed, abs(wanted)))
        wanted = math.copysign(magnitude, error)

        # 20 Hz control loop, so this is rad/s per tick.
        step = self.max_angular_accel * 0.05
        delta = wanted - self.last_angular_cmd
        if abs(delta) > step:
            wanted = self.last_angular_cmd + math.copysign(step, delta)
        self.last_angular_cmd = wanted
        return wanted

    def standoff_back_off(self):
        """Chassis distance behind the dock target that clears the Dolly."""
        heading = math.radians(float(self.dock_info["dock"]["yaw_deg"]))
        return (
            self.snapshot_standoff_m
            + self.deck_depth_along(heading)
            + self.camera_forward_m
        )

    def maybe_snapshot_on_approach(self, now):
        """Take the picture on the way in, not after driving past.

        The approach route ends at the pickup node, which for Node_10 sits
        2.2 m nearer the Dolly than the standoff does. Aiming at the standoff
        only after arriving there meant reversing away from the Dolly, stopping,
        and driving back in - three moves where one would do, and it looked
        like the robot had changed its mind.

        Watching the range during the approach instead means the standoff is
        reached going forwards. The robot is already pointed at the Dolly at
        that moment, which is also the condition the bearing measurement needs.

        Snapshotting at the pickup node itself is not an option: at 2.47 m the
        deck spans 647 px of a 640 px frame, so it is clipped and unmeasurable.
        """
        if not self.snapshot_enabled() or self.snapshot_done:
            return False
        if self.state != "APPROACH" or self.dock_info is None:
            return False

        x, y, _ = self.world_pose()
        dock = self.dock_info["dock"]
        remaining = math.hypot(dock["x"] - x, dock["y"] - y)
        if remaining > self.standoff_back_off():
            return False

        self.publish_stop()
        self.snapshot_done = True
        self.snapshot_resume_state = (self.state, self.waypoint_index)
        self.get_logger().info(
            "STANDOFF reached on approach at %.2f m from dock, "
            "turning to face the Dolly" % remaining
        )
        # Stopping is not enough: the approach arrives on whatever heading the
        # route left it on, and the first attempt at this measured 70 px of
        # deck where 318 were predicted because the Dolly was off to one side.
        # Turning on the spot to the dock heading costs a second and puts the
        # Dolly in the middle of the frame, which is the condition every
        # bearing figure in the worklog was measured under.
        self.state = "SNAPSHOT_ALIGN"
        self.reached = False
        return True

    def drive_towards_target(self, now):
        if self.vision_active(now):
            if not self.vision_used:
                self.vision_used = True
                self.get_logger().info(
                    "VISION DOCKING engaged for mission %d"
                    % (self.mission_index + 1)
                )
            self.publisher.publish(self.vision_cmd)
            return

        if (
            self.state in self.VISION_STATES
            and self.vision_used
            and not self.vision_complete
            and not self.vision_fallback_logged
        ):
            self.vision_fallback_logged = True
            # Expected at the end of the visual approach: once the Dolly fills
            # the narrow lens there is nothing left to measure, so the vision
            # node stops publishing and coordinate docking finishes the insert.
            # The same path also covers a genuine dropout, which is the point -
            # either way the robot keeps docking.
            self.get_logger().warning(
                "VISION handed back to coordinate docking "
                "(node stopped publishing or timed out)"
            )

        # The standoff is a place to stop and look, so it is approached at
        # docking speed rather than cruise speed. Arriving fast meant arriving
        # past the point, and recovering from that is what produced the long
        # spin-in-place before the snapshot.
        docking = self.state in ("FINAL_DOCK", "VISION_STANDOFF")
        tolerance = self.dock_tolerance if docking else self.tolerance
        angular_limit = (
            self.dock_angular_speed if docking else self.max_angular_speed
        )
        linear_limit = self.dock_linear_speed if docking else self.max_linear_speed

        x, y, yaw = self.world_pose()
        dx = self.target_x - x
        dy = self.target_y - y
        distance = math.hypot(dx, dy)

        # Ease off before arriving instead of running flat out into the
        # tolerance band. At 4 m/s and a 20 Hz loop the robot covers 0.2 m per
        # tick, so a target with a 0.1 m tolerance was routinely overshot; the
        # controller then had to turn most of the way round and come back,
        # which is the circling that was visible in the viewport. Scaling speed
        # with the remaining distance removes the overshoot at the source.
        if distance < self.approach_ramp_m:
            linear_limit *= max(
                self.min_approach_speed_fraction,
                distance / self.approach_ramp_m,
            )

        if distance <= tolerance and self.target_yaw is not None:
            yaw_error = normalize_angle(self.target_yaw - yaw)
            # The standoff is aimed tighter than a waypoint because the whole
            # point of stopping there is the picture. Arriving 4.5 degrees off
            # put a Dolly that was entirely visible far enough to one side that
            # its deck ran past the frame edge, and a clipped deck has no
            # usable centroid - the measurement was lost to heading, not to
            # the detector.
            yaw_limit = (
                self.standoff_yaw_tolerance_rad
                if self.state == "VISION_STANDOFF"
                else self.yaw_tolerance
            )
            if abs(yaw_error) > yaw_limit:
                command = Twist()
                command.angular.z = self.smooth_angular(
                    yaw_error, angular_limit
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

        # A target that ends up behind the robot by a few centimetres is not
        # worth turning around for. Without this the controller would spin
        # 180 degrees, drive the short distance, overshoot again and repeat -
        # the circling that was visible at the standoff. Close enough and
        # facing away is treated as arrived; the yaw alignment above still
        # runs, so the final heading is unaffected.
        if distance <= tolerance * 1.5 and abs(heading_error) > math.radians(120):
            self.publish_stop()
            self.reached = True
            self.get_logger().info(
                "%s close enough at %.3f m (target is behind by %.0f deg)"
                % (self.state, distance, math.degrees(abs(heading_error)))
            )
            self.on_target_reached(now, x, y, yaw, distance)
            return

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

        if state == "VISION_STANDOFF":
            self.get_logger().info(
                "REACHED STANDOFF world=(%.3f, %.3f, %.1f deg), error=%.3f m"
                % (x, y, math.degrees(yaw), distance)
            )
            # Hold still until the answer arrives. The robot is already stopped
            # by the reached branch above, and WAIT_SNAPSHOT keeps publishing
            # zero velocity, so the frames the vision node measures are taken
            # from a stationary platform.
            self.request_snapshot(now)
            self.state = "WAIT_SNAPSHOT"
            self.reached = False
            self.started_at = now
            return

        if state == "GO_TO_PRE_DOCK":
            self.get_logger().info(
                "REACHED PRE_DOCK world=(%.3f, %.3f, %.1f deg), error=%.3f m"
                % (x, y, math.degrees(yaw), distance)
            )
            dock = self.dock_info["dock"]
            offset_x, offset_y = self.dock_offset
            self.aim_at(
                dock["x"] + offset_x,
                dock["y"] + offset_y,
                dock["yaw_deg"],
                "FINAL_DOCK",
            )
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
