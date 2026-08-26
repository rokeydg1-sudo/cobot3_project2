#!/usr/bin/env python3
"""Isaac Sim standalone side of the minimal integration test.

Run this with Isaac Sim's Python wrapper (for example
`${ISAAC_SIM_DIR:-$HOME/isaacsim}/python.sh`).
It creates a ground plane, four station markers, and one simple dynamic cuboid AMR.
The AMR receives `/fms/goal` and publishes `/amr/pose` + `/amr/status`.
"""

from __future__ import annotations

# SimulationApp must be created before most Isaac Sim / Omniverse imports.
from isaacsim.simulation_app import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.utils.extensions import enable_extension

# Enable the ROS 2 bridge before importing rclpy/message packages.
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import String

LOCATIONS = {
    0: {"name": "Depot", "xy": (0.0, 0.0)},
    1: {"name": "Parts_Supermarket", "xy": (-3.0, 2.0)},
    2: {"name": "Assembly_Cell_A", "xy": (3.0, 2.0)},
    3: {"name": "Assembly_Cell_B", "xy": (3.0, -2.0)},
}

AMR_HEIGHT_M = 0.30
AMR_Z_M = AMR_HEIGHT_M / 2.0
AMR_SPEED_MPS = 1.0
ARRIVAL_TOLERANCE_M = 0.10
POSE_PUBLISH_EVERY_N_FRAMES = 6  # ~10 Hz when physics is 60 Hz


class IsaacAmrRosNode(Node):
    def __init__(self) -> None:
        super().__init__("isaac_simple_amr")

        self.goal_sub = self.create_subscription(
            Point, "/fms/goal", self._on_goal, 10
        )
        self.pose_pub = self.create_publisher(Point, "/amr/pose", 10)
        self.status_pub = self.create_publisher(String, "/amr/status", 10)

        self.goal_xy: np.ndarray | None = None
        self.frame_count = 0
        self._last_goal_for_log: tuple[float, float] | None = None

        self.get_logger().info("ROS interface ready")
        self.publish_status("READY")

    def _on_goal(self, msg: Point) -> None:
        self.goal_xy = np.array([float(msg.x), float(msg.y)], dtype=np.float64)
        goal_key = (round(float(msg.x), 3), round(float(msg.y), 3))
        if goal_key != self._last_goal_for_log:
            self._last_goal_for_log = goal_key
            self.get_logger().info(
                f"Goal received: x={msg.x:.2f}, y={msg.y:.2f}"
            )
            self.publish_status("MOVING")

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_pose(self, position: np.ndarray) -> None:
        msg = Point()
        msg.x = float(position[0])
        msg.y = float(position[1])
        msg.z = float(position[2])
        self.pose_pub.publish(msg)

    def update_motion(self, amr: DynamicCuboid) -> None:
        position, _ = amr.get_world_pose()
        position = np.asarray(position, dtype=np.float64)

        if self.goal_xy is None:
            amr.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
        else:
            delta = self.goal_xy - position[:2]
            distance = float(np.linalg.norm(delta))

            if distance <= ARRIVAL_TOLERANCE_M:
                amr.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
                self.get_logger().info(
                    f"Goal reached: x={position[0]:.2f}, y={position[1]:.2f}"
                )
                self.publish_status("ARRIVED")
                self.goal_xy = None
            else:
                direction = delta / distance
                velocity = np.array(
                    [
                        direction[0] * AMR_SPEED_MPS,
                        direction[1] * AMR_SPEED_MPS,
                        0.0,
                    ],
                    dtype=np.float64,
                )
                amr.set_linear_velocity(velocity)

        self.frame_count += 1
        if self.frame_count % POSE_PUBLISH_EVERY_N_FRAMES == 0:
            position, _ = amr.get_world_pose()
            self.publish_pose(np.asarray(position))


# ---------------------------------------------------------------------------
# World setup
# ---------------------------------------------------------------------------
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

station_colors = {
    0: np.array([0.25, 0.25, 0.25]),
    1: np.array([0.15, 0.65, 0.95]),
    2: np.array([0.25, 0.85, 0.35]),
    3: np.array([0.95, 0.55, 0.15]),
}

# Low visual markers only; they do not block the proxy AMR.
for location_id, data in LOCATIONS.items():
    x, y = data["xy"]
    world.scene.add(
        VisualCuboid(
            prim_path=f"/World/Station_{location_id}",
            name=f"station_{location_id}",
            position=np.array([x, y, 0.01]),
            scale=np.array([0.8, 0.8, 0.02]),
            color=station_colors[location_id],
        )
    )

amr = world.scene.add(
    DynamicCuboid(
        prim_path="/World/SimpleAMR",
        name="simple_amr",
        position=np.array([0.0, 0.0, AMR_Z_M]),
        scale=np.array([0.60, 0.40, AMR_HEIGHT_M]),
        color=np.array([0.85, 0.15, 0.15]),
        mass=10.0,
    )
)

world.reset()

rclpy.init(args=None)
ros_node = IsaacAmrRosNode()

print("\n=== Isaac Sim integration world ===")
print("SUB  /fms/goal   geometry_msgs/msg/Point")
print("PUB  /amr/pose   geometry_msgs/msg/Point")
print("PUB  /amr/status std_msgs/msg/String")
print("STATE ISAAC_READY\n")

try:
    while simulation_app.is_running():
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        ros_node.update_motion(amr)
        world.step(render=True)
finally:
    amr.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
    ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    simulation_app.close()
