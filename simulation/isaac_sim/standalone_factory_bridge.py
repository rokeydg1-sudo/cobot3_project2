#!/usr/bin/env python3
"""
standalone_factory_bridge_v2.py
==============================

역할
----
PC1에서 Isaac Sim 5.1을 Standalone으로 실행하고, 공장 USD를 연 뒤
IW Hub를 ROS 2 Bridge에 연결하는 "시뮬레이터/브리지 전용" 프로세스.

이 파일에는 Mission / cuOpt / Dolly 선택 / 도킹 판단 로직을 넣지 않는다.
그 로직은 PC2의 ROS 2 노드들이 담당한다.

PC2 -> PC1
-----------
/cmd_vel                       geometry_msgs/Twist
    AMR 주행 명령

PC1 -> PC2
-----------
/clock                         rosgraph_msgs/Clock
/odom                          nav_msgs/Odometry
/tf                            tf2_msgs/TFMessage
/tf_static                     tf2_msgs/TFMessage

카메라 Prim이 Stage 안에 존재할 경우:
/vision/front_camera/image_raw sensor_msgs/Image

추가 진단
---------
- Stage에서 IW Hub 자동 탐색
- left_wheel_joint / right_wheel_joint / lift_joint 검사
- Dolly 후보 전부 탐색 + World XYZ/Yaw 출력
- /World/WaypointGraph/Nodes 아래 Node 좌표 전부 출력
- Camera 후보 탐색

중요
----
- 기존 USD 안의 ActionGraph / ROS_Lidar_Graph는 런타임에서 비활성화하여
  중복 Publisher/Subscriber가 생기지 않도록 한다.
- 원본 USD 자체를 Save 하지 않으므로 디스크 원본은 수정하지 않는다.
- rclpy를 직접 import하지 않는다.
  Isaac Sim의 `isaacsim.ros2.bridge` OmniGraph 노드를 사용한다.

실행 예
-------
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30

cd ~/cobot3_project2

~/isaacsim/python.sh simulation/isaac_sim/standalone_factory_bridge.py

USD 경로는 스크립트 위치를 기준으로 자동 해석되므로 별도 설정이 필요 없다.
다른 위치의 에셋을 쓰려면 FACTORY_USD / SENSORS_USD 환경변수를 지정한다.

다른 PC에서도 같은 ROS_DOMAIN_ID를 사용한다.
"""

# ============================================================
# 0. SimulationApp must be created first
# ============================================================

import os as _os

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": _os.environ.get("HEADLESS", "0") == "1",
        "width": 1280,
        "height": 720,
        "sync_loads": True,
    }
)

# ============================================================
# 1. Imports after SimulationApp
# ============================================================

import json
import math
import os
import re
import sys
import time
import traceback
from pathlib import Path

import omni.graph.core as og
import omni.usd
import omni.timeline
import numpy as np
import mission_planner as planner
from pxr import Gf, Sdf, Usd, UsdGeom
from pxr import UsdPhysics, UsdShade

import isaacsim.core.utils.stage as stage_utils
from isaacsim.core.utils.extensions import enable_extension


# ============================================================
# 2. User configuration
# ============================================================

# Everything resolves relative to this file, so the repository can be cloned
# anywhere. Environment variables still win if you want to point elsewhere.
SCRIPT_DIR = Path(__file__).resolve().parent
ASSET_DIR = Path(
    os.environ.get("FACTORY_ASSET_DIR", str(SCRIPT_DIR / "Collected_AF2_FLAT"))
)

FACTORY_USD = os.environ.get(
    "FACTORY_USD", str(ASSET_DIR / "AF2_MULTI_BACKUP.usd")
)

SENSORS_USD = os.environ.get(
    "SENSORS_USD", str(ASSET_DIR / "iw_hub_sensors.usd")
)

INVENTORY_TXT = Path(
    os.environ.get(
        "INVENTORY_TXT",
        str(SCRIPT_DIR / "factory_inventory.txt"),
    )
)

INVENTORY_JSON = Path(
    os.environ.get(
        "INVENTORY_JSON",
        str(SCRIPT_DIR / "factory_inventory.json"),
    )
)

GRAPH_PATH = "/World/StandaloneROSBridge"

CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
CLOCK_TOPIC = "/clock"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
LIFT_CMD_TOPIC = "/lift_cmd"
LIFT_STATE_TOPIC = "/lift_joint_state"

ODOM_FRAME = "odom"
BASE_FRAME = "base_link"
BASE_FOOTPRINT_FRAME = "base_footprint"

CAMERA_FRAME = "front_camera"
CAMERA_TOPIC = "/vision/front_camera/image_raw"
CAMERA_NAME = "transporter_camera_first_person"

# Docking camera placement, measured in the robot's own frame: metres ahead of
# the chassis origin, metres above the floor, and how far the optical axis is
# tilted down from horizontal.
#
# The authored `transporter_camera_first_person` cannot be reused as-is. Copying
# its pose puts the docking view up in the roof trusses - a frame captured 0.5 m
# from a Dolly showed sky and a storage rack, no Dolly at all. The synthetic
# training set placed the Dolly centred on the optical axis at 2.0-3.5 m, so
# inference only matches training if the camera looks straight ahead and roughly
# level. Only the pose is rebuilt here; the optics below are still copied from
# the authored camera, so the calibrated intrinsics stay valid.
# The authored waypoint markers are white spheres parked at z = 0.15 m, which
# is inside the docking camera's line of sight. Node 10 sits on amr1's approach
# line to its pickup Dolly, and a frame captured 1.07 m short of it was one
# smooth white surface edge to edge - the detector had nothing else to look at.
# Hidden by default; set SHOW_WAYPOINT_GRAPH=1 to get them back for debugging.
SHOW_WAYPOINT_GRAPH = os.environ.get("SHOW_WAYPOINT_GRAPH", "0") in (
    "1",
    "true",
    "True",
)

# Periodically print the docking camera's composed world pose, for checking the
# aim against what the pictures actually show.
CAMERA_DEBUG = os.environ.get("CAMERA_DEBUG", "0") in ("1", "true", "True")

CAMERA_FORWARD_M = float(os.environ.get("CAMERA_FORWARD_M", "0.55"))
CAMERA_HEIGHT_M = float(os.environ.get("CAMERA_HEIGHT_M", "0.45"))
CAMERA_PITCH_DEG = float(os.environ.get("CAMERA_PITCH_DEG", "4.0"))

# Overwrite the camera's optics from the intrinsics file. Off by default: the
# renderer does not honour the values reliably, and the file is now a
# calibrated description of the authored lens rather than a request to change
# it. See create_docking_camera().
FORCE_CAMERA_OPTICS = os.environ.get("FORCE_CAMERA_OPTICS", "0") in (
    "1", "true", "True",
)

CAMERA_WIDTH = int(os.environ.get("CAMERA_WIDTH", "1280"))
CAMERA_HEIGHT = int(os.environ.get("CAMERA_HEIGHT", "720"))

# IW Hub 값: 기존 정상 구성 기준
WHEEL_RADIUS = float(os.environ.get("WHEEL_RADIUS", "0.10"))
WHEEL_DISTANCE = float(os.environ.get("WHEEL_DISTANCE", "0.70"))
# 9x the original cruise limits (0.45 / 0.70 -> 1.35 / 2.10 -> 4.05 / 6.30).
# Precision docking stays slow and is limited by the mission controller.
MAX_LINEAR_SPEED = float(os.environ.get("MAX_LINEAR_SPEED", "4.05"))
MAX_ANGULAR_SPEED = float(os.environ.get("MAX_ANGULAR_SPEED", "6.30"))
# (4.05 + 6.30 * 0.70 / 2) / 0.10 = 62.55 rad/s, so the cap needs headroom.
MAX_WHEEL_SPEED = float(os.environ.get("MAX_WHEEL_SPEED", "100.0"))

# 카메라 브리지는 Camera prim이 실제로 존재할 때만 생성
ENABLE_CAMERA = os.environ.get("ENABLE_CAMERA", "1") not in ("0", "false", "False")
INTRINSICS_PATH = Path(
    os.environ.get(
        "CAMERA_INTRINSICS",
        str(SCRIPT_DIR / "vision_docking" / "config" / "camera_intrinsics.npz"),
    )
)
CALIBRATION_FILE = Path(
    os.environ.get("CAMERA_CALIBRATION_FILE", "/tmp/docking_camera_candidate")
)
PICKUP_DOLLY_PATH = os.environ.get(
    "PICKUP_DOLLY_PATH", "/World/dolly_physics_01"
)
PICKUP_JOINT_PATH = "/World/StandalonePickupJoint"

# PC2 -> PC1 Dolly command channel. Carried on sensor_msgs/JointState as
# position = [command_code, sequence] so PC1 and PC2 need no shared filesystem.
DOLLY_CMD_TOPIC = "/dolly_cmd"
DOLLY_CMD_FREEZE = 1
DOLLY_CMD_ATTACH = 2
DOLLY_CMD_LIFT = 3
DOLLY_CMD_LOWER = 4
DOLLY_CMD_RELEASE = 5
DOLLY_CMD_NAMES = {
    DOLLY_CMD_FREEZE: "FREEZE",
    DOLLY_CMD_ATTACH: "ATTACH",
    DOLLY_CMD_LIFT: "LIFT",
    DOLLY_CMD_LOWER: "LOWER",
    DOLLY_CMD_RELEASE: "RELEASE",
}
# Seconds for the Dolly to travel the full lift height, so it does not snap.
LIFT_RAMP_SEC = float(os.environ.get("LIFT_RAMP_SEC", "1.2"))
# Authored factory floor height; the Dolly is parked 28 cm above it in the USD.
FLOOR_Z = float(os.environ.get("FLOOR_Z", "0.0"))
CARRY_LIFT_HEIGHT = float(os.environ.get("CARRY_LIFT_HEIGHT", "0.04"))

# ============================================================
# 3. Helpers
# ============================================================

def yaw_deg_from_world_matrix(matrix):
    """USD world matrix에서 Z-yaw(deg)를 얻는다."""
    quat = matrix.ExtractRotationQuat()
    w = float(quat.GetReal())
    x, y, z = [float(v) for v in quat.GetImaginary()]

    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return math.degrees(yaw)


def world_pose_of_prim(prim):
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matrix = cache.GetLocalToWorldTransform(prim)
    t = matrix.ExtractTranslation()

    return {
        "x": float(t[0]),
        "y": float(t[1]),
        "z": float(t[2]),
        "yaw_deg": yaw_deg_from_world_matrix(matrix),
    }


def world_bounds_of_prim(prim):
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    bound = cache.ComputeWorldBound(prim)
    box = bound.ComputeAlignedRange()
    minimum, maximum = box.GetMin(), box.GetMax()
    return {
        "min": [float(value) for value in minimum],
        "max": [float(value) for value in maximum],
        "size": [float(maximum[i] - minimum[i]) for i in range(3)],
        "center": [float((minimum[i] + maximum[i]) * 0.5) for i in range(3)],
    }


def inspect_camera_and_robot(stage, robot_path, camera_path):
    """Print composed camera optics and likely moving body prims."""
    camera = stage.GetPrimAtPath(camera_path)
    if camera and camera.IsValid():
        camera_geom = UsdGeom.Camera(camera)
        local_matrix = UsdGeom.Xformable(camera).GetLocalTransformation()
        print("\n[CAMERA ANALYSIS]")
        print(f"  path={camera_path}")
        print(f"  relative_translation={local_matrix.ExtractTranslation()}")
        print(f"  relative_rotation={local_matrix.ExtractRotationQuat()}")
        for name, attr in (
            ("focalLength", camera_geom.GetFocalLengthAttr()),
            ("horizontalAperture", camera_geom.GetHorizontalApertureAttr()),
            ("verticalAperture", camera_geom.GetVerticalApertureAttr()),
            ("horizontalApertureOffset", camera_geom.GetHorizontalApertureOffsetAttr()),
            ("verticalApertureOffset", camera_geom.GetVerticalApertureOffsetAttr()),
            ("clippingRange", camera_geom.GetClippingRangeAttr()),
            ("projection", camera_geom.GetProjectionAttr()),
        ):
            print(f"  {name}={attr.Get()}")

    print("\n[ROBOT BODY CANDIDATES]")
    prefix = robot_path + "/"
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith(prefix):
            continue
        if not (
            prim.HasAPI(UsdPhysics.RigidBodyAPI)
            or prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            or any(token in prim.GetName().lower() for token in ("chassis", "base", "body", "root"))
        ):
            continue
        print(
            f"  {path} type={prim.GetTypeName()} "
            f"rigid={prim.HasAPI(UsdPhysics.RigidBodyAPI)} "
            f"articulation={prim.HasAPI(UsdPhysics.ArticulationRootAPI)} "
            f"pose={world_pose_of_prim(prim)}"
        )


def create_docking_camera(stage, robot_path, source_camera_path):
    """Clone authored camera optics under the moving chassis body."""
    source = stage.GetPrimAtPath(source_camera_path)
    chassis = stage.GetPrimAtPath(f"{robot_path}/chassis")
    if not source.IsValid() or not chassis.IsValid():
        raise RuntimeError("Source camera or moving chassis is missing")

    source_camera = UsdGeom.Camera(source)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    chassis_world = cache.GetLocalToWorldTransform(chassis)

    # Build the view in world space first, then convert into the chassis frame.
    # Doing it that way avoids having to guess which chassis-local axis points
    # forward: the robot prim's yaw is already the fleet-wide definition of
    # forward, since the docking targets are derived from it.
    robot_pose = world_pose_of_prim(stage.GetPrimAtPath(robot_path))
    yaw = math.radians(robot_pose["yaw_deg"])
    forward = Gf.Vec3d(math.cos(yaw), math.sin(yaw), 0.0)
    up = Gf.Vec3d(0.0, 0.0, 1.0)

    chassis_origin = chassis_world.ExtractTranslation()
    eye = Gf.Vec3d(
        chassis_origin[0] + forward[0] * CAMERA_FORWARD_M,
        chassis_origin[1] + forward[1] * CAMERA_FORWARD_M,
        CAMERA_HEIGHT_M,
    )
    tilt = math.radians(CAMERA_PITCH_DEG)
    view = Gf.Vec3d(
        forward[0] * math.cos(tilt),
        forward[1] * math.cos(tilt),
        -math.sin(tilt),
    )
    target = eye + view * 4.0

    # SetLookAt returns a world->camera view matrix; USD wants camera->parent.
    camera_world = Gf.Matrix4d().SetLookAt(eye, target, up).GetInverse()
    # Gf matrices are row-vector, so world = local * parent and therefore
    # local = world * parent^-1. Composing the other way around silently
    # produces a rotated camera.
    mount_relative = Gf.Matrix4d(1.0)
    camera_relative = camera_world * chassis_world.GetInverse()

    print(
        f"  camera_pose forward={CAMERA_FORWARD_M:.2f} m "
        f"height={CAMERA_HEIGHT_M:.2f} m pitch={CAMERA_PITCH_DEG:.1f} deg "
        f"robot_yaw={robot_pose['yaw_deg']:.1f} deg eye=({eye[0]:.3f}, "
        f"{eye[1]:.3f}, {eye[2]:.3f})"
    )

    mount_path = f"{chassis.GetPath()}/docking_camera_mount"
    camera_path = f"{mount_path}/docking_camera"
    mount = UsdGeom.Xform.Define(stage, mount_path)
    mount_op = mount.AddTransformOp()
    mount_op.Set(mount_relative)
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.AddTransformOp().Set(camera_relative)

    for destination, source_attr in (
        (camera.GetFocalLengthAttr(), source_camera.GetFocalLengthAttr()),
        (camera.GetHorizontalApertureAttr(), source_camera.GetHorizontalApertureAttr()),
        (camera.GetVerticalApertureAttr(), source_camera.GetVerticalApertureAttr()),
        (camera.GetHorizontalApertureOffsetAttr(), source_camera.GetHorizontalApertureOffsetAttr()),
        (camera.GetVerticalApertureOffsetAttr(), source_camera.GetVerticalApertureOffsetAttr()),
        (camera.GetClippingRangeAttr(), source_camera.GetClippingRangeAttr()),
        (camera.GetProjectionAttr(), source_camera.GetProjectionAttr()),
        (camera.GetFStopAttr(), source_camera.GetFStopAttr()),
        (camera.GetFocusDistanceAttr(), source_camera.GetFocusDistanceAttr()),
    ):
        value = source_attr.Get()
        if value is not None:
            destination.Set(value)

    # Keep the committed intrinsics authoritative, by moving the focal length
    # rather than the aperture.
    #
    # Setting the aperture alone did not work. The bridge logged
    # aperture=(0.335640, 0.188797) for a 60-degree lens, but measuring the
    # Dolly deck in the resulting frames - known width 1.242 m, known range
    # from odometry - gave an effective fx of about 1467 across five distances
    # with 3% spread, where 554 was configured. Castor diameter agreed at
    # 1270-1450. In other words the renderer kept using the authored optics and
    # the frames were still roughly the authored 30-degree telephoto, while
    # every consumer of the intrinsics file believed otherwise. That single
    # mismatch is what made a Dolly appear two and a half times too large,
    # which in turn made the width gates reject everything and made bearings
    # computed from those pixels wrong.
    #
    # Holding the aperture at whatever the asset authored and solving for the
    # focal length instead means the field of view comes out right whichever of
    # the two the renderer actually reads, because both now describe the same
    # lens.
    if FORCE_CAMERA_OPTICS and INTRINSICS_PATH.exists():
        intrinsics = np.load(INTRINSICS_PATH)
        K = np.asarray(intrinsics["K"], dtype=float)

        horizontal_aperture = float(
            source_camera.GetHorizontalApertureAttr().Get()
        )
        vertical_aperture = horizontal_aperture * CAMERA_HEIGHT / CAMERA_WIDTH
        focal_length = K[0, 0] * horizontal_aperture / CAMERA_WIDTH

        camera.GetFocalLengthAttr().Set(focal_length)
        camera.GetHorizontalApertureAttr().Set(horizontal_aperture)
        camera.GetVerticalApertureAttr().Set(vertical_aperture)
        print(
            f"  FORCED optics from intrinsics: fx={K[0, 0]:.1f}, "
            f"focal={focal_length:.4f}",
            flush=True,
        )
    else:
        # Describe the lens, do not dictate it.
        #
        # Two attempts to widen the field of view from the intrinsics file both
        # failed silently. Setting the aperture logged a 60-degree lens while
        # the renderer kept producing about 30 degrees; setting the focal
        # length moved the measured fx from 1467 to 1286 instead of to the
        # requested 554. Meanwhile every consumer trusted the file, so a Dolly
        # appeared two and a half times larger than predicted, width gates
        # rejected everything, and bearings computed from those pixels were
        # wrong by up to 35 degrees.
        #
        # Leaving the authored optics alone and calibrating the file to match
        # what actually comes out fixed it: bearing error fell from a standard
        # deviation of about 19 degrees to 3.0, with the median at zero.
        # Re-measure with the fx check in the worklog if the camera moves.
        focal = float(camera.GetFocalLengthAttr().Get())
        aperture = float(camera.GetHorizontalApertureAttr().Get())
        implied_fx = focal * CAMERA_WIDTH / aperture
        print(
            f"  authored optics kept: focal={focal:.4f}, "
            f"aperture={aperture:.6f}, implied fx={implied_fx:.1f} "
            f"at {CAMERA_WIDTH} px",
            flush=True,
        )
        if INTRINSICS_PATH.exists():
            K = np.asarray(np.load(INTRINSICS_PATH)["K"], dtype=float)
            print(
                f"  calibrated fx in {INTRINSICS_PATH.name}: {K[0, 0]:.1f} "
                "(measured from rendered frames, authoritative for bearing)",
                flush=True,
            )

    print("\n[DOCKING CAMERA CREATED]")
    print(f"  chassis={chassis.GetPath()}")
    print(f"  camera={camera_path}")
    return camera_path, mount_path, mount_op, mount_relative


def apply_camera_candidate(mount_op, seed_matrix, candidate_index):
    """Apply a chassis-frame extrinsic candidate without touching optics."""
    candidates = (
        (0.00, 0.00, 0.00, 0.0),
        (-0.10, 0.00, 0.05, 5.0),
        (-0.20, 0.00, 0.10, 8.0),
        (-0.30, 0.00, 0.15, 12.0),
        (-0.20, 0.05, 0.10, 8.0),
        (-0.20, -0.05, 0.10, 8.0),
    )
    dx, dy, dz, pitch_deg = candidates[candidate_index % len(candidates)]
    matrix = Gf.Matrix4d(seed_matrix)
    matrix.SetTranslate(
        seed_matrix.ExtractTranslation() + Gf.Vec3d(dx, dy, dz)
    )
    if pitch_deg:
        rotation = Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), pitch_deg)
        matrix.SetRotate(rotation * seed_matrix.ExtractRotation())
    mount_op.Set(matrix)
    return (dx, dy, dz, pitch_deg)


MATRIX_ROW_VECTOR = True


def detect_matrix_convention(stage, prim):
    """Empirically decide how USD composes local/parent transforms here."""
    global MATRIX_ROW_VECTOR
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world = cache.GetLocalToWorldTransform(prim)
    parent = cache.GetParentToWorldTransform(prim)
    local = UsdGeom.Xformable(prim).GetLocalTransformation()
    reference = world.ExtractTranslation()
    row_error = ((local * parent).ExtractTranslation() - reference).GetLength()
    column_error = ((parent * local).ExtractTranslation() - reference).GetLength()
    MATRIX_ROW_VECTOR = row_error <= column_error
    print(
        f"[MATRIX CONVENTION] row_vector={MATRIX_ROW_VECTOR} "
        f"row_error={row_error:.6f} column_error={column_error:.6f}",
        flush=True,
    )
    return MATRIX_ROW_VECTOR


def compose_world(local, parent):
    return local * parent if MATRIX_ROW_VECTOR else parent * local


def local_from_world(world, parent):
    inverse = parent.GetInverse()
    return world * inverse if MATRIX_ROW_VECTOR else inverse * world


def relative_of(child_world, parent_world):
    inverse = parent_world.GetInverse()
    return child_world * inverse if MATRIX_ROW_VECTOR else inverse * child_world


IW_HUB_ASSET = os.path.join(
    os.path.dirname(FACTORY_USD),
    "omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
    "/Isaac/Robots/Idealworks/iwhub/iw_hub_sensors.usd",
)

# Fleet definition. Missions are no longer hard-wired to a robot: the planner
# decides who takes which task, so this only describes the robots themselves.
AMRS = [
    {
        "name": "amr1",
        "prim": "/World/_23/iw_hub_01",
        "namespace": "",
        "spawn": None,
        "colour": (0.05, 0.85, 0.15),   # green
    },
    {
        "name": "amr2",
        "prim": "/World/iw_hub_02",
        "namespace": "amr2",
        "spawn": (-29.86, -15.57, 0.0),
        "colour": (1.0, 0.10, 0.55),    # pink
    },
    {
        "name": "amr3",
        "prim": "/World/iw_hub_03",
        "namespace": "amr3",
        "spawn": (-9.00, 6.751, 0.0),
        "colour": (0.15, 0.55, 1.0),    # blue
    },
]

# Transport jobs: carry <dolly> from <pickup> to <dropoff>.
# Destinations come from an exhaustive search over the waypoint graph, keeping
# only sets where every one of these holds:
#   * all four storage corners are used
#   * no Dolly is dropped somewhere another robot still has to drive through
#   * the optimal plan needs no shared aisle (conflict-free)
#   * every AMR crosses at least 40 m north-south, so the run looks alive
#   * naive dispatch is clearly worse than the optimum
TASKS = [
    {"id": "T1", "dolly": "/World/dolly_physics_01", "pickup": 10, "dropoff": 12},
    {"id": "T2", "dolly": "/World/dolly_physics", "pickup": 10, "dropoff": 9},
    {"id": "T3", "dolly": "/World/dolly_physics_03", "pickup": 11, "dropoff": 7},
    {"id": "T4", "dolly": "/World/dolly_physics_02", "pickup": 11, "dropoff": 13},
    {"id": "T5", "dolly": "/World/dolly_physics_04", "pickup": 4, "dropoff": 8},
    {"id": "T6", "dolly": "/World/dolly_physics_07", "pickup": 6, "dropoff": 2},
    # Seventh task so the fleet splits 3/2/2 instead of 3/2/1.
    #
    # With six tasks the optimiser left amr3 carrying only T6, which reads as an
    # idle robot however good the makespan is. This one starts from the spare
    # Dolly at the same park as T6 (dolly_physics_06) and ends at Node_5, which
    # is where amr3 spawns, so it pairs naturally with T6 rather than pulling
    # another robot across the factory.
    {"id": "T7", "dolly": "/World/dolly_physics_06", "pickup": 6, "dropoff": 5},
]

# "auto" lets the planner choose; "manual" reproduces the hand-written baseline.
PLAN_SOLVER = os.environ.get("PLAN_SOLVER", "auto")
# Round-robin dispatch, the simplest thing a scheduler would do.
MANUAL_ASSIGNMENT = {
    "amr1": ["T1", "T4"],
    "amr2": ["T2", "T5"],
    "amr3": ["T6", "T7"],
}

# The vision-docking demo runs a two-robot fleet so both AMRs stay on screen and
# each one docks from a camera the operator can actually watch. FLEET/TASK_IDS
# restore the full three-robot, six-task run without touching the code.
FLEET = [
    name.strip()
    for name in os.environ.get("FLEET", "amr1,amr2").split(",")
    if name.strip()
]
TASK_IDS = [
    task_id.strip()
    for task_id in os.environ.get("TASK_IDS", "T1,T2,T3,T4,T5,T6,T7").split(",")
    if task_id.strip()
]

AMRS = [spec for spec in AMRS if spec["name"] in FLEET]
TASKS = [task for task in TASKS if task["id"] in TASK_IDS]
if not AMRS or not TASKS:
    raise RuntimeError(f"FLEET={FLEET} / TASK_IDS={TASK_IDS} selected nothing")
# Keep the round-robin baseline consistent with whatever subset is active,
# otherwise the planner comparison would score against phantom missions.
MANUAL_ASSIGNMENT = {spec["name"]: [] for spec in AMRS}
for _index, _task in enumerate(TASKS):
    MANUAL_ASSIGNMENT[AMRS[_index % len(AMRS)]["name"]].append(_task["id"])
print(
    f"[FLEET] amrs={[spec['name'] for spec in AMRS]} "
    f"tasks={[task['id'] for task in TASKS]} baseline={MANUAL_ASSIGNMENT}",
    flush=True,
)

AMR_SPAWN_Z = float(os.environ.get("AMR_SPAWN_Z", "0.35"))
# Multiplier applied to an edge another mission already claimed.
ROUTE_CONFLICT_PENALTY = float(os.environ.get("ROUTE_CONFLICT_PENALTY", "25.0"))


def spawn_amr(stage, prim_path, x, y, yaw_deg):
    """Add another IW Hub by payloading the same asset the authored one uses."""
    if stage.GetPrimAtPath(prim_path).IsValid():
        print(f"[AMR SPAWN] {prim_path} already present", flush=True)
        return prim_path

    xform = UsdGeom.Xform.Define(stage, prim_path)
    xform.GetPrim().GetPayloads().AddPayload(IW_HUB_ASSET)
    matrix = Gf.Matrix4d(1.0).SetRotate(
        Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), float(yaw_deg))
    )
    matrix.SetTranslateOnly(Gf.Vec3d(float(x), float(y), AMR_SPAWN_Z))
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(matrix)
    print(
        f"[AMR SPAWN] {prim_path} at ({x:.3f}, {y:.3f}, {AMR_SPAWN_Z:.3f}) "
        f"yaw={yaw_deg:.1f} payload={os.path.basename(IW_HUB_ASSET)}",
        flush=True,
    )
    return prim_path


def build_mission_dock(stage, robot_path, dolly_path, start_node):
    """Pre-dock/dock poses that approach the Dolly from the start node's side."""
    dolly = stage.GetPrimAtPath(dolly_path)
    dolly_pose = world_pose_of_prim(dolly)
    dolly_bounds = world_bounds_of_prim(dolly)

    robot_pose = world_pose_of_prim(stage.GetPrimAtPath(robot_path))
    chassis_bounds = world_bounds_of_prim(
        stage.GetPrimAtPath(f"{robot_path}/chassis")
    )
    robot_yaw = math.radians(robot_pose["yaw_deg"])
    offset_x = chassis_bounds["center"][0] - robot_pose["x"]
    offset_y = chassis_bounds["center"][1] - robot_pose["y"]
    body_forward = offset_x * math.cos(robot_yaw) + offset_y * math.sin(robot_yaw)
    body_left = -offset_x * math.sin(robot_yaw) + offset_y * math.cos(robot_yaw)

    approach = 1.5
    best = None
    for heading_deg in (dolly_pose["yaw_deg"], dolly_pose["yaw_deg"] + 180.0):
        heading = math.radians(heading_deg)
        dock_x = dolly_bounds["center"][0] - (
            body_forward * math.cos(heading) - body_left * math.sin(heading)
        )
        dock_y = dolly_bounds["center"][1] - (
            body_forward * math.sin(heading) + body_left * math.cos(heading)
        )
        pre_x = dock_x - approach * math.cos(heading)
        pre_y = dock_y - approach * math.sin(heading)
        # Enter from whichever side the AMR is already coming from.
        cost = math.hypot(pre_x - start_node["x"], pre_y - start_node["y"])
        candidate = {
            "heading_deg": normalize_deg(heading_deg),
            "dock": {"x": dock_x, "y": dock_y, "yaw_deg": normalize_deg(heading_deg)},
            "pre_dock": {
                "x": pre_x,
                "y": pre_y,
                "yaw_deg": normalize_deg(heading_deg),
            },
            "cost": cost,
        }
        if best is None or cost < best["cost"]:
            best = candidate

    return {
        "dolly_path": dolly_path,
        "dolly_pose": dolly_pose,
        "dolly_center": dolly_bounds["center"],
        "chassis_offset": {"forward": body_forward, "left": body_left},
        "pre_dock": best["pre_dock"],
        "dock": best["dock"],
    }


def normalize_deg(value):
    return (float(value) + 180.0) % 360.0 - 180.0


def ensure_runtime_floor(stage):
    """Add the floor collider the factory USD is missing east of x ~= -17.25.

    Raycasts show the authored floor colliders stop at x = -17.25, so every node
    east of it (including the Node3 destination) has visible floor geometry but
    nothing to drive on. This adds an invisible static collider at the real floor
    height (z = 0) for that region only; the original USD is never saved.
    """
    # An analytic infinite plane at the authored floor height. A scaled Cube was
    # tried first, but the wheels tunnelled ~2 cm into it at cruise speed and the
    # AMR jammed; an infinite plane has no such contact-precision problem and is
    # seamless, so the missing-collider region behaves like the rest of the floor.
    path = "/World/RuntimeFloorCollider"
    try:
        ground = UsdGeom.Plane.Define(stage, path)
        ground.CreateAxisAttr("Z")
        collider = ground.GetPrim()
        kind = "infinite plane"
    except AttributeError:
        ground = UsdGeom.Cube.Define(stage, path)
        ground.CreateSizeAttr(2.0)
        xformable = UsdGeom.Xformable(ground)
        xformable.ClearXformOpOrder()
        matrix = Gf.Matrix4d(1.0)
        matrix.SetScale(Gf.Vec3d(40.0, 40.0, 0.25))
        matrix.SetTranslateOnly(Gf.Vec3d(-16.0, 0.0, FLOOR_Z - 0.25))
        xformable.AddTransformOp().Set(matrix)
        collider = ground.GetPrim()
        kind = "box fallback"
    UsdPhysics.CollisionAPI.Apply(collider)
    UsdGeom.Imageable(collider).MakeInvisible()
    cube = ground

    # Without an explicit physics material the default has no friction, so the
    # wheels just spin in place once the AMR rolls onto this collider.
    material_path = "/World/RuntimeFloorPhysicsMaterial"
    material = UsdShade.Material.Define(stage, material_path)
    UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material = UsdPhysics.MaterialAPI(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(0.9)
    physics_material.CreateDynamicFrictionAttr().Set(0.9)
    physics_material.CreateRestitutionAttr().Set(0.0)
    binding = UsdShade.MaterialBindingAPI.Apply(cube.GetPrim())
    binding.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")

    print(
        f"[RUNTIME FLOOR] {path} kind={kind} top_z={FLOOR_Z} friction=0.9 "
        "(invisible static collider)",
        flush=True,
    )


def neutralize_dolly_physics(stage, dolly_path):
    """Turn the carry target into a render-only prim before physics starts.

    The Dolly is a PhysX articulation, so per-link `rigidBodyEnabled` toggles are
    ignored and driving its root transform each frame fights the solver, which
    previously ejected the AMR through the floor. Stripping the physics API
    schemas (visuals and materials untouched) makes the follower safe.
    """
    dolly = stage.GetPrimAtPath(dolly_path)
    if not dolly.IsValid():
        print(f"[DOLLY PHYSICS] target not found: {dolly_path}", flush=True)
        return

    stripped, deactivated = 0, 0
    for prim in Usd.PrimRange(dolly):
        if "Joint" in prim.GetTypeName():
            prim.SetActive(False)
            deactivated += 1
            continue
        applied = list(prim.GetAppliedSchemas())
        keep = [
            schema
            for schema in applied
            if not (schema.startswith("Physics") or schema.startswith("Physx"))
        ]
        if len(keep) != len(applied):
            prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(keep))
            stripped += 1

    print(
        f"[DOLLY PHYSICS] {dolly_path} is now render-only "
        f"(prims_stripped={stripped}, joints_deactivated={deactivated})",
        flush=True,
    )


def set_dolly_dynamics(stage, dolly, enabled):
    for prim in Usd.PrimRange(dolly):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            attr = UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr()
            if attr.IsValid():
                attr.Set(enabled)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            attr = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
            if attr.IsValid():
                attr.Set(enabled)


def ideal_dolly_relative_transform(stage, chassis, dolly, lift_height=0.0):
    """Snap the Dolly centered on the chassis, yaw aligned, wheels on the floor.

    The USD parks this Dolly 28 cm above the floor, and stripping its physics
    means it never settles, so the snap has to put its lowest geometry exactly on
    the floor. `lift_height` is added on top once the lift actually raises.
    """
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    chassis_world = cache.GetLocalToWorldTransform(chassis)
    dolly_world = cache.GetLocalToWorldTransform(dolly)
    chassis_bound = world_bounds_of_prim(chassis)
    dolly_bound = world_bounds_of_prim(dolly)

    chassis_yaw = yaw_deg_from_world_matrix(chassis_world)
    dolly_yaw = yaw_deg_from_world_matrix(dolly_world)
    yaw_delta = chassis_yaw - dolly_yaw

    current_center = Gf.Vec3d(*dolly_bound["center"])
    # Drop the Dolly so its lowest point rests on the floor, then apply the lift.
    ground_drop = (FLOOR_Z + float(lift_height)) - float(dolly_bound["min"][2])
    desired_center = Gf.Vec3d(
        float(chassis_bound["center"][0]),
        float(chassis_bound["center"][1]),
        float(dolly_bound["center"][2]) + ground_drop,
    )

    # Rotate the Dolly about its own center, then move that center on target.
    to_origin = Gf.Matrix4d(1.0).SetTranslate(-current_center)
    rotation = Gf.Matrix4d(1.0).SetRotate(
        Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), yaw_delta)
    )
    back = Gf.Matrix4d(1.0).SetTranslate(desired_center)
    if MATRIX_ROW_VECTOR:
        desired_world = dolly_world * to_origin * rotation * back
    else:
        desired_world = back * rotation * to_origin * dolly_world

    relative = relative_of(desired_world, chassis_world)
    print(
        "[DOCK GEOMETRY] "
        f"chassis_center={chassis_bound['center']} chassis_size={chassis_bound['size']} "
        f"dolly_center={dolly_bound['center']} dolly_size={dolly_bound['size']} "
        f"chassis_yaw={chassis_yaw:.2f} dolly_yaw={dolly_yaw:.2f} "
        f"yaw_delta={yaw_delta:.2f} desired_center={desired_center} "
        f"dolly_bottom={dolly_bound['min'][2]:.4f} ground_drop={ground_drop:.4f} "
        f"lift_height={lift_height:.3f}",
        flush=True,
    )
    return relative


def ground_dolly_to_floor(stage, dolly_path):
    """Drop the Dolly so its lowest geometry rests on the floor.

    The USD parks this Dolly 28 cm in the air and stripping its physics means it
    never settles, so it has to be placed explicitly.
    """
    dolly = stage.GetPrimAtPath(dolly_path)
    if not dolly.IsValid():
        return
    bounds = world_bounds_of_prim(dolly)
    drop = FLOOR_Z - float(bounds["min"][2])
    if abs(drop) < 1e-6:
        return
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world = cache.GetLocalToWorldTransform(dolly)
    parent_world = cache.GetParentToWorldTransform(dolly)
    grounded = Gf.Matrix4d(world)
    grounded.SetTranslateOnly(
        world.ExtractTranslation() + Gf.Vec3d(0.0, 0.0, drop)
    )
    local = local_from_world(grounded, parent_world)
    xformable = UsdGeom.Xformable(dolly)
    ops = xformable.GetOrderedXformOps()
    if len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform:
        ops[0].Set(local)
    else:
        xformable.ClearXformOpOrder()
        xformable.AddTransformOp().Set(local)
    print(
        f"[DOLLY GROUNDED] {dolly_path} drop={drop:.4f} "
        f"bottom={world_bounds_of_prim(dolly)['min'][2]:.4f} "
        f"pose={world_pose_of_prim(dolly)}",
        flush=True,
    )


ROUTE_VISUAL_PATH = "/World/MissionRoutePath"


def set_route_visual_visible(stage, visible, root_path=ROUTE_VISUAL_PATH):
    """Reveal or hide the planned-route strip."""
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return
    imageable = UsdGeom.Imageable(root)
    if visible:
        imageable.MakeVisible()
    else:
        imageable.MakeInvisible()
    print(
        f"[ROUTE VISUAL] {root_path.rsplit('/', 1)[-1]} "
        f"{'shown' if visible else 'hidden'}",
        flush=True,
    )


def create_route_visual(
    stage, route, nodes_by_id, root_path=ROUTE_VISUAL_PATH, colour=(1.0, 0.42, 0.0)
):
    """Lay an orange, collision-free guide strip along the planned route.

    Created hidden: it is revealed once the Dolly is actually picked up, so the
    approach phase is not cluttered by a path the AMR is not following yet.
    """
    if stage.GetPrimAtPath(root_path).IsValid():
        stage.RemovePrim(root_path)
    UsdGeom.Xform.Define(stage, root_path)

    colour = Gf.Vec3f(*colour)
    width, thickness, height = 0.45, 0.02, 0.012
    segments = 0
    for index in range(len(route) - 1):
        start = nodes_by_id.get(route[index])
        end = nodes_by_id.get(route[index + 1])
        if not start or not end:
            continue
        sx, sy = float(start["x"]), float(start["y"])
        ex, ey = float(end["x"]), float(end["y"])
        length = math.hypot(ex - sx, ey - sy)
        if length < 1e-6:
            continue
        strip = UsdGeom.Cube.Define(stage, f"{root_path}/Segment_{index}")
        strip.CreateSizeAttr(2.0)
        strip.CreateDisplayColorAttr([colour])
        xformable = UsdGeom.Xformable(strip)
        xformable.ClearXformOpOrder()
        matrix = Gf.Matrix4d(1.0)
        matrix.SetScale(Gf.Vec3d(length * 0.5, width * 0.5, thickness * 0.5))
        matrix = matrix * Gf.Matrix4d(1.0).SetRotate(
            Gf.Rotation(
                Gf.Vec3d(0.0, 0.0, 1.0),
                math.degrees(math.atan2(ey - sy, ex - sx)),
            )
        )
        matrix.SetTranslateOnly(
            Gf.Vec3d((sx + ex) * 0.5, (sy + ey) * 0.5, height)
        )
        xformable.AddTransformOp().Set(matrix)
        segments += 1

    UsdGeom.Imageable(stage.GetPrimAtPath(root_path)).MakeInvisible()
    print(
        f"[ROUTE VISUAL] {root_path} segments={segments} rgb={tuple(colour)} "
        "(no collider, hidden until pickup)",
        flush=True,
    )



RESET_REQUEST_FILE = Path(
    os.environ.get("SIM_RESET_FILE", "/tmp/sim_reset_request")
)


def capture_reset_state(stage, dolly_paths, amr_paths):
    """Record where everything starts, so a run can be repeated exactly.

    The evaluation asks that Play/Stop return the scene to the same initial
    state, and that a completed run can be run again. Neither held: a Dolly
    delivered to its drop node stays there, so restarting the controllers sends
    a robot to fetch something that is no longer present. That was observed as
    a snapshot reporting "no blue region" - the detector was right, the scene
    was stale.

    Local transforms are stored rather than world poses. Writing a world pose
    back requires knowing the parent transform at the time of the write, and
    the AMR chassis moves under its own parent; the local matrix is what the
    prim actually holds and can be restored without that dependency.
    """
    state = {}
    for path in list(dolly_paths) + list(amr_paths):
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        xformable = UsdGeom.Xformable(prim)
        state[path] = Gf.Matrix4d(xformable.GetLocalTransformation())
    print(f"[RESET] captured initial pose of {len(state)} prim(s)", flush=True)
    return state


def restore_reset_state(stage, state):
    """Put every recorded prim back where it started."""
    restored = 0
    for path, matrix in state.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        xformable = UsdGeom.Xformable(prim)
        ops = xformable.GetOrderedXformOps()
        if len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform:
            ops[0].Set(matrix)
        else:
            xformable.ClearXformOpOrder()
            xformable.AddTransformOp().Set(matrix)
        restored += 1
    print(f"[RESET] restored {restored} prim(s) to their initial pose", flush=True)
    return restored


def poll_reset_request():
    """True once per request. File based, for the same reason the Dolly
    commands are: the bridge is not an rclpy node, so it cannot subscribe.
    """
    if not RESET_REQUEST_FILE.exists():
        return False
    try:
        RESET_REQUEST_FILE.unlink()
    except OSError:
        return False
    return True


def freeze_target_dolly(stage, dolly_path=None):
    dolly_path = dolly_path or PICKUP_DOLLY_PATH
    dolly = stage.GetPrimAtPath(dolly_path)
    if not dolly.IsValid():
        return
    set_dolly_dynamics(stage, dolly, False)

    bounds = world_bounds_of_prim(dolly)
    drop = FLOOR_Z - float(bounds["min"][2])
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world = cache.GetLocalToWorldTransform(dolly)
    parent_world = cache.GetParentToWorldTransform(dolly)
    grounded = Gf.Matrix4d(world)
    grounded.SetTranslateOnly(
        world.ExtractTranslation() + Gf.Vec3d(0.0, 0.0, drop)
    )
    local = local_from_world(grounded, parent_world)
    xformable = UsdGeom.Xformable(dolly)
    ops = xformable.GetOrderedXformOps()
    if len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform:
        ops[0].Set(local)
    else:
        xformable.ClearXformOpOrder()
        xformable.AddTransformOp().Set(local)

    print(
        f"[DOLLY FREEZE] path={dolly_path} dropped_to_floor drop={drop:.4f} "
        f"pose={world_pose_of_prim(dolly)} "
        f"bottom={world_bounds_of_prim(dolly)['min'][2]:.4f}",
        flush=True,
    )


_last_dolly_seq = {}
_dolly_poll_debug = 0


def poll_dolly_command(graph_path=None):
    """Edge-detect the newest /dolly_cmd message coming from PC2.

    The ROS 2 subscriber node latches its last value, so the publisher sends
    position = [command_code, sequence] and only a new sequence is acted on.
    """
    global _dolly_poll_debug
    graph_path = graph_path or GRAPH_PATH
    try:
        values = og.Controller.attribute(
            f"{graph_path}/SubscribeDollyCmd.outputs:positionCommand"
        ).get()
    except Exception as exc:
        if _dolly_poll_debug < 3:
            _dolly_poll_debug += 1
            print(f"[DOLLY CMD ERROR] {type(exc).__name__}: {exc}", flush=True)
        return None
    if values is None or len(values) < 2:
        if _dolly_poll_debug < 3:
            _dolly_poll_debug += 1
            print(f"[DOLLY CMD RAW] value={values!r}", flush=True)
        return None

    code = int(round(float(values[0])))
    sequence = int(round(float(values[1])))
    if _last_dolly_seq.get(graph_path) == sequence:
        return None
    _last_dolly_seq[graph_path] = sequence
    print(
        f"[DOLLY CMD] {graph_path.rsplit('/', 1)[-1]} "
        f"{DOLLY_CMD_NAMES.get(code, code)} seq={sequence}",
        flush=True,
    )
    return code


def apply_follower_transform(stage, chassis, dolly, relative, lift_offset=0.0):
    """Drive the Dolly transform from the chassis for the current frame."""
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    chassis_world = cache.GetLocalToWorldTransform(chassis)
    dolly_parent_world = cache.GetParentToWorldTransform(dolly)
    world_matrix = compose_world(relative, chassis_world)
    if lift_offset:
        raised = Gf.Matrix4d(world_matrix)
        raised.SetTranslateOnly(
            world_matrix.ExtractTranslation() + Gf.Vec3d(0.0, 0.0, float(lift_offset))
        )
        world_matrix = raised
    local_matrix = local_from_world(world_matrix, dolly_parent_world)
    dolly_xform = UsdGeom.Xformable(dolly)
    ops = dolly_xform.GetOrderedXformOps()
    if len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform:
        ops[0].Set(local_matrix)
    else:
        dolly_xform.ClearXformOpOrder()
        dolly_xform.AddTransformOp().Set(local_matrix)


def place_dolly_flat(stage, dolly, reference_world, x, y, yaw_deg):
    """Set the Dolly down level on the floor at (x, y).

    The follower mirrors the chassis, so if the AMR ends up tilted or climbing
    something the Dolly would inherit that pose. Releasing from a stored level
    reference keeps the drop clean no matter what the AMR is doing.
    """
    reference_yaw = yaw_deg_from_world_matrix(reference_world)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    # Rotate the level reference about its own centre, then move it into place.
    UsdGeom.Xformable(dolly).GetOrderedXformOps()
    parent_world = cache.GetParentToWorldTransform(dolly)
    centre_local = Gf.Vec3d(0.0, 0.0, 0.0)
    world = Gf.Matrix4d(reference_world)
    to_origin = Gf.Matrix4d(1.0).SetTranslate(-world.ExtractTranslation())
    rotation = Gf.Matrix4d(1.0).SetRotate(
        Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), float(yaw_deg) - reference_yaw)
    )
    back = Gf.Matrix4d(1.0).SetTranslate(world.ExtractTranslation())
    if MATRIX_ROW_VECTOR:
        levelled = world * to_origin * rotation * back
    else:
        levelled = back * rotation * to_origin * world

    # Drop it at the requested spot, then rest its lowest point on the floor.
    translation = levelled.ExtractTranslation()
    levelled.SetTranslateOnly(Gf.Vec3d(float(x), float(y), translation[2]))
    _write_dolly_local(dolly, local_from_world(levelled, parent_world))

    bounds = world_bounds_of_prim(dolly)
    drop = FLOOR_Z - float(bounds["min"][2])
    if abs(drop) > 1e-6:
        translation = levelled.ExtractTranslation()
        levelled.SetTranslateOnly(translation + Gf.Vec3d(0.0, 0.0, drop))
        _write_dolly_local(dolly, local_from_world(levelled, parent_world))


def _write_dolly_local(dolly, local_matrix):
    xformable = UsdGeom.Xformable(dolly)
    ops = xformable.GetOrderedXformOps()
    if len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform:
        ops[0].Set(local_matrix)
    else:
        xformable.ClearXformOpOrder()
        xformable.AddTransformOp().Set(local_matrix)


def update_pickup_follower(
    stage, robot_path, lift_path, follower_state, command=None, dolly_path=None
):
    """Follow the Dolly from the chassis without relying on PhysX contact."""
    dolly_path = dolly_path or PICKUP_DOLLY_PATH
    lift = stage.GetPrimAtPath(lift_path)
    dolly = stage.GetPrimAtPath(dolly_path)
    chassis = stage.GetPrimAtPath(f"{robot_path}/chassis")
    if not lift.IsValid() or not dolly.IsValid() or not chassis.IsValid():
        return follower_state

    attached = follower_state is not None
    if not attached and command != DOLLY_CMD_ATTACH:
        return None

    if not attached:
        chassis_pose = world_pose_of_prim(chassis)
        dolly_pose = world_pose_of_prim(dolly)
        pickup_distance = math.hypot(
            chassis_pose["x"] - dolly_pose["x"],
            chassis_pose["y"] - dolly_pose["y"],
        )
        # Grounded snap: docking finishes with both the AMR and the Dolly on the
        # floor. The lift offset is applied later, when the lift really rises.
        relative = ideal_dolly_relative_transform(stage, chassis, dolly, 0.0)
        set_dolly_dynamics(stage, dolly, False)
        apply_follower_transform(stage, chassis, dolly, relative, 0.0)
        print(
            f"[PICKUP ATTACH] dolly={dolly_path} mode=runtime_follower "
            f"approach_distance={pickup_distance:.3f} m grounded "
            f"dolly_before={dolly_pose} dolly_after={world_pose_of_prim(dolly)}",
            flush=True,
        )
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        level_reference = cache.GetLocalToWorldTransform(dolly)
        return (relative, 0.0, 0.0, time.monotonic(), False, level_reference)

    (
        relative,
        lift_offset,
        target_offset,
        last_time,
        release_pending,
        level_reference,
    ) = follower_state

    if command == DOLLY_CMD_LIFT:
        target_offset = CARRY_LIFT_HEIGHT
        print(f"[DOLLY LIFT] raising to {target_offset:.3f} m", flush=True)
    elif command == DOLLY_CMD_LOWER:
        target_offset = 0.0
        print("[DOLLY LIFT] lowering to floor", flush=True)
    elif command == DOLLY_CMD_RELEASE:
        # Latch it: the Dolly must finish coming down before we let go.
        release_pending = True

    # Ease the Dolly toward the target height instead of snapping to it.
    current_time = time.monotonic()
    step = CARRY_LIFT_HEIGHT * max(0.0, current_time - last_time) / LIFT_RAMP_SEC
    if lift_offset < target_offset:
        lift_offset = min(target_offset, lift_offset + step)
    elif lift_offset > target_offset:
        lift_offset = max(target_offset, lift_offset - step)

    if release_pending and abs(lift_offset) <= 1e-4:
        apply_follower_transform(stage, chassis, dolly, relative, 0.0)
        here = world_pose_of_prim(dolly)
        place_dolly_flat(
            stage, dolly, level_reference, here["x"], here["y"], here["yaw_deg"]
        )
        print(
            f"[PICKUP DETACH] follower stopped, dolly left at "
            f"{world_pose_of_prim(dolly)}",
            flush=True,
        )
        return None

    apply_follower_transform(stage, chassis, dolly, relative, lift_offset)
    return (
        relative,
        lift_offset,
        target_offset,
        current_time,
        release_pending,
        level_reference,
    )


def inspect_lift_joint(stage, robot_path):
    """Read the lift joint limits and drive state from the loaded USD."""
    path = f"{robot_path}/lift_joint"
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"lift_joint not found: {path}")

    lower = prim.GetAttribute("physics:lowerLimit").Get()
    upper = prim.GetAttribute("physics:upperLimit").Get()
    target = prim.GetAttribute("drive:linear:physics:targetPosition").Get()
    if lower is None or upper is None:
        raise RuntimeError(f"lift_joint has no finite USD limits: {path}")

    print(
        "[LIFT] "
        f"type={prim.GetTypeName()}, axis="
        f"{prim.GetAttribute('physics:axis').Get()}, "
        f"lower={float(lower):.6f}, upper={float(upper):.6f}, "
        f"current_target={float(target or 0.0):.6f}"
    )
    return float(lower), float(upper)


def attach_iw_hub_sensors(stage, robot_path):
    """Reference only the docking camera, not the asset's duplicate robot joints."""
    if not os.path.isfile(SENSORS_USD):
        raise FileNotFoundError(f"Sensor USD not found: {SENSORS_USD}")

    sensor_root = f"{robot_path}/iw_hub_sensors"
    mount_root = f"{sensor_root}/camera_mount"
    camera_path = f"{sensor_root}/camera_mount/{CAMERA_NAME}"

    stage.DefinePrim(sensor_root, "Xform")
    stage.DefinePrim(mount_root, "Xform")
    camera = stage.GetPrimAtPath(camera_path)
    if not camera or not camera.IsValid():
        camera = stage.DefinePrim(camera_path, "Camera")
        camera.GetReferences().AddReference(
            SENSORS_USD,
            f"/Root/iw_hub_sensors/camera_mount/{CAMERA_NAME}",
        )

    if camera.IsValid() and camera.IsActive():
        return camera_path

    raise RuntimeError(
        f"Referenced sensor asset did not create camera Prim: {camera_path}"
    )


def disable_old_ros_graphs(stage):
    """
    백업 USD 안에 남아 있는 기존 OmniGraph들을 런타임에서 비활성화한다.

    이전 멀티-AMR Graph 이름이 무엇이든 OmniGraph 타입이면 제거 대상으로 잡고,
    새 StandaloneROSBridge 계열만 제외한다.

    WaypointGraph처럼 단순 Xform인 Prim은 비활성화하지 않는다.
    """
    candidates = []

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        name_lower = prim.GetName().lower()
        type_lower = str(prim.GetTypeName()).lower()

        # 우리가 새로 만들 브리지 영역은 절대 건드리지 않는다.
        if (
            path == GRAPH_PATH
            or path.startswith(GRAPH_PATH + "/")
            or path.startswith(GRAPH_PATH + "_")
        ):
            continue

        is_omnigraph = "omnigraph" in type_lower

        # 타입 정보가 불완전한 USD를 위한 fallback
        legacy_name = (
            "actiongraph" in name_lower
            or "ros_lidar" in name_lower
            or "roslidar" in name_lower
        )

        if is_omnigraph or legacy_name:
            candidates.append(path)

    # 부모 Graph만 비활성화하면 하위 Node도 함께 꺼진다.
    candidates = sorted(
        set(candidates),
        key=lambda p: (p.count("/"), p),
    )

    roots = []
    for path in candidates:
        if any(path.startswith(root + "/") for root in roots):
            continue
        roots.append(path)

    disabled = []

    for path in roots:
        prim = stage.GetPrimAtPath(path)

        if prim and prim.IsValid() and prim.IsActive():
            # Deactivating the USD prim does not always tear down an already
            # instantiated OmniGraph runtime. Remove those nodes explicitly
            # so their ROS endpoints cannot remain advertised.
            try:
                graph = og.get_graph_by_path(path)
                if graph:
                    nodes = graph.get_nodes()
                    if nodes:
                        og.GraphController.delete_node(nodes, undoable=False)
            except Exception as exc:
                raise RuntimeError(
                    f"Failed to tear down legacy OmniGraph runtime: {path}"
                ) from exc

            prim.SetActive(False)
            disabled.append(path)

    # Do not start a second ROS graph if a USD variant or an unusual graph
    # type escaped the candidate scan above.
    active_legacy = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if (
            path == GRAPH_PATH
            or path.startswith(GRAPH_PATH + "/")
            or path.startswith(GRAPH_PATH + "_")
        ):
            continue

        name_lower = prim.GetName().lower()
        type_lower = str(prim.GetTypeName()).lower()
        if (
            "omnigraph" in type_lower
            or "actiongraph" in name_lower
            or "ros_lidar" in name_lower
            or "roslidar" in name_lower
        ) and prim.IsActive():
            active_legacy.append(path)

    if active_legacy:
        raise RuntimeError(
            "Active legacy OmniGraph(s) remain after deactivation: "
            + ", ".join(sorted(active_legacy))
        )

    return disabled


def find_robot_root(stage):
    """
    IW Hub root를 자동 탐색한다.
    먼저 알려진 경로를 보고, 없으면 이름 prefix로 찾는다.
    """
    preferred = [
        "/World/_23/iw_hub_01",
        "/World/_23/iw_hub_02",
        "/World/_23/iw_hub",
        "/World/iw_hub",
        "/World/iw_hub_01",
        "/World/iw_hub_02",
    ]

    for path in preferred:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid() and prim.IsActive():
            return path

    candidates = []

    for prim in stage.Traverse():
        if prim.GetName().lower().startswith("iw_hub"):
            candidates.append(str(prim.GetPath()))

    if not candidates:
        raise RuntimeError("Stage에서 iw_hub root Prim을 찾지 못했습니다.")

    candidates.sort(key=lambda p: (p.count("/"), p))
    return candidates[0]


def find_named_prims(stage, exact_name):
    exact_lower = exact_name.lower()
    result = []

    for prim in stage.Traverse():
        if prim.GetName().lower() == exact_lower:
            result.append(str(prim.GetPath()))

    return result


def find_dolly_prims(stage):
    """
    Dolly 루트 후보만 최대한 추린다.
    같은 Dolly 내부 mesh들이 전부 잡히는 것을 피하기 위해
    이름에 dolly가 들어간 Prim 중 부모 이름에도 dolly가 들어가면 제외한다.
    """
    candidates = []

    for prim in stage.Traverse():
        name = prim.GetName().lower()

        if "dolly" not in name:
            continue

        parent = prim.GetParent()
        parent_name = (
            parent.GetName().lower()
            if parent and parent.IsValid()
            else ""
        )

        if "dolly" in parent_name:
            continue

        candidates.append(str(prim.GetPath()))

    return sorted(set(candidates))


def find_waypoint_nodes(stage):
    """
    extension.py와 동일하게 /World/WaypointGraph/Nodes를 우선 사용.
    없으면 Node_<id> 형식을 Stage 전체에서 탐색.
    """
    node_root = stage.GetPrimAtPath("/World/WaypointGraph/Nodes")
    nodes = []

    if node_root and node_root.IsValid():
        for prim in node_root.GetChildren():
            nodes.append(prim)
    else:
        for prim in stage.Traverse():
            name = prim.GetName()
            if name.startswith("Node_"):
                nodes.append(prim)

    def node_sort_key(prim):
        name = prim.GetName()
        tail = name.split("_")[-1]
        return int(tail) if tail.isdigit() else 999999

    return sorted(nodes, key=node_sort_key)


def report_camera_aim(stage, camera_path, runtime):
    """Print where the docking camera really is and what it is pointed at.

    Reasoning about the camera from the robot pose plus the mount constants kept
    disagreeing with the pictures: the geometry said a Dolly filled most of the
    frame while the frame showed an empty aisle. Rather than keep guessing at
    the transform, read the composed world matrix straight off the stage and
    compare it against the target Dolly.

    A USD camera looks down its own -Z axis, so the third row of the
    local-to-world matrix, negated, is the viewing direction.
    """
    camera = stage.GetPrimAtPath(camera_path)
    if not camera.IsValid():
        return
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world = cache.GetLocalToWorldTransform(camera)
    eye = world.ExtractTranslation()
    view = -Gf.Vec3d(world[2][0], world[2][1], world[2][2]).GetNormalized()

    heading_deg = math.degrees(math.atan2(view[1], view[0]))
    pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, view[2]))))

    message = (
        f"[CAMERA AIM] eye=({eye[0]:.3f}, {eye[1]:.3f}, {eye[2]:.3f}) "
        f"heading={heading_deg:+.1f} pitch={pitch_deg:+.1f}"
    )

    for robot in runtime:
        if robot["active"] >= len(robot["missions"]):
            continue
        dolly = stage.GetPrimAtPath(
            robot["missions"][robot["active"]]["dolly_path"]
        )
        if not dolly.IsValid():
            continue
        pose = world_pose_of_prim(dolly)
        to_dolly = Gf.Vec3d(pose["x"] - eye[0], pose["y"] - eye[1], 0.0)
        distance = to_dolly.GetLength()
        bearing = math.degrees(
            math.atan2(to_dolly[1], to_dolly[0])
        ) - heading_deg
        message += (
            f" | {robot['name']} dolly d={distance:.2f} "
            f"bearing={normalize_deg(bearing):+.1f}"
        )
        break

    print(message, flush=True)


def hide_waypoint_graph(stage):
    """Remove the authored waypoint markers so they stop blocking the camera.

    The node markers sit at z = 0.15 m, which is squarely inside the docking
    camera's line of sight - the camera is 0.45 m up and tilted 4 degrees down,
    so a marker a couple of metres ahead lands right where the Dolly should be.
    Node 10 in particular sits directly between amr1 and its pickup Dolly, and
    the detector never saw the Dolly because a marker was in front of it.

    Deactivating the root prim, not just clearing its visibility, is what makes
    the graph actually go away. `MakeInvisible` only authors an opinion on the
    prim it is called on, and the marker meshes underneath carry their own
    authored visibility, so the two Xforms reported themselves hidden while the
    spheres kept rendering. Deactivation prunes the whole subtree out of
    composition instead, which no descendant opinion can override, and it also
    drops the markers from the viewport and the Stage tree.

    Positions and edges have already been read into the inventory and the
    planner graph by this point, so nothing downstream needs the geometry.
    Deactivation is a stage-local edit and the original USD is never written.
    """
    if SHOW_WAYPOINT_GRAPH:
        print("[WAYPOINT GRAPH] left visible (SHOW_WAYPOINT_GRAPH=1)", flush=True)
        return

    root = stage.GetPrimAtPath("/World/WaypointGraph")
    if root.IsValid():
        children = {
            child.GetName(): len(child.GetChildren())
            for child in root.GetChildren()
        }
        root.SetActive(False)
        summary = ", ".join(f"{name}({count})" for name, count in children.items())
        print(
            f"[WAYPOINT GRAPH] deactivated /World/WaypointGraph: "
            f"{summary or 'no children'}",
            flush=True,
        )
        return

    # Older stages scatter the markers instead of parenting them under a single
    # root. Fall back to hiding whatever carries a Node_/Edge_ name.
    hidden = 0
    for prim in stage.Traverse():
        name = prim.GetName()
        if name.startswith("Node_") or name.startswith("Edge_"):
            UsdGeom.Imageable(prim).MakeInvisible()
            hidden += 1
    print(f"[WAYPOINT GRAPH] no root prim; hid {hidden} loose marker(s)", flush=True)



CUOPT_PLAN_PATH = Path(
    os.environ.get("CUOPT_PLAN", str(SCRIPT_DIR / "cuopt_plan.json"))
)


def load_cuopt_plan(graph, tasks, vehicles):
    """Read the assignment cuOpt produced, and score it like any other plan.

    cuOpt is solved out of process. It needs cudf and lives in
    ~/.venvs/cuopt on Python 3.12, while this bridge runs on Isaac Sim's
    Python 3.11; neither interpreter can import the other's packages, and
    putting cudf next to Isaac's CUDA stack is not worth the risk. So
    scripts/plan_cuopt.py solves it and leaves a file here.

    The plan is re-scored through planner._build_result rather than trusted, so
    the makespan printed in the comparison comes from the same cost model as
    the manual and greedy rows. A plan that referenced a task or vehicle this
    run does not have would otherwise show up as a plausible-looking number.
    """
    if not CUOPT_PLAN_PATH.exists():
        raise RuntimeError(
            f"PLAN_SOLVER=cuopt but {CUOPT_PLAN_PATH} is missing. Run:\n"
            "  ~/.venvs/cuopt/bin/python scripts/plan_cuopt.py"
        )

    payload = json.loads(CUOPT_PLAN_PATH.read_text(encoding="utf-8"))
    by_id = {task.task_id: task for task in tasks}
    known = {vehicle.name for vehicle in vehicles}

    buckets = {vehicle.name: [] for vehicle in vehicles}
    missing = []
    for name, task_ids in payload.get("assignment", {}).items():
        if name not in known:
            raise RuntimeError(
                f"cuOpt plan assigns work to unknown vehicle '{name}'; "
                f"this run has {sorted(known)}"
            )
        for task_id in task_ids:
            if task_id not in by_id:
                missing.append(task_id)
                continue
            buckets[name].append(by_id[task_id])

    if missing:
        raise RuntimeError(
            f"cuOpt plan references tasks not in this run: {sorted(missing)}. "
            "Regenerate it with the same TASK_IDS."
        )
    unplanned = sorted(set(by_id) - {t.task_id for g in buckets.values() for t in g})
    if unplanned:
        raise RuntimeError(
            f"cuOpt plan leaves tasks unassigned: {unplanned}"
        )

    print(
        f"[CUOPT] loaded {CUOPT_PLAN_PATH.name}: solved in "
        f"{payload.get('solve_seconds', 0.0) * 1000:.1f} ms",
        flush=True,
    )
    return planner._build_result(
        graph, vehicles, buckets, "cuopt",
        float(payload.get("solve_seconds", 0.0)),
    )


def load_waypoint_graph(stage, nodes_by_id):
    """Build the planner graph from /World/WaypointGraph/Edges."""
    edge_root = stage.GetPrimAtPath("/World/WaypointGraph/Edges")
    edges = []
    if edge_root and edge_root.IsValid():
        for prim in edge_root.GetChildren():
            match = re.fullmatch(r"Edge_(\d+)_(\d+)", prim.GetName())
            if not match:
                continue
            a, b = (int(v) for v in match.groups())
            weight = prim.GetAttribute("weight").Get()
            if weight is None:
                weight = math.dist(
                    (nodes_by_id[a]["x"], nodes_by_id[a]["y"]),
                    (nodes_by_id[b]["x"], nodes_by_id[b]["y"]),
                )
            edges.append((a, b, float(weight)))
    nodes = {i: (n["x"], n["y"]) for i, n in nodes_by_id.items()}
    print(f"[GRAPH] {len(nodes)} nodes, {len(edges)} edges", flush=True)
    return planner.WaypointGraph(nodes, edges)


def route_edges(route):
    """Undirected edge keys used by a route."""
    return {frozenset((route[i], route[i + 1])) for i in range(len(route) - 1)}


def edge_key(edge):
    """Stable text form of an undirected edge, e.g. frozenset({5, 4}) -> '4-5'."""
    a, b = sorted(edge)
    return f"{a}-{b}"


def find_camera_prim(stage, robot_path):
    """
    업로드된 export_camera_intrinsics.py의 카메라 경로를 우선 사용.
    없으면 Camera schema / 이름으로 자동 탐색.
    """
    preferred = [
        f"{robot_path}/iw_hub_sensors/camera_mount/{CAMERA_NAME}",
        "/World/iw_hub_sensors/camera_mount/"
        f"{CAMERA_NAME}",
    ]

    for path in preferred:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid() and prim.IsActive():
            return path

    cameras = []

    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Camera):
            cameras.append(str(prim.GetPath()))

    if cameras:
        cameras.sort(key=lambda p: (p.count("/"), p))
        return cameras[0]

    # fallback name search
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if "camera" in name:
            return str(prim.GetPath())

    return None


def save_inventory_files(data):
    """사람이 읽는 TXT + 이후 Mission 코드가 읽을 JSON을 함께 저장한다."""
    INVENTORY_TXT.parent.mkdir(parents=True, exist_ok=True)
    INVENTORY_JSON.parent.mkdir(parents=True, exist_ok=True)

    lines = []
    lines.append("=" * 78)
    lines.append("FACTORY INVENTORY")
    lines.append("=" * 78)

    amr = data["amr"]
    lines.append("")
    lines.append("[AMR]")
    lines.append(f"path={amr['path']}")
    lines.append(
        f"x={amr['x']:.6f}, y={amr['y']:.6f}, "
        f"z={amr['z']:.6f}, yaw_deg={amr['yaw_deg']:.6f}"
    )

    lines.append("")
    lines.append("[JOINTS]")
    for name, paths in data["joints"].items():
        lines.append(f"{name}={paths if paths else 'NOT FOUND'}")

    lines.append("")
    lines.append("[DOLLIES]")
    if data["dollies"]:
        for item in data["dollies"]:
            lines.append(
                f"{item['index']:02d}. {item['path']} | "
                f"x={item['x']:.6f}, y={item['y']:.6f}, "
                f"z={item['z']:.6f}, yaw_deg={item['yaw_deg']:.6f}"
            )
    else:
        lines.append("NOT FOUND")

    lines.append("")
    lines.append("[WAYPOINT_NODES]")
    if data["nodes"]:
        for item in data["nodes"]:
            lines.append(
                f"{item['name']} | id={item['id']} | "
                f"x={item['x']:.6f}, y={item['y']:.6f}, z={item['z']:.6f}"
            )
    else:
        lines.append("NOT FOUND")

    lines.append("")
    lines.append("[CAMERA]")
    lines.append(data["camera_path"] or "NOT FOUND")

    lines.append("")
    lines.append("[DISABLED_OLD_GRAPHS]")
    if data["disabled_old_graphs"]:
        lines.extend(data["disabled_old_graphs"])
    else:
        lines.append("NONE")

    lines.append("=" * 78)

    INVENTORY_TXT.write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )

    INVENTORY_JSON.write_text(
        json.dumps(data, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    print(f"[OK] Inventory TXT saved : {INVENTORY_TXT}")
    print(f"[OK] Inventory JSON saved: {INVENTORY_JSON}")


def print_stage_inventory(stage, robot_path, disabled_old_graphs=None, camera_path=None):
    print("\n" + "=" * 78)
    print("STAGE INVENTORY")
    print("=" * 78)

    robot_prim = stage.GetPrimAtPath(robot_path)
    robot_pose = world_pose_of_prim(robot_prim)

    print(
        f"[AMR] {robot_path}\n"
        f"      x={robot_pose['x']:.3f}, "
        f"y={robot_pose['y']:.3f}, "
        f"z={robot_pose['z']:.3f}, "
        f"yaw={robot_pose['yaw_deg']:.2f} deg"
    )

    joint_data = {}

    print("\n[JOINT NAME CANDIDATES]")
    for name in (
        "left_wheel_joint",
        "right_wheel_joint",
        "lift_joint",
    ):
        found = find_named_prims(stage, name)
        joint_data[name] = found
        print(f"  {name}: {found if found else 'NOT FOUND'}")

    print("\n[DOLLY CANDIDATES]")
    dolly_paths = find_dolly_prims(stage)
    dolly_data = []

    if not dolly_paths:
        print("  NOT FOUND")
    else:
        for index, path in enumerate(dolly_paths, start=1):
            prim = stage.GetPrimAtPath(path)
            pose = world_pose_of_prim(prim)

            # Size, not just position. The vision standoff has to be measured
            # from the near edge of the Dolly rather than from its centre: the
            # dock target sits underneath the middle of it, so standing "3 m
            # back from the dock" puts the leading edge about 1 m from the lens
            # and it overflows the frame at any field of view. Reading the
            # bound here means the controller never has to hardcode a size that
            # would silently go wrong if the asset changed.
            bounds = world_bounds_of_prim(prim)
            item = {
                "index": index,
                "path": path,
                **pose,
                "size": bounds["size"],
                "half_length_m": max(bounds["size"][0], bounds["size"][1]) / 2.0,
            }
            dolly_data.append(item)

            print(
                f"  {index:02d}. {path} | "
                f"x={pose['x']:.3f}, "
                f"y={pose['y']:.3f}, "
                f"z={pose['z']:.3f}, "
                f"yaw={pose['yaw_deg']:.2f} | "
                f"size=({bounds['size'][0]:.2f}, {bounds['size'][1]:.2f}, "
                f"{bounds['size'][2]:.2f})"
            )

    print("\n[WAYPOINT NODES]")
    node_prims = find_waypoint_nodes(stage)
    node_data = []

    if not node_prims:
        print("  NOT FOUND")
    else:
        for prim in node_prims:
            pose = world_pose_of_prim(prim)

            node_id_attr = prim.GetAttribute("node_id")
            node_id_value = (
                node_id_attr.Get()
                if node_id_attr and node_id_attr.IsValid()
                else None
            )

            if node_id_value is None:
                tail = prim.GetName().split("_")[-1]
                node_id_value = int(tail) if tail.isdigit() else None

            display = (
                f"Node_{node_id_value}"
                if node_id_value is not None
                else prim.GetName()
            )

            item = {
                "id": (
                    int(node_id_value)
                    if node_id_value is not None
                    else None
                ),
                "name": display,
                "path": str(prim.GetPath()),
                **pose,
            }
            node_data.append(item)

            print(
                f"  {display:12s} "
                f"x={pose['x']:.3f}, "
                f"y={pose['y']:.3f}, "
                f"z={pose['z']:.3f}"
            )

    camera_path = camera_path or find_camera_prim(stage, robot_path)

    print("\n[CAMERA]")
    print(f"  {camera_path if camera_path else 'NOT FOUND'}")

    inventory = {
        "factory_usd": FACTORY_USD,
        "amr": {
            "path": robot_path,
            **robot_pose,
        },
        "joints": joint_data,
        "dollies": dolly_data,
        "nodes": node_data,
        "camera_path": camera_path,
        "disabled_old_graphs": list(disabled_old_graphs or []),
    }

    save_inventory_files(inventory)

    print("=" * 78 + "\n")

    # 기존 호출부 호환
    return {
        "dollies": dolly_paths,
        "nodes": node_prims,
        "camera_path": camera_path,
        "inventory": inventory,
    }


# ============================================================
# 4. ROS 2 Bridge graph
# ============================================================

def build_core_bridge_graph(
    robot_path,
    lift_lower,
    lift_upper,
    graph_path=GRAPH_PATH,
    namespace="",
    publish_clock=True,
):
    """
    기존에 검증한 Single-AMR 구성과 같은 구조를 Python으로 만든다.

    /cmd_vel
      -> ROS2 Subscribe Twist
      -> Differential Controller
      -> Articulation Controller

    Robot
      -> Compute Odometry
      -> /odom
      -> odom -> base_link TF

    Simulation Time
      -> /clock
    """

    # Per-AMR topics/frames so several robots can share one ROS graph domain.
    prefix = namespace.strip("/")
    topic = (lambda name: f"/{prefix}{name}") if prefix else (lambda name: name)
    frame = (lambda name: f"{prefix}_{name}") if prefix else (lambda name: name)

    # 동일 경로가 이전 런타임에 남아있을 경우 제거
    stage = omni.usd.get_context().get_stage()
    old_graph = stage.GetPrimAtPath(graph_path)
    if old_graph and old_graph.IsValid():
        stage.RemovePrim(graph_path)

    keys = og.Controller.Keys

    (graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                (
                    "OnPlaybackTick",
                    "omni.graph.action.OnPlaybackTick",
                ),
                (
                    "SubscribeTwist",
                    "isaacsim.ros2.bridge.ROS2SubscribeTwist",
                ),
                (
                    "BreakAngular",
                    "omni.graph.nodes.BreakVector3",
                ),
                (
                    "BreakLinear",
                    "omni.graph.nodes.BreakVector3",
                ),
                (
                    "DifferentialController",
                    "isaacsim.robot.wheeled_robots.DifferentialController",
                ),
                (
                    "ArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                (
                    "SubscribeLift",
                    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                ),
                (
                    "SubscribeDollyCmd",
                    "isaacsim.ros2.bridge.ROS2SubscribeJointState",
                ),
                (
                    "LiftArticulationController",
                    "isaacsim.core.nodes.IsaacArticulationController",
                ),
                (
                    "LiftState",
                    "isaacsim.core.nodes.IsaacArticulationState",
                ),
                (
                    "PublishLiftState",
                    "isaacsim.ros2.bridge.ROS2PublishJointState",
                ),
                (
                    "ComputeOdometry",
                    "isaacsim.core.nodes.IsaacComputeOdometry",
                ),
                (
                    "ReadSimTime",
                    "isaacsim.core.nodes.IsaacReadSimulationTime",
                ),
                (
                    "PublishOdometry",
                    "isaacsim.ros2.bridge.ROS2PublishOdometry",
                ),
                (
                    "PublishClock",
                    "isaacsim.ros2.bridge.ROS2PublishClock",
                ),
                (
                    "PublishOdomTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
                (
                    "PublishBaseFootprintTF",
                    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
                ),
            ],

            keys.CONNECT: [
                # ---- /cmd_vel -> differential drive
                (
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeTwist.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "DifferentialController.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "ArticulationController.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:deltaSeconds",
                    "DifferentialController.inputs:dt",
                ),
                (
                    "SubscribeTwist.outputs:angularVelocity",
                    "BreakAngular.inputs:tuple",
                ),
                (
                    "BreakAngular.outputs:z",
                    "DifferentialController.inputs:angularVelocity",
                ),
                (
                    "SubscribeTwist.outputs:linearVelocity",
                    "BreakLinear.inputs:tuple",
                ),
                (
                    "BreakLinear.outputs:x",
                    "DifferentialController.inputs:linearVelocity",
                ),
                (
                    "DifferentialController.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                # ---- lift position command (independent articulation controller)
                (
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeLift.inputs:execIn",
                ),
                # ---- Dolly pickup command channel from PC2 (no shared filesystem)
                (
                    "OnPlaybackTick.outputs:tick",
                    "SubscribeDollyCmd.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "LiftArticulationController.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "LiftState.inputs:execIn",
                ),
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishLiftState.inputs:execIn",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishLiftState.inputs:timeStamp",
                ),

                # ---- odometry
                (
                    "OnPlaybackTick.outputs:tick",
                    "ComputeOdometry.inputs:execIn",
                ),
                (
                    "ComputeOdometry.outputs:execOut",
                    "PublishOdometry.inputs:execIn",
                ),
                (
                    "ComputeOdometry.outputs:angularVelocity",
                    "PublishOdometry.inputs:angularVelocity",
                ),
                (
                    "ComputeOdometry.outputs:linearVelocity",
                    "PublishOdometry.inputs:linearVelocity",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdometry.inputs:orientation",
                ),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdometry.inputs:position",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdometry.inputs:timeStamp",
                ),

                # ---- clock
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishClock.inputs:execIn",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishClock.inputs:timeStamp",
                ),

                # ---- dynamic TF odom -> base_link
                (
                    "ComputeOdometry.outputs:execOut",
                    "PublishOdomTF.inputs:execIn",
                ),
                (
                    "ComputeOdometry.outputs:position",
                    "PublishOdomTF.inputs:translation",
                ),
                (
                    "ComputeOdometry.outputs:orientation",
                    "PublishOdomTF.inputs:rotation",
                ),
                (
                    "ReadSimTime.outputs:simulationTime",
                    "PublishOdomTF.inputs:timeStamp",
                ),

                # ---- static base_link -> base_footprint
                (
                    "OnPlaybackTick.outputs:tick",
                    "PublishBaseFootprintTF.inputs:execIn",
                ),
            ],

            keys.SET_VALUES: [
                # subscribe
                (
                    "SubscribeTwist.inputs:topicName",
                    topic(CMD_VEL_TOPIC),
                ),
                (
                    "SubscribeLift.inputs:topicName",
                    topic(LIFT_CMD_TOPIC),
                ),
                (
                    "SubscribeDollyCmd.inputs:topicName",
                    topic(DOLLY_CMD_TOPIC),
                ),

                # differential controller
                (
                    "DifferentialController.inputs:wheelRadius",
                    WHEEL_RADIUS,
                ),
                (
                    "DifferentialController.inputs:wheelDistance",
                    WHEEL_DISTANCE,
                ),
                (
                    "DifferentialController.inputs:maxLinearSpeed",
                    MAX_LINEAR_SPEED,
                ),
                (
                    "DifferentialController.inputs:maxAngularSpeed",
                    MAX_ANGULAR_SPEED,
                ),
                (
                    "DifferentialController.inputs:maxWheelSpeed",
                    MAX_WHEEL_SPEED,
                ),

                # articulation controller
                (
                    "ArticulationController.inputs:robotPath",
                    robot_path,
                ),
                (
                    "ArticulationController.inputs:jointNames",
                    [
                        "left_wheel_joint",
                        "right_wheel_joint",
                    ],
                ),
                (
                    "LiftArticulationController.inputs:robotPath",
                    robot_path,
                ),
                (
                    "LiftArticulationController.inputs:jointNames",
                    ["lift_joint"],
                ),
                (
                    "LiftState.inputs:robotPath",
                    robot_path,
                ),
                (
                    "LiftState.inputs:jointNames",
                    ["lift_joint"],
                ),
                (
                    "PublishLiftState.inputs:topicName",
                    topic(LIFT_STATE_TOPIC),
                ),
                (
                    "PublishLiftState.inputs:targetPrim",
                    [robot_path],
                ),

                # odometry
                (
                    "ComputeOdometry.inputs:chassisPrim",
                    [robot_path],
                ),
                (
                    "PublishOdometry.inputs:topicName",
                    topic(ODOM_TOPIC),
                ),
                (
                    "PublishOdometry.inputs:odomFrameId",
                    frame(ODOM_FRAME),
                ),
                (
                    "PublishOdometry.inputs:chassisFrameId",
                    frame(BASE_FRAME),
                ),

                # clock
                (
                    "PublishClock.inputs:topicName",
                    CLOCK_TOPIC if publish_clock else topic("/clock_unused"),
                ),

                # dynamic tf
                (
                    "PublishOdomTF.inputs:topicName",
                    TF_TOPIC,
                ),
                (
                    "PublishOdomTF.inputs:parentFrameId",
                    frame(ODOM_FRAME),
                ),
                (
                    "PublishOdomTF.inputs:childFrameId",
                    frame(BASE_FRAME),
                ),
                (
                    "PublishOdomTF.inputs:staticPublisher",
                    False,
                ),

                # base footprint tf
                (
                    "PublishBaseFootprintTF.inputs:topicName",
                    TF_STATIC_TOPIC,
                ),
                (
                    "PublishBaseFootprintTF.inputs:parentFrameId",
                    frame(BASE_FRAME),
                ),
                (
                    "PublishBaseFootprintTF.inputs:childFrameId",
                    frame(BASE_FOOTPRINT_FRAME),
                ),
                (
                    "PublishBaseFootprintTF.inputs:translation",
                    [0.0, 0.0, 0.0],
                ),
                (
                    "PublishBaseFootprintTF.inputs:rotation",
                    [1.0, 0.0, 0.0, 0.0],
                ),
                (
                    "PublishBaseFootprintTF.inputs:staticPublisher",
                    True,
                ),
            ],
        },
    )

    og.Controller.edit(
        graph,
        {
            og.Controller.Keys.CONNECT: [
                (
                    f"{graph_path}/SubscribeLift.outputs:positionCommand",
                    f"{graph_path}/LiftArticulationController.inputs:positionCommand",
                ),
            ],
        },
    )

    print(
        f"[OK] Core ROS bridge graph created: {graph_path} "
        f"(lift down={lift_lower:.6f}, up={lift_upper:.6f})"
    )


def build_camera_bridge(camera_path):
    """
    Camera가 실제 Stage에 있을 때만 RGB ROS topic을 생성한다.

    docking node가 기본적으로 기다리는 topic:
        /vision/front_camera/image_raw
    """
    if not ENABLE_CAMERA or not camera_path:
        print("[INFO] Camera ROS bridge skipped.")
        return None

    try:
        import omni.replicator.core as rep

        render_product = rep.create.render_product(
            camera_path,
            resolution=(CAMERA_WIDTH, CAMERA_HEIGHT),
        )

        render_product_path = str(render_product.path)

        camera_graph = f"{GRAPH_PATH}_Camera"
        stage = omni.usd.get_context().get_stage()

        old = stage.GetPrimAtPath(camera_graph)
        if old and old.IsValid():
            stage.RemovePrim(camera_graph)

        keys = og.Controller.Keys

        og.Controller.edit(
            {
                "graph_path": camera_graph,
                "evaluator_name": "execution",
            },
            {
                keys.CREATE_NODES: [
                    (
                        "OnPlaybackTick",
                        "omni.graph.action.OnPlaybackTick",
                    ),
                    (
                        "CameraHelper",
                        "isaacsim.ros2.bridge.ROS2CameraHelper",
                    ),
                ],

                keys.CONNECT: [
                    (
                        "OnPlaybackTick.outputs:tick",
                        "CameraHelper.inputs:execIn",
                    ),
                ],

                keys.SET_VALUES: [
                    (
                        "CameraHelper.inputs:renderProductPath",
                        render_product_path,
                    ),
                    (
                        "CameraHelper.inputs:topicName",
                        CAMERA_TOPIC,
                    ),
                    (
                        "CameraHelper.inputs:frameId",
                        CAMERA_FRAME,
                    ),
                    (
                        "CameraHelper.inputs:type",
                        "rgb",
                    ),
                ],
            },
        )

        print(
            "[OK] Camera ROS bridge created\n"
            f"     camera={camera_path}\n"
            f"     topic={CAMERA_TOPIC}\n"
            f"     resolution={CAMERA_WIDTH}x{CAMERA_HEIGHT}"
        )

        return render_product

    except Exception as exc:
        print(
            "[WARN] Camera bridge creation failed. "
            "Core AMR bridge will continue."
        )
        print(f"       {type(exc).__name__}: {exc}")
        return None


# ============================================================
# 5. Main
# ============================================================

def main():
    if not os.path.isfile(FACTORY_USD):
        raise FileNotFoundError(
            f"\nFACTORY_USD not found:\n{FACTORY_USD}\n"
        )

    print("\n" + "=" * 78)
    print("STANDALONE FACTORY ROS 2 BRIDGE")
    print("=" * 78)
    print(f"FACTORY_USD = {FACTORY_USD}")
    print(f"ROS_DOMAIN_ID = {os.environ.get('ROS_DOMAIN_ID', '(default 0)')}")
    print("=" * 78)

    # ROS 2 bridge extension
    enable_extension("isaacsim.ros2.bridge")
    # Extension registration을 몇 프레임 기다린 뒤 OmniGraph를 생성한다.
    for _ in range(5):
        simulation_app.update()

    # USD open
    ok = stage_utils.open_stage(FACTORY_USD)
    if not ok:
        raise RuntimeError(f"Failed to open USD: {FACTORY_USD}")

    simulation_app.update()
    simulation_app.update()

    while stage_utils.is_stage_loading():
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()

    # Disable old graph copies from backed-up USD
    disabled = disable_old_ros_graphs(stage)

    if disabled:
        print("\n[OLD GRAPHS DISABLED]")
        for path in disabled:
            print(f"  {path}")

    # Robot / Stage inventory
    robot_path = find_robot_root(stage)
    source_camera_path = attach_iw_hub_sensors(stage, robot_path)
    inspect_camera_and_robot(stage, robot_path, source_camera_path)
    camera_path, camera_mount_path, camera_mount_op, camera_mount_seed = create_docking_camera(
        stage, robot_path, source_camera_path
    )
    inventory = print_stage_inventory(stage, robot_path, disabled, camera_path)
    nodes_by_id = {
        int(n["id"]): n
        for n in inventory["inventory"]["nodes"]
        if n.get("id") is not None
    }

    if inventory["camera_path"] != camera_path:
        raise RuntimeError(
            "Inventory camera does not match referenced IW Hub sensor camera: "
            f"{inventory['camera_path']} != {camera_path}"
        )

    ensure_runtime_floor(stage)

    # ---- Spawn every AMR and wire one ROS graph per robot.
    ensure_runtime_floor(stage)
    amr_records = []
    for index, spec in enumerate(AMRS):
        amr_path = spec["prim"]
        if spec["spawn"] is not None:
            spawn_amr(stage, amr_path, *spec["spawn"])
            for _ in range(6):
                simulation_app.update()
        if not stage.GetPrimAtPath(amr_path).IsValid():
            raise RuntimeError(f"AMR not found: {amr_path}")

        graph_path = GRAPH_PATH if not spec["namespace"] else (
            f"{GRAPH_PATH}_{spec['namespace']}"
        )
        lift_lower, lift_upper = inspect_lift_joint(stage, amr_path)
        build_core_bridge_graph(
            amr_path,
            lift_lower,
            lift_upper,
            graph_path=graph_path,
            namespace=spec["namespace"],
            publish_clock=(index == 0),
        )
        prefix = spec["namespace"].strip("/")
        topic = (lambda n, p=prefix: f"/{p}{n}") if prefix else (lambda n: n)
        amr_records.append(
            {
                "name": spec["name"],
                "amr_prim": amr_path,
                "namespace": spec["namespace"],
                "graph_path": graph_path,
                "colour": list(spec["colour"]),
                "amr_start": world_pose_of_prim(stage.GetPrimAtPath(amr_path)),
                "topics": {
                    "cmd_vel": topic(CMD_VEL_TOPIC),
                    "odom": topic(ODOM_TOPIC),
                    "lift_cmd": topic(LIFT_CMD_TOPIC),
                    "lift_state": topic(LIFT_STATE_TOPIC),
                    "dolly_cmd": topic(DOLLY_CMD_TOPIC),
                },
                "missions": [],
            }
        )

    # ---- Assign tasks to robots (planner layer, solver is swappable).
    graph = load_waypoint_graph(stage, nodes_by_id)

    # Persist the corridor topology, not just the node positions.
    #
    # Without it, anything outside this process that wants to reason about
    # routes has to guess the edges, and a complete graph is the only available
    # guess. That turns every route into a straight line and makes questions
    # like "does this task pass a corner" unanswerable - which is exactly what
    # blocked scripts/classify_tasks.py. The list is small and the planner has
    # already built it by this point.
    inventory["inventory"]["edges"] = sorted(
        [int(a), int(b), round(float(weight), 4)]
        for a, neighbours in graph.adjacency.items()
        for b, weight in neighbours
        if int(a) < int(b)
    )
    print(
        f"[GRAPH] persisted {len(inventory['inventory']['edges'])} edges "
        "to the inventory",
        flush=True,
    )

    # Safe only from here: the node coordinates and the edge list have both been
    # read out already, so hiding the markers costs the planner nothing.
    hide_waypoint_graph(stage)

    tasks = [
        planner.Task(t["id"], t["dolly"], t["pickup"], t["dropoff"])
        for t in TASKS
    ]
    vehicles = [
        planner.Vehicle(
            record["name"],
            planner.nearest_node(
                graph, record["amr_start"]["x"], record["amr_start"]["y"]
            ),
        )
        for record in amr_records
    ]

    baseline = planner.plan_manual(graph, tasks, vehicles, MANUAL_ASSIGNMENT)
    greedy = planner.plan(graph, tasks, vehicles, solver="greedy")
    if PLAN_SOLVER == "manual":
        chosen = baseline
    elif PLAN_SOLVER == "cuopt":
        chosen = load_cuopt_plan(graph, tasks, vehicles)
    else:
        chosen = planner.plan(graph, tasks, vehicles, solver=PLAN_SOLVER)

    print("\n" + "=" * 78)
    print("FLEET PLAN")
    print("=" * 78)
    for label, result in (
        ("manual  ", baseline),
        ("greedy  ", greedy),
        ("selected", chosen),
    ):
        print(f"  {label} {result.summary()}")
    if baseline.makespan > 0:
        print(
            "  improvement vs manual: makespan "
            f"{100.0 * (1.0 - chosen.makespan / baseline.makespan):+.1f}% | "
            "distance "
            f"{100.0 * (1.0 - chosen.total_distance / baseline.total_distance):+.1f}%"
        )
    print("=" * 78 + "\n", flush=True)
    # Record every candidate, not only the winner.
    #
    # The comparison is the evidence that cuOpt did something: on its own,
    # "makespan 107.2 m" means nothing to a viewer. Storing the rejected plans
    # alongside it lets the panel node draw the contrast without re-running any
    # solver, and keeps the numbers identical to the ones printed above rather
    # than recomputed and possibly drifting.
    inventory["inventory"]["plan_candidates"] = [
        {
            "label": label.strip(),
            "solver": result.solver,
            "solve_seconds": result.solve_seconds,
            "makespan_m": result.makespan,
            "total_m": result.total_distance,
            "assignment": {
                a.vehicle: [task.task_id for task in a.tasks]
                for a in result.assignments
            },
            "cost_m": {a.vehicle: a.cost for a in result.assignments},
        }
        for label, result in (
            ("manual", baseline), ("greedy", greedy), ("selected", chosen)
        )
    ]
    inventory["inventory"]["fleet_plan"] = {
        "solver": chosen.solver,
        "solve_seconds": chosen.solve_seconds,
        "makespan_m": chosen.makespan,
        "total_distance_m": chosen.total_distance,
        "baseline_manual": {
            "makespan_m": baseline.makespan,
            "total_distance_m": baseline.total_distance,
        },
        "baseline_greedy": {
            "makespan_m": greedy.makespan,
            "total_distance_m": greedy.total_distance,
        },
    }

    # ---- Turn the assignment into drivable routes, phase by phase. Tasks with
    # the same index run concurrently, so they reserve edges against each other.
    plan_by_vehicle = chosen.by_vehicle()
    phase_count = max((len(a.tasks) for a in chosen.assignments), default=0)
    shared_edges = set()
    for phase in range(phase_count):
        reserved_edges = set()
        phase_claims = []
        for spec, record in zip(AMRS, amr_records):
            assignment = plan_by_vehicle[record["name"]]
            if phase >= len(assignment.tasks):
                continue
            task = assignment.tasks[phase]
            here = assignment.tasks[phase - 1].dropoff if phase else None

            if here is None:
                approach_route = [task.pickup]
            else:
                approach_route, _ = graph.shortest_path(
                    here, task.pickup, reserved_edges
                )
            carry_route, _ = graph.shortest_path(
                task.pickup, task.dropoff, reserved_edges
            )
            claimed = route_edges(approach_route) | route_edges(carry_route)

            dock = build_mission_dock(
                stage, record["amr_prim"], task.dolly, nodes_by_id[task.pickup]
            )
            neutralize_dolly_physics(stage, task.dolly)
            ground_dolly_to_floor(stage, task.dolly)

            visual_path = f"{ROUTE_VISUAL_PATH}_{record['name']}_m{phase + 1}"
            create_route_visual(
                stage, carry_route, nodes_by_id, visual_path, spec["colour"]
            )

            record["missions"].append(
                {
                    "index": phase,
                    "task_id": task.task_id,
                    "dolly_path": task.dolly,
                    "start_node": task.pickup,
                    "goal_node": task.dropoff,
                    "approach_route": (
                        approach_route if here is None else approach_route[1:]
                    ),
                    "mission_route": carry_route,
                    "mission_dock": dock,
                    "route_visual": visual_path,
                }
            )
            reserved_edges |= claimed
            phase_claims.append((record["name"], claimed))
            print(
                f"[PLAN] {record['name']} mission{phase + 1} ({task.task_id}) "
                f"Node_{task.pickup} -> Node_{task.dropoff} dolly={task.dolly} "
                f"approach={approach_route} carry={carry_route}",
                flush=True,
            )

        for i in range(len(phase_claims)):
            for j in range(i + 1, len(phase_claims)):
                overlap = phase_claims[i][1] & phase_claims[j][1]
                if overlap:
                    shared_edges |= overlap
                    print(
                        f"[TRAFFIC] phase{phase + 1} {phase_claims[i][0]} vs "
                        f"{phase_claims[j][0]} shares "
                        f"{sorted(edge_key(e) for e in overlap)}",
                        flush=True,
                    )

    inventory["inventory"]["shared_edges"] = sorted(
        edge_key(e) for e in shared_edges
    )
    print(
        "[TRAFFIC] runtime-locked edges: "
        f"{inventory['inventory']['shared_edges'] or 'none'}",
        flush=True,
    )
    inventory["inventory"]["amrs"] = amr_records
    save_inventory_files(inventory["inventory"])

    # Flat view used by the runtime follower loop.
    missions = []
    for record in amr_records:
        for entry in record["missions"]:
            missions.append(
                {
                    "name": f"{record['name']}_m{entry['index'] + 1}",
                    "amr": record["name"],
                    "amr_prim": record["amr_prim"],
                    "graph_path": record["graph_path"],
                    "dolly_path": entry["dolly_path"],
                    "route_visual": entry["route_visual"],
                }
            )

    render_product = build_camera_bridge(inventory["camera_path"])

    # Start Timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    print("\n" + "=" * 78)
    print("BRIDGE RUNNING")
    print("=" * 78)
    print(f"Inventory TXT : {INVENTORY_TXT}")
    print(f"Inventory JSON: {INVENTORY_JSON}")
    print("")
    print("PC2에서 확인:")
    print("  ros2 topic list")
    print("  ros2 topic echo /odom --once")
    print("  ros2 topic echo /clock --once")
    print("")
    print("AMR 짧은 직진 테스트:")
    print(
        "  timeout 2 ros2 topic pub -r 10 /cmd_vel "
        "geometry_msgs/msg/Twist "
        "\"{linear: {x: 0.15}, angular: {z: 0.0}}\""
    )
    print("리프트 테스트 (USD limit 기반):")
    print(
        "  ros2 topic pub --once /lift_cmd sensor_msgs/msg/JointState "
        "\"{name: [lift_joint], position: [%.6f]}\"" % lift_upper
    )
    print(
        "  ros2 topic pub --once /lift_cmd sensor_msgs/msg/JointState "
        "\"{name: [lift_joint], position: [%.6f]}\"" % lift_lower
    )
    print("")
    if inventory["camera_path"]:
        print(f"Camera topic expected: {CAMERA_TOPIC}")
    else:
        print(
            "Camera Prim was not found. "
            "iw_hub_sensors.usd를 Stage에 붙이는 단계가 다음 작업입니다."
        )

    print("=" * 78 + "\n")

    chassis_prim = stage.GetPrimAtPath(f"{robot_path}/chassis")
    docking_camera_prim = stage.GetPrimAtPath(camera_path)
    detect_matrix_convention(stage, chassis_prim)
    runtime = []
    for record in amr_records:
        runtime.append(
            {
                "name": record["name"],
                "graph_path": record["graph_path"],
                "amr_prim": record["amr_prim"],
                "chassis_prim": stage.GetPrimAtPath(f"{record['amr_prim']}/chassis"),
                "lift_path": f"{record['amr_prim']}/lift_joint",
                "missions": [
                    {
                        "dolly_path": m["dolly_path"],
                        "dolly_prim": stage.GetPrimAtPath(m["dolly_path"]),
                        "route_visual": m["route_visual"],
                    }
                    for m in record["missions"]
                ],
                "active": 0,
                "follower": None,
            }
        )
    print(
        "[GEOMETRY CHECK] "
        f"chassis={world_bounds_of_prim(chassis_prim)} "
        f"dolly={world_bounds_of_prim(runtime[0]['missions'][0]['dolly_prim'])}",
        flush=True,
    )
    follower_tick = 0
    last_candidate = None
    while simulation_app.is_running():
        simulation_app.update()
        follower_tick += 1

        if CAMERA_DEBUG and follower_tick % 30 == 0:
            report_camera_aim(stage, camera_path, runtime)
        for robot in runtime:
            if robot["active"] >= len(robot["missions"]):
                continue
            mission = robot["missions"][robot["active"]]
            dolly_command = poll_dolly_command(robot["graph_path"])
            if dolly_command == DOLLY_CMD_FREEZE:
                freeze_target_dolly(stage, mission["dolly_path"])
            elif dolly_command == DOLLY_CMD_ATTACH:
                # Docked and holding it: now show where the Dolly is going.
                set_route_visual_visible(stage, True, mission["route_visual"])
            elif dolly_command == DOLLY_CMD_RELEASE:
                set_route_visual_visible(stage, False, mission["route_visual"])
            was_attached = robot["follower"] is not None
            robot["follower"] = update_pickup_follower(
                stage,
                robot["amr_prim"],
                robot["lift_path"],
                robot["follower"],
                dolly_command,
                mission["dolly_path"],
            )
            if was_attached and robot["follower"] is None:
                # Dolly delivered: this robot moves on to its next mission.
                robot["active"] += 1
                print(
                    f"[MISSION SLOT] {robot['name']} -> mission "
                    f"{robot['active'] + 1}",
                    flush=True,
                )
        if CALIBRATION_FILE.exists():
            try:
                candidate = int(CALIBRATION_FILE.read_text(encoding="ascii").strip())
                if candidate != last_candidate:
                    applied = apply_camera_candidate(
                        camera_mount_op, camera_mount_seed, candidate
                    )
                    last_candidate = candidate
                    print(
                        f"[CAMERA CALIBRATION] candidate={candidate} "
                        f"dx={applied[0]:+.2f} dy={applied[1]:+.2f} "
                        f"dz={applied[2]:+.2f} pitch={applied[3]:+.1f}",
                        flush=True,
                    )
            except (OSError, ValueError):
                pass
        if follower_tick % 60 == 0:
            for robot in runtime:
                if not robot["chassis_prim"].IsValid():
                    continue
                slot = min(robot["active"], len(robot["missions"]) - 1)
                chassis_pose = world_pose_of_prim(robot["chassis_prim"])
                dolly_pose = world_pose_of_prim(
                    robot["missions"][slot]["dolly_prim"]
                )
                print(
                    f"[STATE] {robot['name']}/m{slot + 1} "
                    f"chassis=({chassis_pose['x']:.3f},{chassis_pose['y']:.3f},"
                    f"z={chassis_pose['z']:.3f},{chassis_pose['yaw_deg']:.2f}) "
                    f"dolly=({dolly_pose['x']:.3f},{dolly_pose['y']:.3f},"
                    f"{dolly_pose['z']:.3f})",
                    flush=True,
                )



if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        print("\n[CTRL+C] Bridge stopped by user.")

    except Exception as exc:
        print("\n[FATAL]")
        print(f"{type(exc).__name__}: {exc}")
        traceback.print_exc()

    finally:
        try:
            simulation_app.close()
        except Exception:
            pass
