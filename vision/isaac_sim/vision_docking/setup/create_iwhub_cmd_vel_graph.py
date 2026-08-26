"""
Create a ROS 2 /cmd_vel -> iw.hub differential-drive Action Graph.

Run in Isaac Sim 5.1 Script Editor with Timeline STOPPED.
The script:
1) validates the iw.hub root,
2) auto-detects left/right wheel joint prims,
3) creates ROS2 Subscribe Twist -> Differential Controller
   -> Articulation Controller,
4) prints only the essential result.

Project scene root expected:
    /World/iw_hub_sensors
"""

import omni.graph.core as og
import omni.usd
import omni.kit.app
from pxr import Sdf, UsdPhysics


ROBOT_ROOT = "/World/iw_hub_sensors"
GRAPH_PATH = "/World/IWHubCmdVel"
CMD_VEL_TOPIC = "/cmd_vel"

# Official iw.hub differential-base reference values.
WHEEL_RADIUS_M = 0.08
WHEEL_DISTANCE_M = 0.58

# Graph-side safety clamps. The ROS docking node can be set lower.
MAX_LINEAR_SPEED = 0.20
MAX_ANGULAR_SPEED = 0.30
MAX_WHEEL_SPEED = 8.0


def find_wheel_joint(stage, root_path, side):
    side = side.lower()

    candidates = []

    for prim in stage.Traverse():
        path = str(prim.GetPath())

        if not path.startswith(root_path + "/"):
            continue

        name = prim.GetName().lower()

        if side not in name or "wheel" not in name:
            continue

        # Prefer actual physics joints.
        is_joint = (
            prim.IsA(UsdPhysics.RevoluteJoint)
            or prim.IsA(UsdPhysics.Joint)
        )

        score = 0

        if is_joint:
            score += 100

        if name == f"{side}_wheel":
            score += 50

        if "caster" not in name:
            score += 20

        candidates.append(
            (
                score,
                prim,
            )
        )

    if not candidates:
        return None

    candidates.sort(
        key=lambda item: item[0],
        reverse=True,
    )

    return candidates[0][1]


stage = omni.usd.get_context().get_stage()

robot_prim = stage.GetPrimAtPath(ROBOT_ROOT)

if not robot_prim.IsValid():
    raise RuntimeError(
        f"Robot root not found: {ROBOT_ROOT}"
    )

left_joint_prim = find_wheel_joint(
    stage,
    ROBOT_ROOT,
    "left",
)

right_joint_prim = find_wheel_joint(
    stage,
    ROBOT_ROOT,
    "right",
)

if left_joint_prim is None or right_joint_prim is None:

    wheel_candidates = []

    for prim in stage.Traverse():
        path = str(prim.GetPath())

        if (
            path.startswith(ROBOT_ROOT + "/")
            and "wheel" in prim.GetName().lower()
        ):
            wheel_candidates.append(path)

    raise RuntimeError(
        "Could not auto-detect both drive wheel joints.\n"
        + "\n".join(wheel_candidates[:30])
    )

left_joint_name = left_joint_prim.GetName()
right_joint_name = right_joint_prim.GetName()

# Enable required extensions.
ext_mgr = (
    omni.kit.app.get_app()
    .get_extension_manager()
)

for ext_name in (
    "isaacsim.ros2.bridge",
    "isaacsim.core.nodes",
    "isaacsim.robot.wheeled_robots",
):
    ext_mgr.set_extension_enabled_immediate(
        ext_name,
        True,
    )

# Re-running the script replaces only this graph.
old_graph = stage.GetPrimAtPath(
    GRAPH_PATH
)

if old_graph.IsValid():
    stage.RemovePrim(
        GRAPH_PATH
    )

keys = og.Controller.Keys

og.Controller.edit(
    {
        "graph_path": GRAPH_PATH,
        "evaluator_name": "execution",
    },
    {
        keys.CREATE_NODES: [
            (
                "OnPlaybackTick",
                "omni.graph.action.OnPlaybackTick",
            ),
            (
                "Context",
                "isaacsim.ros2.bridge.ROS2Context",
            ),
            (
                "SubscribeTwist",
                "isaacsim.ros2.bridge.ROS2SubscribeTwist",
            ),
            (
                "BreakLinVel",
                "omni.graph.nodes.BreakVector3",
            ),
            (
                "BreakAngVel",
                "omni.graph.nodes.BreakVector3",
            ),
            (
                "DiffController",
                "isaacsim.robot.wheeled_robots.DifferentialController",
            ),
            (
                "ArtController",
                "isaacsim.core.nodes.IsaacArticulationController",
            ),
        ],

        keys.SET_VALUES: [
            (
                "Context.inputs:useDomainIDEnvVar",
                True,
            ),
            (
                "SubscribeTwist.inputs:topicName",
                CMD_VEL_TOPIC,
            ),
            (
                "DiffController.inputs:wheelRadius",
                WHEEL_RADIUS_M,
            ),
            (
                "DiffController.inputs:wheelDistance",
                WHEEL_DISTANCE_M,
            ),
            (
                "DiffController.inputs:maxLinearSpeed",
                MAX_LINEAR_SPEED,
            ),
            (
                "DiffController.inputs:maxAngularSpeed",
                MAX_ANGULAR_SPEED,
            ),
            (
                "DiffController.inputs:maxWheelSpeed",
                MAX_WHEEL_SPEED,
            ),
            (
                "ArtController.inputs:jointNames",
                [
                    left_joint_name,
                    right_joint_name,
                ],
            ),
            (
                "ArtController.inputs:targetPrim",
                [
                    Sdf.Path(
                        ROBOT_ROOT
                    )
                ],
            ),
        ],

        keys.CONNECT: [
            (
                "OnPlaybackTick.outputs:tick",
                "SubscribeTwist.inputs:execIn",
            ),
            (
                "Context.outputs:context",
                "SubscribeTwist.inputs:context",
            ),
            (
                "SubscribeTwist.outputs:linearVelocity",
                "BreakLinVel.inputs:tuple",
            ),
            (
                "SubscribeTwist.outputs:angularVelocity",
                "BreakAngVel.inputs:tuple",
            ),
            (
                "BreakLinVel.outputs:x",
                "DiffController.inputs:linearVelocity",
            ),
            (
                "BreakAngVel.outputs:z",
                "DiffController.inputs:angularVelocity",
            ),
            (
                "OnPlaybackTick.outputs:deltaSeconds",
                "DiffController.inputs:dt",
            ),
            (
                "OnPlaybackTick.outputs:tick",
                "DiffController.inputs:execIn",
            ),
            (
                "DiffController.outputs:velocityCommand",
                "ArtController.inputs:velocityCommand",
            ),
            (
                "OnPlaybackTick.outputs:tick",
                "ArtController.inputs:execIn",
            ),
        ],
    },
)

print("")
print("===== IW.HUB CMD_VEL GRAPH: PASS =====")
print(f"Robot       : {ROBOT_ROOT}")
print(f"Left joint  : {left_joint_prim.GetPath()}")
print(f"Right joint : {right_joint_prim.GetPath()}")
print(f"Wheel radius: {WHEEL_RADIUS_M:.3f} m")
print(f"Wheel dist  : {WHEEL_DISTANCE_M:.3f} m")
print(f"Topic       : {CMD_VEL_TOPIC}")
print(f"Graph       : {GRAPH_PATH}")
print("")
print("SAVE USD -> PLAY -> check /cmd_vel subscription.")
