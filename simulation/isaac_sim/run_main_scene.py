#!/usr/bin/env python3
"""Run the collected Main Scene with deterministic ROS 2 runtime wiring.

All USD edits are authored only in the anonymous session layer. The collected
``AF2_FLAT.usd`` root layer is never saved by this runner.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_USD = (
    REPO_ROOT
    / "simulation"
    / "isaac_sim"
    / "config"
    / "AF2_FLAT_integration.usda"
)
DEFAULT_AMR_PRIM = "/World/_23/iw_hub_01"
DEFAULT_WHEEL_RADIUS_M = 0.08
DEFAULT_WHEEL_DISTANCE_M = 0.57926
DEFAULT_DOLLY_PRIM = "/World/dolly_physics/Base"
DEFAULT_CAMERA_PRIM = (
    DEFAULT_AMR_PRIM
    + "/camera_mount/transporter_camera_first_person"
)
RUNTIME_GRAPH = "/World/RuntimeIntegration"
DEFAULT_GEOMETRY_REPORT = (
    REPO_ROOT
    / "ros2_ws"
    / "log"
    / "runtime_integration"
    / "docking_lift_geometry.txt"
)
DEFAULT_NODEMAP_EXTENSION = (
    REPO_ROOT / "simulation" / "isaac_sim" / "ExtNodeMapBuild"
)
RUNTIME_REQUESTS = {"geometry": False}


def parse_args() -> argparse.Namespace:
    """Parse runtime options before creating SimulationApp."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--usd",
        type=Path,
        default=Path(os.environ.get("COBOT3_MAIN_USD", DEFAULT_USD)),
    )
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=os.environ.get("COBOT3_HEADLESS", "1") != "0",
    )
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument(
        "--geometry-report",
        type=Path,
        default=DEFAULT_GEOMETRY_REPORT,
    )
    parser.add_argument(
        "--lift-contact-offset-x",
        type=float,
        default=float(os.environ.get("COBOT3_LIFT_CONTACT_OFFSET_X", "0.0")),
    )
    parser.add_argument(
        "--lift-contact-offset-y",
        type=float,
        default=float(os.environ.get("COBOT3_LIFT_CONTACT_OFFSET_Y", "0.0")),
    )
    parser.add_argument(
        "--wheel-radius-m",
        type=float,
        default=float(
            os.environ.get("COBOT3_WHEEL_RADIUS_M", DEFAULT_WHEEL_RADIUS_M)
        ),
    )
    parser.add_argument(
        "--wheel-distance-m",
        type=float,
        default=float(
            os.environ.get("COBOT3_WHEEL_DISTANCE_M", DEFAULT_WHEEL_DISTANCE_M)
        ),
    )
    parser.add_argument("--amr-prim", default=DEFAULT_AMR_PRIM)
    parser.add_argument("--dolly-prim", default=DEFAULT_DOLLY_PRIM)
    parser.add_argument("--dolly-frame", default="ground_truth/dolly_base")
    parser.add_argument("--camera-prim", default=DEFAULT_CAMERA_PRIM)
    parser.add_argument("--image-topic", default="/vision/front_camera/image_raw")
    parser.add_argument("--camera-info-topic", default="/vision/front_camera/camera_info")
    parser.add_argument("--camera-frame", default="amr1/front_camera")
    parser.add_argument("--joint-state-topic", default="/amr1/joint_states")
    parser.add_argument("--joint-command-topic", default="/amr1/joint_commands")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--camera-focal-length",
        type=float,
        default=0.5,
        help="Runtime-only focal length matching the existing 2344.32 px calibration.",
    )
    parser.add_argument(
        "--disable-secondary-amr",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--publish-map-to-odom",
        action=argparse.BooleanOptionalAction,
        default=os.environ.get("COBOT3_PUBLISH_MAP_TO_ODOM", "1") != "0",
        help=(
            "Publish the integration-test static map->amr1/odom transform. "
            "Disable when AMCL owns map->odom."
        ),
    )
    parser.add_argument(
        "--enable-nodemap-extension",
        action=argparse.BooleanOptionalAction,
        default=os.environ.get("COBOT3_ENABLE_NODEMAP_EXTENSION", "1") != "0",
    )
    parser.add_argument(
        "--nodemap-extension-path",
        type=Path,
        default=DEFAULT_NODEMAP_EXTENSION,
    )
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
    }
)

import omni.graph.core as og  # noqa: E402
import omni.usd  # noqa: E402
import usdrt  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from pxr import Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from docking_lift_geometry import (  # noqa: E402
    apply_lift_geometry_offset,
    format_snapshot,
    geometry_snapshot,
)


STOP_REQUESTED = False


def request_stop(_signum=None, _frame=None) -> None:
    """Request a graceful stop from SIGINT or SIGTERM."""
    global STOP_REQUESTED
    STOP_REQUESTED = True


def request_geometry_report(_signum=None, _frame=None) -> None:
    """Request a geometry snapshot on the simulation thread."""
    RUNTIME_REQUESTS["geometry"] = True


def write_geometry_report(stage, report_count):
    """Measure, persist, and summarize one live geometry snapshot."""
    dolly_prim = stage.GetPrimAtPath(ARGS.dolly_prim)
    snapshot = geometry_snapshot(
        stage,
        ARGS.amr_prim,
        ARGS.amr_prim + "/chassis",
        ARGS.amr_prim + "/lift_joint",
        str(dolly_prim.GetParent().GetPath()),
        ARGS.dolly_prim,
    )
    report_path = ARGS.geometry_report.expanduser().resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    mode = "w" if report_count == 0 else "a"
    with report_path.open(mode, encoding="utf-8") as stream:
        stream.write(f"SNAPSHOT={report_count + 1}\n")
        stream.write(format_snapshot(snapshot))
    print(
        "PASS: geometry snapshot "
        f"lift_center={snapshot['lift_bounds'].center} "
        f"dolly_center={snapshot['dolly_bounds'].center} "
        f"dx={snapshot['dx']:.6f} dy={snapshot['dy']:.6f} "
        f"yaw_error={snapshot['relative_yaw']:.6f} "
        f"XY_overlap={snapshot['xy_overlap']} "
        f"vertical_gap={snapshot['vertical_gap']:.6f}",
        flush=True,
    )
    return report_count + 1


def validate_environment() -> None:
    """Reject unsafe or ambiguous ROS middleware settings."""
    domain_text = os.environ.get("ROS_DOMAIN_ID", "129")
    try:
        domain_id = int(domain_text)
    except ValueError as error:
        raise RuntimeError(f"Invalid ROS_DOMAIN_ID={domain_text!r}") from error
    if domain_id not in range(129, 136):
        raise RuntimeError(
            f"ROS_DOMAIN_ID must be 129..135, got {domain_id}"
        )
    rmw = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    if rmw != "rmw_fastrtps_cpp":
        raise RuntimeError(
            "RMW_IMPLEMENTATION must be rmw_fastrtps_cpp, "
            f"got {rmw!r}"
        )


def require_prim(stage, prim_path: str):
    """Return a valid stage prim or fail with its exact path."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Required prim not found: {prim_path}")
    return prim


def set_graph_attribute(stage, path: str, value) -> None:
    """Set one existing graph attribute in the session layer."""
    attribute = stage.GetAttributeAtPath(path)
    if not attribute.IsValid():
        raise RuntimeError(f"Graph attribute not found: {path}")
    attribute.Set(value)


def configure_existing_amr_graphs(stage) -> None:
    """Normalize the Main USD's AMR1 ROS contract without saving it."""
    graph = ARGS.amr_prim + "/ActionGraph1"
    lidar_graph = ARGS.amr_prim + "/ROS_Lidar_Graph1"
    require_prim(stage, graph)
    require_prim(stage, lidar_graph)

    set_graph_attribute(
        stage,
        graph + "/ros2_subscribe_twist.inputs:nodeNamespace",
        "",
    )
    set_graph_attribute(
        stage,
        graph + "/ros2_subscribe_twist.inputs:topicName",
        "/amr1/cmd_vel",
    )
    set_graph_attribute(
        stage,
        graph + "/differential_controller.inputs:wheelRadius",
        ARGS.wheel_radius_m,
    )
    set_graph_attribute(
        stage,
        graph + "/differential_controller.inputs:wheelDistance",
        ARGS.wheel_distance_m,
    )

    odom_publisher = graph + "/ros2_publish_odometry"
    set_graph_attribute(
        stage,
        odom_publisher + ".inputs:nodeNamespace",
        "",
    )
    set_graph_attribute(
        stage,
        odom_publisher + ".inputs:topicName",
        "/amr1/odom",
    )
    set_graph_attribute(
        stage,
        odom_publisher + ".inputs:odomFrameId",
        "amr1/odom",
    )
    set_graph_attribute(
        stage,
        odom_publisher + ".inputs:chassisFrameId",
        "amr1/base_link",
    )

    clock_publisher = graph + "/ros2_publish_clock"
    set_graph_attribute(
        stage,
        clock_publisher + ".inputs:nodeNamespace",
        "",
    )
    set_graph_attribute(
        stage,
        clock_publisher + ".inputs:topicName",
        "/clock",
    )

    for suffix in (
        "ros2_publish_raw_transform_tree",
        "ros2_publish_raw_transform_tree_01",
        "ros2_publish_raw_transform_tree_02",
    ):
        publisher = graph + "/" + suffix
        set_graph_attribute(
            stage,
            publisher + ".inputs:nodeNamespace",
            "",
        )
        static_publisher = bool(
            stage.GetAttributeAtPath(
                publisher + ".inputs:staticPublisher"
            ).Get()
        )
        set_graph_attribute(
            stage,
            publisher + ".inputs:topicName",
            "/tf_static" if static_publisher else "/tf",
        )

    lidar_publisher = lidar_graph + "/LaserScanPublish"
    set_graph_attribute(
        stage,
        lidar_publisher + ".inputs:nodeNamespace",
        "",
    )
    set_graph_attribute(
        stage,
        lidar_publisher + ".inputs:topicName",
        "/amr1/scan",
    )

    if ARGS.disable_secondary_amr:
        for prim_path in (
            "/World/_23/iw_hub_02/ActionGraph2",
            "/World/_23/iw_hub_02/ROS_Lidar_Graph2",
        ):
            prim = stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                stage.RemovePrim(prim_path)


def camera_relative_transform(amr_prim, camera_prim):
    """Return camera pose relative to the AMR articulation root."""
    amr_world = UsdGeom.Xformable(amr_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    camera_world = UsdGeom.Xformable(
        camera_prim
    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    relative = camera_world * amr_world.GetInverse()
    translation = relative.ExtractTranslation()
    quaternion = relative.ExtractRotationQuat()
    imaginary = quaternion.GetImaginary()
    return (
        [translation[0], translation[1], translation[2]],
        [imaginary[0], imaginary[1], imaginary[2], quaternion.GetReal()],
    )


def create_runtime_graph(stage, amr_prim, camera_prim, dolly_prim) -> None:
    """Add camera publishers plus map and camera TF to the session layer."""
    camera = UsdGeom.Camera(camera_prim)
    camera.GetFocalLengthAttr().Set(ARGS.camera_focal_length)
    translation, rotation = camera_relative_transform(amr_prim, camera_prim)
    amr_world = UsdGeom.Xformable(amr_prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    amr_translation = amr_world.ExtractTranslation()
    amr_quaternion = amr_world.ExtractRotationQuat()
    amr_imaginary = amr_quaternion.GetImaginary()
    map_to_odom_translation = list(amr_translation)
    map_to_odom_rotation = [
        amr_imaginary[0],
        amr_imaginary[1],
        amr_imaginary[2],
        amr_quaternion.GetReal(),
    ]

    old_graph = stage.GetPrimAtPath(RUNTIME_GRAPH)
    if old_graph.IsValid():
        stage.RemovePrim(RUNTIME_GRAPH)

    keys = og.Controller.Keys
    connections = [
        ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
        ("CreateRenderProduct.outputs:execOut", "PublishRgb.inputs:execIn"),
        ("CreateRenderProduct.outputs:execOut", "PublishCameraInfo.inputs:execIn"),
        ("CreateRenderProduct.outputs:renderProductPath", "PublishRgb.inputs:renderProductPath"),
        ("CreateRenderProduct.outputs:renderProductPath", "PublishCameraInfo.inputs:renderProductPath"),
        ("OnPlaybackTick.outputs:tick", "PublishCameraTf.inputs:execIn"),
        ("ReadSimulationTime.outputs:simulationTime", "PublishCameraTf.inputs:timeStamp"),
        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "LiftController.inputs:execIn"),
        ("ReadSimulationTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        ("SubscribeJointState.outputs:jointNames", "LiftController.inputs:jointNames"),
        ("SubscribeJointState.outputs:positionCommand", "LiftController.inputs:positionCommand"),
        ("SubscribeJointState.outputs:velocityCommand", "LiftController.inputs:velocityCommand"),
        ("SubscribeJointState.outputs:effortCommand", "LiftController.inputs:effortCommand"),
        ("OnPlaybackTick.outputs:tick", "PublishDollyTf.inputs:execIn"),
        ("ReadSimulationTime.outputs:simulationTime", "PublishDollyTf.inputs:timeStamp"),
        ("ReadDollyPose.outputs:translation", "PublishDollyTf.inputs:translation"),
        ("ReadDollyPose.outputs:orientation", "PublishDollyTf.inputs:rotation"),
    ]
    if ARGS.publish_map_to_odom:
        connections.extend(
            [
                ("OnPlaybackTick.outputs:tick", "PublishMapToOdom.inputs:execIn"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishMapToOdom.inputs:timeStamp"),
            ]
        )

    og.Controller.edit(
        {
            "graph_path": RUNTIME_GRAPH,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PublishRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("PublishCameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("PublishMapToOdom", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PublishCameraTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("LiftController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("ReadDollyPose", "isaacsim.core.nodes.IsaacReadWorldPose"),
                ("PublishDollyTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", [Sdf.Path(ARGS.camera_prim)]),
                ("CreateRenderProduct.inputs:width", ARGS.width),
                ("CreateRenderProduct.inputs:height", ARGS.height),
                ("PublishRgb.inputs:topicName", ARGS.image_topic),
                ("PublishRgb.inputs:frameId", ARGS.camera_frame),
                ("PublishRgb.inputs:type", "rgb"),
                ("PublishCameraInfo.inputs:topicName", ARGS.camera_info_topic),
                ("PublishCameraInfo.inputs:frameId", ARGS.camera_frame),
                ("PublishMapToOdom.inputs:topicName", "/tf_static"),
                ("PublishMapToOdom.inputs:parentFrameId", "map"),
                ("PublishMapToOdom.inputs:childFrameId", "amr1/odom"),
                ("PublishMapToOdom.inputs:translation", map_to_odom_translation),
                ("PublishMapToOdom.inputs:rotation", map_to_odom_rotation),
                ("PublishMapToOdom.inputs:staticPublisher", True),
                ("PublishCameraTf.inputs:topicName", "/tf_static"),
                ("PublishCameraTf.inputs:parentFrameId", "amr1/base_link"),
                ("PublishCameraTf.inputs:childFrameId", ARGS.camera_frame),
                ("PublishCameraTf.inputs:translation", translation),
                ("PublishCameraTf.inputs:rotation", rotation),
                ("PublishCameraTf.inputs:staticPublisher", True),
                ("PublishJointState.inputs:topicName", ARGS.joint_state_topic),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ARGS.amr_prim)]),
                ("SubscribeJointState.inputs:topicName", ARGS.joint_command_topic),
                ("LiftController.inputs:targetPrim", [usdrt.Sdf.Path(ARGS.amr_prim)]),
                ("LiftController.inputs:jointNames", ["lift_joint"]),
                ("ReadDollyPose.inputs:prim", [Sdf.Path(ARGS.dolly_prim)]),
                ("PublishDollyTf.inputs:topicName", "/tf"),
                ("PublishDollyTf.inputs:parentFrameId", "map"),
                ("PublishDollyTf.inputs:childFrameId", ARGS.dolly_frame),
                ("PublishDollyTf.inputs:staticPublisher", False),
            ],
            keys.CONNECT: connections,
        },
    )


def main() -> int:
    """Load, configure, initialize, and continuously step the Main Scene."""
    validate_environment()
    usd_path = ARGS.usd.expanduser().resolve()
    if not usd_path.is_file():
        raise RuntimeError(f"Main USD does not exist: {usd_path}")

    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()

    if ARGS.enable_nodemap_extension:
        extension_path = ARGS.nodemap_extension_path.expanduser().resolve()
        if not (extension_path / "config" / "extension.toml").is_file():
            raise RuntimeError(
                f"NodeMap extension not found: {extension_path}"
            )
        extension_manager = (
            omni.kit.app.get_app().get_extension_manager()
        )
        extension_manager.add_path(str(extension_path.parent))
        if not enable_extension("ExtNodeMapBuild"):
            raise RuntimeError("Failed to enable ExtNodeMapBuild")
        simulation_app.update()
        from ext_node_map_build import extension as nodemap_extension_runtime

        if not nodemap_extension_runtime.EXTENSION_RUNTIME_READY:
            raise RuntimeError(
                "ExtNodeMapBuild was registered but its private scene bridge "
                "startup failed"
            )

    if not open_stage(str(usd_path)):
        raise RuntimeError(f"Failed to open Main USD: {usd_path}")
    for _ in range(3):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    amr_prim = require_prim(stage, ARGS.amr_prim)
    camera_prim = require_prim(stage, ARGS.camera_prim)
    dolly_prim = require_prim(stage, ARGS.dolly_prim)

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        configure_existing_amr_graphs(stage)

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        create_runtime_graph(stage, amr_prim, camera_prim, dolly_prim)

    with Usd.EditContext(stage, stage.GetSessionLayer()):
        shifted_lift_geometry = apply_lift_geometry_offset(
            stage,
            ARGS.amr_prim + "/lift_joint",
            ARGS.lift_contact_offset_x,
            ARGS.lift_contact_offset_y,
        )

    simulation_context = SimulationContext(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
    )
    simulation_context.initialize_physics()

    physics_scenes = [
        str(prim.GetPath())
        for prim in stage.TraverseAll()
        if prim.IsA(UsdPhysics.Scene)
    ]
    if not physics_scenes:
        raise RuntimeError("Physics initialization created no PhysicsScene")

    simulation_context.play()
    simulation_context.step(render=True)

    print("PASS: main USD load", flush=True)
    print(f"PASS: physics scene={physics_scenes[0]}", flush=True)
    print(f"PASS: articulation={ARGS.amr_prim}", flush=True)
    print(
        "PASS: lift contact geometry offset "
        f"x={ARGS.lift_contact_offset_x:.4f} "
        f"y={ARGS.lift_contact_offset_y:.4f} "
        f"prims={shifted_lift_geometry}",
        flush=True,
    )
    print("PASS: ROS2 bridge=isaacsim.ros2.bridge", flush=True)
    if ARGS.enable_nodemap_extension:
        print(
            "PASS: scene bridge=ExtNodeMapBuild "
            "public endpoints owner=scene_endpoint_adapter",
            flush=True,
        )
    print(
        "PASS: map->odom owner="
        + (
            "runner_static_test"
            if ARGS.publish_map_to_odom
            else "external_localization"
        ),
        flush=True,
    )
    print("PASS: cmd_vel=/amr1/cmd_vel", flush=True)
    print(
        "PASS: differential drive "
        f"wheel_radius={ARGS.wheel_radius_m:.5f}m "
        f"wheel_distance={ARGS.wheel_distance_m:.5f}m",
        flush=True,
    )
    print("PASS: odom=/amr1/odom", flush=True)
    print("PASS: scan=/amr1/scan", flush=True)
    print(f"PASS: camera={ARGS.image_topic} {ARGS.width}x{ARGS.height}", flush=True)
    print(f"PASS: camera_info={ARGS.camera_info_topic}", flush=True)
    print(f"PASS: joint_state={ARGS.joint_state_topic} command={ARGS.joint_command_topic}", flush=True)
    print(f"PASS: dolly_tf=map->{ARGS.dolly_frame} prim={ARGS.dolly_prim}", flush=True)
    print("READY: main scene runtime", flush=True)

    step_count = 1
    report_count = 0
    while simulation_app.is_running() and not STOP_REQUESTED:
        if ARGS.max_steps > 0 and step_count >= ARGS.max_steps:
            break
        simulation_context.step(render=True)
        step_count += 1
        if RUNTIME_REQUESTS["geometry"]:
            report_count = write_geometry_report(stage, report_count)
            RUNTIME_REQUESTS["geometry"] = False

    simulation_context.stop()
    print(f"PASS: graceful shutdown steps={step_count}", flush=True)
    return 0


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)
signal.signal(signal.SIGUSR1, request_geometry_report)

try:
    raise SystemExit(main())
finally:
    simulation_app.close()
