#!/usr/bin/env python3
"""Read-only structural audit for the collected Main Isaac Sim scene."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_USD = (
    REPO_ROOT
    / "simulation"
    / "isaac_sim"
    / "worlds"
    / "Collected_AF2_FLAT"
    / "AF2_FLAT.usd"
)
DEFAULT_LOG = (
    REPO_ROOT
    / "ros2_ws"
    / "log"
    / "runtime_integration"
    / "main_scene_audit.txt"
)

TOKENS = (
    "iw",
    "hub",
    "amr",
    "robot",
    "dolly",
    "camera",
    "lidar",
    "laser",
    "lift",
    "joint",
    "wheel",
    "actiongraph",
    "ros",
)


def parse_args() -> argparse.Namespace:
    """Parse paths before starting the Isaac runtime."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils  # noqa: E402


def has_api(prim, schema) -> bool:
    """Return whether a prim carries an applied API schema."""
    try:
        return prim.HasAPI(schema)
    except Exception:
        return False


def authored_value(prim, attribute_name: str) -> str:
    """Render an authored USD attribute or a stable placeholder."""
    attribute = prim.GetAttribute(attribute_name)
    if not attribute or not attribute.HasAuthoredValueOpinion():
        return "<unauthored>"
    try:
        return repr(attribute.Get())
    except Exception as error:
        return f"<error: {error}>"


def world_translation(prim) -> str:
    """Return the default-time world translation for an Xformable prim."""
    if not prim.IsA(UsdGeom.Xformable):
        return "<not-xformable>"
    try:
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        translation = matrix.ExtractTranslation()
        return (
            f"({translation[0]:.6f}, {translation[1]:.6f}, "
            f"{translation[2]:.6f})"
        )
    except Exception as error:
        return f"<error: {error}>"


def compute_dependencies(usd_path: Path):
    """Return layers, resolved assets, and unresolved dependencies."""
    try:
        return UsdUtils.ComputeAllDependencies(str(usd_path))
    except Exception as error:
        return (), (), (f"dependency audit error: {error}",)


def main() -> int:
    """Load and inspect the Main USD without saving any layer."""
    usd_path = ARGS.usd.expanduser().resolve()
    log_path = ARGS.log.expanduser().resolve()
    log_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "MAIN SCENE READ-ONLY AUDIT",
        f"USD={usd_path}",
        "USD_SAVED=NO",
    ]

    if not usd_path.is_file():
        lines.append("LOAD=FAIL")
        lines.append("ERROR=USD file does not exist")
        log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print("FAIL: main USD missing")
        print(f"log: {log_path}")
        return 2

    stage = Usd.Stage.Open(str(usd_path), Usd.Stage.LoadAll)
    if stage is None:
        lines.append("LOAD=FAIL")
        log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print("FAIL: main USD load")
        print(f"log: {log_path}")
        return 2

    prims = list(stage.TraverseAll())
    physics_scenes = [prim for prim in prims if prim.IsA(UsdPhysics.Scene)]
    articulation_roots = [
        prim for prim in prims
        if has_api(prim, UsdPhysics.ArticulationRootAPI)
    ]
    rigid_bodies = [
        prim for prim in prims if has_api(prim, UsdPhysics.RigidBodyAPI)
    ]
    collisions = [
        prim for prim in prims if has_api(prim, UsdPhysics.CollisionAPI)
    ]
    joints = [prim for prim in prims if prim.IsA(UsdPhysics.Joint)]
    prismatic_joints = [
        prim for prim in prims if prim.IsA(UsdPhysics.PrismaticJoint)
    ]
    revolute_joints = [
        prim for prim in prims if prim.IsA(UsdPhysics.RevoluteJoint)
    ]
    cameras = [prim for prim in prims if prim.IsA(UsdGeom.Camera)]
    candidates = [
        prim for prim in prims
        if any(token in str(prim.GetPath()).lower() for token in TOKENS)
    ]
    graph_candidates = [
        prim for prim in prims
        if (
            "graph" in prim.GetTypeName().lower()
            or "graph" in str(prim.GetPath()).lower()
            or "ros2" in str(prim.GetPath()).lower()
        )
    ]

    layers, assets, unresolved = compute_dependencies(usd_path)
    absolute_assets = sorted(
        str(asset) for asset in assets
        if Path(str(asset)).is_absolute()
    )

    default_prim = stage.GetDefaultPrim()
    root_paths = [str(prim.GetPath()) for prim in stage.GetPseudoRoot().GetChildren()]
    type_counts = Counter(prim.GetTypeName() or "<untyped>" for prim in prims)

    lines.extend(
        [
            "LOAD=PASS",
            f"ROOT_PRIMS={root_paths}",
            f"DEFAULT_PRIM={default_prim.GetPath() if default_prim else '<none>'}",
            f"UP_AXIS={UsdGeom.GetStageUpAxis(stage)}",
            f"METERS_PER_UNIT={UsdGeom.GetStageMetersPerUnit(stage)}",
            f"PRIM_COUNT={len(prims)}",
            f"PHYSICS_SCENE_COUNT={len(physics_scenes)}",
            f"ARTICULATION_ROOT_COUNT={len(articulation_roots)}",
            f"RIGID_BODY_COUNT={len(rigid_bodies)}",
            f"COLLISION_COUNT={len(collisions)}",
            f"JOINT_COUNT={len(joints)}",
            f"PRISMATIC_JOINT_COUNT={len(prismatic_joints)}",
            f"REVOLUTE_JOINT_COUNT={len(revolute_joints)}",
            f"CAMERA_COUNT={len(cameras)}",
            f"GRAPH_CANDIDATE_COUNT={len(graph_candidates)}",
            f"DEPENDENCY_LAYER_COUNT={len(layers)}",
            f"DEPENDENCY_ASSET_COUNT={len(assets)}",
            f"UNRESOLVED_DEPENDENCY_COUNT={len(unresolved)}",
            f"ABSOLUTE_ASSET_DEPENDENCY_COUNT={len(absolute_assets)}",
            "",
            "TYPE COUNTS",
        ]
    )
    lines.extend(
        f"{type_name}: {count}"
        for type_name, count in sorted(
            type_counts.items(), key=lambda item: (-item[1], item[0])
        )
    )

    def append_prim_section(title: str, selected_prims) -> None:
        lines.extend(("", title))
        if not selected_prims:
            lines.append("<none>")
            return
        for prim in selected_prims:
            lines.append(
                f"{prim.GetPath()} | type={prim.GetTypeName() or '<untyped>'} "
                f"| world_translation={world_translation(prim)}"
            )

    append_prim_section("PHYSICS SCENES", physics_scenes)
    append_prim_section("ARTICULATION ROOTS", articulation_roots)
    append_prim_section("RIGID BODIES", rigid_bodies)
    append_prim_section("COLLISIONS", collisions)
    append_prim_section("CAMERAS", cameras)
    append_prim_section("GRAPH CANDIDATES", graph_candidates)

    lines.extend(("", "JOINTS"))
    if not joints:
        lines.append("<none>")
    for prim in joints:
        lines.append(
            f"{prim.GetPath()} | type={prim.GetTypeName()} "
            f"| lower={authored_value(prim, 'physics:lowerLimit')} "
            f"| upper={authored_value(prim, 'physics:upperLimit')}"
        )

    append_prim_section("TOKEN CANDIDATES", candidates)

    lines.extend(("", "UNRESOLVED DEPENDENCIES"))
    lines.extend(str(path) for path in unresolved)
    if not unresolved:
        lines.append("<none>")

    lines.extend(("", "ABSOLUTE ASSET DEPENDENCIES"))
    lines.extend(absolute_assets or ("<none>",))

    log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    print("PASS: main USD load")
    print(
        "PASS: stage metadata "
        f"default={default_prim.GetPath() if default_prim else '<none>'} "
        f"up={UsdGeom.GetStageUpAxis(stage)} "
        f"meters={UsdGeom.GetStageMetersPerUnit(stage)}"
    )
    print(
        ("PASS" if not unresolved else "FAIL")
        + f": unresolved dependencies={len(unresolved)}"
    )
    print(
        ("PASS" if physics_scenes else "WARN")
        + f": physics scenes={len(physics_scenes)}"
    )
    print(
        ("PASS" if articulation_roots else "WARN")
        + f": articulation roots={len(articulation_roots)}"
    )
    print(f"PASS: cameras={len(cameras)}")
    print(f"PASS: joints={len(joints)}")
    for prim in articulation_roots[:5]:
        print(f"candidate articulation: {prim.GetPath()}")
    for prim in cameras[:5]:
        print(f"candidate camera: {prim.GetPath()}")
    for prim in prismatic_joints[:5]:
        print(f"candidate prismatic: {prim.GetPath()}")
    print(f"log: {log_path}")
    return 0


try:
    raise SystemExit(main())
finally:
    simulation_app.close()
