#!/usr/bin/env python3
"""Inspect and safely exercise the IW Hub lift with Isaac Sim 5.1."""

from __future__ import annotations

import argparse
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_USD = (
    REPO_ROOT
    / "simulation"
    / "isaac_sim"
    / "config"
    / "AF2_FLAT_integration.usda"
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD)
    parser.add_argument("--amr-prim", default="/World/_23/iw_hub_01")
    parser.add_argument("--joint-name", default="lift_joint")
    parser.add_argument("--delta", type=float, default=0.01)
    parser.add_argument("--settle-steps", type=int, default=180)
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.prims import SingleArticulation  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402


def world_z(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    return float(transform.ExtractTranslation()[2])


def settle(simulation_context, articulation, index, target):
    action = ArticulationAction(
        joint_positions=np.array([target], dtype=np.float32),
        joint_indices=np.array([index], dtype=np.int32),
    )
    for _ in range(ARGS.settle_steps):
        articulation.apply_action(action)
        simulation_context.step(render=False)
    return float(articulation.get_joint_positions([index])[0])


def main():
    usd_path = ARGS.usd.resolve()
    if not open_stage(str(usd_path)):
        raise RuntimeError(f"Failed to open {usd_path}")
    for _ in range(3):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    joint_path = f"{ARGS.amr_prim}/{ARGS.joint_name}"
    lift_path = f"{ARGS.amr_prim}/lift"
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim.IsValid():
        raise RuntimeError(f"Missing lift joint: {joint_path}")

    joint = UsdPhysics.PrismaticJoint(joint_prim)
    lower = float(joint.GetLowerLimitAttr().Get())
    upper = float(joint.GetUpperLimitAttr().Get())
    axis = str(joint.GetAxisAttr().Get())

    simulation_context = SimulationContext(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
    )
    simulation_context.initialize_physics()
    simulation_context.play()
    simulation_context.step(render=False)

    articulation = SingleArticulation(ARGS.amr_prim)
    articulation.initialize()
    index = articulation.get_dof_index(ARGS.joint_name)
    dof_names = list(articulation.dof_names)
    initial = float(articulation.get_joint_positions([index])[0])
    initial = settle(
        simulation_context,
        articulation,
        index,
        initial,
    )
    initial_z = world_z(stage, lift_path)
    positive_target = min(upper, initial + abs(ARGS.delta))
    positive_position = settle(
        simulation_context,
        articulation,
        index,
        positive_target,
    )
    positive_z = world_z(stage, lift_path)
    returned_position = settle(
        simulation_context,
        articulation,
        index,
        initial,
    )
    returned_z = world_z(stage, lift_path)

    direction = "UP" if positive_z > initial_z else "DOWN"
    print("PASS: IW Hub articulation")
    print(f"prim={ARGS.amr_prim}")
    print(f"dof_names={dof_names}")
    print(f"joint={ARGS.joint_name} index={index} type=Prismatic axis={axis}")
    print(f"lower={lower:.9f} upper={upper:.9f} initial={initial:.9f}")
    print(
        f"positive_target={positive_target:.9f} "
        f"measured={positive_position:.9f} direction={direction}"
    )
    print(
        f"lift_z_initial={initial_z:.9f} "
        f"lift_z_positive={positive_z:.9f} "
        f"delta_z={positive_z - initial_z:.9f}"
    )
    print(
        f"return_target={initial:.9f} measured={returned_position:.9f} "
        f"lift_z={returned_z:.9f}"
    )
    simulation_context.stop()


try:
    main()
finally:
    simulation_app.close()
