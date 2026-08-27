#!/usr/bin/env python3
"""Audit lift/Dolly collision geometry in the Main Scene."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
ISAAC_DIR = REPO_ROOT / 'simulation' / 'isaac_sim'
DEFAULT_USD = ISAAC_DIR / 'config' / 'AF2_FLAT_integration.usda'


def parse_args():
    """Parse paths before creating SimulationApp."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--usd', type=Path, default=DEFAULT_USD)
    parser.add_argument('--amr-prim', default='/World/_23/iw_hub_01')
    parser.add_argument('--lift-joint', default='lift_joint')
    parser.add_argument('--dolly-prim', default='/World/dolly_physics')
    return parser.parse_args()


ARGS = parse_args()
sys.path.insert(0, str(ISAAC_DIR))

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({'headless': True})

import omni.usd  # noqa: E402, I100, I201
from isaacsim.core.api import SimulationContext  # noqa: E402, I100, I201
from isaacsim.core.utils.stage import open_stage  # noqa: E402, I100, I201
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402, I100, I201

from docking_lift_geometry import (  # noqa: E402, I100, I201
    collision_shapes,
    combined_bounds,
    lift_joint_bodies,
    shape_world_bounds,
    world_pose,
)


def main():
    """Load physics, resolve the joint child body, and list exact bounds."""
    if not open_stage(str(ARGS.usd.resolve())):
        raise RuntimeError(f'Failed to open {ARGS.usd}')
    for _ in range(3):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    simulation_context = SimulationContext(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
    )
    simulation_context.initialize_physics()
    simulation_context.play()
    for _ in range(60):
        simulation_context.step(render=False)

    joint_path = f'{ARGS.amr_prim}/{ARGS.lift_joint}'
    print('AMR_WHEEL_GEOMETRY')
    for side in ('left_wheel', 'right_wheel'):
        wheel_path = f'{ARGS.amr_prim}/{side}/Cylinder'
        wheel_prim = stage.GetPrimAtPath(wheel_path)
        if not wheel_prim.IsValid():
            raise RuntimeError(f'Wheel geometry not found: {wheel_path}')
        cylinder = UsdGeom.Cylinder(wheel_prim)
        print(
            f'{wheel_path} radius={cylinder.GetRadiusAttr().Get()} '
            f'height={cylinder.GetHeightAttr().Get()} '
            f'axis={cylinder.GetAxisAttr().Get()} '
            f'bounds={shape_world_bounds(wheel_prim)}'
        )

    body0, body1 = lift_joint_bodies(stage, joint_path)
    print(f'joint={joint_path}')
    print(f'body0={body0}')
    print(f'body1={body1}')

    moving_paths = body1 or body0
    if len(moving_paths) != 1:
        raise RuntimeError(f'Ambiguous moving body candidates: {moving_paths}')
    moving_prim = stage.GetPrimAtPath(moving_paths[0])
    lift_shapes = collision_shapes(moving_prim)
    print(f'moving_body={moving_prim.GetPath()}')
    print('LIFT_COLLISIONS')
    for prim in lift_shapes:
        xform_ops = [
            (op.GetOpName(), op.Get())
            for op in UsdGeom.Xformable(prim).GetOrderedXformOps()
        ]
        geometry_attrs = [
            (attr.GetName(), attr.Get())
            for attr in prim.GetAttributes()
            if attr.GetName() in (
                'size', 'extent', 'xformOp:scale',
                'xformOp:translate', 'xformOp:orient',
            )
        ]
        print(f'{prim.GetPath()} bounds={shape_world_bounds(prim)}')
        print(
            '  enabled='
            f'{UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()} '
            f'xform_ops={xform_ops}'
        )
        print(f'  geometry_attrs={geometry_attrs}')
    print(f'lift_combined={combined_bounds(lift_shapes)}')
    print('LIFT_VISUAL_MESHES')
    for prim in Usd.PrimRange(moving_prim):
        if prim.IsA(UsdGeom.Mesh):
            print(f'{prim.GetPath()} bounds={shape_world_bounds(prim)}')

    dolly_root = stage.GetPrimAtPath(ARGS.dolly_prim)
    dolly_base = stage.GetPrimAtPath(f'{ARGS.dolly_prim}/Base')
    print(f'dolly_root_pose={world_pose(dolly_root)}')
    print(f'dolly_base_pose={world_pose(dolly_base)}')
    print('DOLLY_VISUAL_MESHES')
    for prim in Usd.PrimRange(dolly_base):
        if prim.IsA(UsdGeom.Mesh):
            print(f'{prim.GetPath()} bounds={shape_world_bounds(prim)}')
    print('DOLLY_COLLISIONS')
    for prim in collision_shapes(dolly_root):
        rigid_owner = UsdPhysics.RigidBodyAPI(prim)
        collision_enabled = (
            UsdPhysics.CollisionAPI(prim)
            .GetCollisionEnabledAttr()
            .Get()
        )
        geometry_attrs = [
            (attr.GetName(), attr.Get())
            for attr in prim.GetAttributes()
            if attr.GetName() in (
                'size', 'extent', 'xformOp:scale',
                'xformOp:translate', 'xformOp:orient',
            )
        ]
        print(
            f'{prim.GetPath()} bounds={shape_world_bounds(prim)} '
            f'rigid_api={bool(rigid_owner)} '
            f'enabled={collision_enabled}'
        )
        print(f'  geometry_attrs={geometry_attrs}')
    simulation_context.stop()


try:
    main()
finally:
    simulation_app.close()
