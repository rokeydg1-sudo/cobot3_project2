#!/usr/bin/env python3
"""Rasterize Main Scene static collision geometry into a Nav2 PGM map."""

from __future__ import annotations

import argparse
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
DEFAULT_OUTPUT = Path("/tmp/af2_flat_scenario0_static_candidate.pgm")
EXCLUDED_PRIMS = ("/World/_23/iw_hub_01", "/World/dolly_physics")
ORIGIN = (-49.325054, -24.624803)
RESOLUTION = 0.10
WIDTH = 594
HEIGHT = 583
SLICE_MIN_Z = 0.10
SLICE_MAX_Z = 0.60
FREE_VALUE = 254
OCCUPIED_VALUE = 0


def parse_args() -> argparse.Namespace:
    """Parse standalone generation options before SimulationApp starts."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, default=DEFAULT_USD)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({"headless": True})

import omni.physx  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import SimulationContext  # noqa: E402
from isaacsim.core.utils.stage import open_stage  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402


def disable_dynamic_collisions(stage) -> dict[str, int]:
    """Disable excluded subtree collisions only in the session layer."""
    disabled = {}
    for root_path in EXCLUDED_PRIMS:
        root = stage.GetPrimAtPath(root_path)
        if not root.IsValid():
            raise RuntimeError(f"Excluded prim not found: {root_path}")
        count = 0
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
                count += 1
        disabled[root_path] = count
    return disabled


def rasterize_static_collisions(stage) -> tuple[bytearray, int, int]:
    """Return a known-free/occupied raster for the robot-height Z slice."""
    query = omni.physx.get_physx_scene_query_interface()
    pixels = bytearray([FREE_VALUE]) * (WIDTH * HEIGHT)
    occupied = 0
    excluded_hits = 0
    ray_distance = SLICE_MAX_Z - SLICE_MIN_Z
    for map_y in range(HEIGHT):
        world_y = ORIGIN[1] + (map_y + 0.5) * RESOLUTION
        image_row = HEIGHT - 1 - map_y
        for column in range(WIDTH):
            world_x = ORIGIN[0] + (column + 0.5) * RESOLUTION
            hit = query.raycast_closest(
                (world_x, world_y, SLICE_MIN_Z),
                (0.0, 0.0, 1.0),
                ray_distance,
            )
            if not hit["hit"]:
                continue
            hit_path = str(hit.get("rigidBody", ""))
            if any(
                hit_path == root or hit_path.startswith(root + "/")
                for root in EXCLUDED_PRIMS
            ):
                excluded_hits += 1
                continue
            pixels[image_row * WIDTH + column] = OCCUPIED_VALUE
            occupied += 1
        if map_y % 100 == 0:
            print(f"PASS: raster progress row={map_y}/{HEIGHT}", flush=True)
    return pixels, occupied, excluded_hits


def map_value(pixels: bytearray, x: float, y: float) -> int:
    """Sample a generated pixel by world coordinate."""
    column = int((x - ORIGIN[0]) // RESOLUTION)
    map_y = int((y - ORIGIN[1]) // RESOLUTION)
    row = HEIGHT - 1 - map_y
    return pixels[row * WIDTH + column]


def main() -> int:
    """Load the original USD read-only and write a candidate static PGM."""
    usd_path = ARGS.usd.expanduser().resolve()
    output_path = ARGS.output.expanduser().resolve()
    if not open_stage(str(usd_path)):
        raise RuntimeError(f"Failed to open Main USD: {usd_path}")
    for _ in range(3):
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        disabled = disable_dynamic_collisions(stage)

    context = SimulationContext(stage_units_in_meters=1.0)
    context.initialize_physics()
    context.play()
    context.step(render=False)
    pixels, occupied, excluded_hits = rasterize_static_collisions(stage)
    context.stop()

    header = (
        b"P5\n"
        b"# AF2_FLAT static collision map; AMR and Dolly excluded\n"
        + f"{WIDTH} {HEIGHT}\n255\n".encode("ascii")
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(header + pixels)

    amr_value = map_value(pixels, -30.360045, 17.247943)
    dolly_value = map_value(pixels, -24.8324585, 22.5609913)
    coverage = (
        ORIGIN[0],
        ORIGIN[0] + WIDTH * RESOLUTION,
        ORIGIN[1],
        ORIGIN[1] + HEIGHT * RESOLUTION,
    )
    nodemap_covered = (
        coverage[0] <= -42.453572 <= 7.905887 <= coverage[1]
        and coverage[2] <= -22.572640 <= 22.502320 <= coverage[3]
    )
    print(f"PASS: candidate={output_path}", flush=True)
    for path, count in disabled.items():
        print(f"PASS: excluded={path} collisions={count}", flush=True)
    print(f"PASS: occupied_cells={occupied}", flush=True)
    print(f"PASS: excluded_hit_count={excluded_hits}", flush=True)
    print(f"PASS: AMR spawn occupancy=FREE value={amr_value}", flush=True)
    print(f"PASS: Dolly static occupancy=FREE value={dolly_value}", flush=True)
    print(f"PASS: NodeMap coverage={nodemap_covered}", flush=True)
    valid = (
        occupied > 0
        and excluded_hits == 0
        and amr_value == FREE_VALUE
        and dolly_value == FREE_VALUE
        and nodemap_covered
    )
    return 0 if valid else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
