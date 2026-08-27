#!/usr/bin/env python3
"""Measure the Dolly in the factory USD, one prim at a time.

Run with Isaac Sim's own Python:

    ~/isaacsim/python.sh scripts/measure_dolly.py

Why this exists
---------------
Two independent numbers for the same Dolly disagree by a factor of three.
`standalone_factory_bridge.py` reports a 1.24 x 0.86 x 0.49 m bounding box from
`UsdGeom.BBoxCache.ComputeWorldBound` on the Dolly root, while a camera frame
taken at a known 4.92 m shows the blue deck spanning more than 404 pixels, which
at fx = 554 is over 3.6 m and still clipped.

That matters more than it looks. `vision_config.DOLLY_KEYPOINTS_LOCAL` places
the deck corners at z = 0.23 and 0.47 m, the SDG projects those keypoints to
generate every training label, and the resulting model detects the Dolly on
roughly a quarter of frames. If the constants are wrong, retraining on them
reproduces the same failure. So the geometry has to be measured before anything
else is rebuilt on top of it.

Walking the children individually is the point: a bound computed on a root prim
whose geometry lives under an articulation can come back describing only part of
it, which would explain the disagreement.
"""
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_USD = os.path.join(
    REPO, "simulation", "isaac_sim", "Collected_AF2_FLAT", "AF2_MULTI_BACKUP.usd"
)


def bounds_of(cache, prim):
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        return None
    low, high = box.GetMin(), box.GetMax()
    return (
        [float(v) for v in low],
        [float(v) for v in high],
        [float(high[i] - low[i]) for i in range(3)],
    )


def report(cache, prim, indent=0):
    result = bounds_of(cache, prim)
    pad = "  " * indent
    if result is None:
        print(f"{pad}{prim.GetName():34s} type={prim.GetTypeName()} <empty bound>")
        return
    low, high, size = result
    print(
        f"{pad}{prim.GetName():34s} type={str(prim.GetTypeName()):12s} "
        f"size=({size[0]:6.3f}, {size[1]:6.3f}, {size[2]:6.3f})  "
        f"z=[{low[2]:6.3f}, {high[2]:6.3f}]"
    )


def main():
    usd_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_USD
    print(f"opening {usd_path}", flush=True)
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise SystemExit(f"could not open {usd_path}")

    # Every purpose, not just the default one. A deck authored as render-only
    # geometry is invisible to a default-purpose bound and would go missing
    # from the total exactly the way the bridge's number appears to.
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy,
            UsdGeom.Tokens.guide,
        ],
        useExtentsHint=False,
    )

    roots = [
        prim
        for prim in stage.Traverse()
        if prim.GetName().startswith("dolly")
    ]
    print(f"found {len(roots)} prim(s) whose name starts with 'dolly'")
    if not roots:
        print("\ntop-level prims under /World:")
        world = stage.GetPrimAtPath("/World")
        if world.IsValid():
            for child in world.GetChildren():
                print(f"  {child.GetName():40s} {child.GetTypeName()}")
        else:
            for prim in stage.GetPseudoRoot().GetChildren():
                print(f"  {prim.GetPath()}")

    for root in roots[:2]:
        print(f"\n=== {root.GetPath()} ===")
        report(cache, root)
        print("  children:")
        for child in root.GetChildren():
            report(cache, child, indent=2)
            for grandchild in child.GetChildren():
                report(cache, grandchild, indent=3)

    simulation_app.close()


main()
