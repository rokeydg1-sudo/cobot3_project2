#!/usr/bin/env python3
"""Run the Dolly SDG generator headlessly.

    ~/isaacsim/python.sh scripts/run_sdg.py

`generate_training_dataset.py` was written for Isaac Sim's Script Editor: it
reads the already-open stage from `omni.usd.get_context()` and hands its work to
the running event loop with `asyncio.ensure_future`. Neither assumption holds in
a standalone process, so this launcher supplies both - it opens the factory USD
first, then pumps the app until the generator's future resolves.

Driving it from a script rather than by hand matters here because the run takes
tens of minutes and has to survive an unattended session.

Environment:
    SDG_USE_BACKDROP   1 to restore the flat colour card (default: real factory)
    SDG_TRAIN_FRAMES   override vision_config.TRAIN_NUM_FRAMES
    SDG_VAL_FRAMES     override vision_config.VAL_NUM_FRAMES
"""
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
ISAAC_DIR = REPO / "simulation" / "isaac_sim"
ASSET_DIR = Path(
    os.environ.get("FACTORY_ASSET_DIR", str(ISAAC_DIR / "Collected_AF2_FLAT"))
)
FACTORY_USD = os.environ.get(
    "FACTORY_USD", str(ASSET_DIR / "AF2_MULTI_BACKUP.usd")
)
SDG_SCRIPT = (
    ISAAC_DIR / "vision_docking" / "sdg" / "generate_training_dataset.py"
)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import asyncio  # noqa: E402

import omni.usd  # noqa: E402


def set_camera_fov(stage):
    """Left in place but no longer called. Do not re-enable without measuring.

    The intent was to make the SDG camera match the field of view the robot
    infers with. It does not work: the renderer ignores both the aperture and
    the focal length written here. Measured off rendered frames against the
    Dolly's known 1.242 m deck, the effective fx stayed at 1286 whether the
    focal length was 0.2364 or 0.2, and whether the requested angle was 60 or
    80 degrees.

    Since the renderer ignores the request, the SDG and the bridge already
    agree - both get the same authored ~28 degree lens - and forcing the value
    only desynchronises the intrinsics file from reality. The bridge made the
    same change; see create_docking_camera().
    """
    from pxr import UsdGeom
    import math

    sys.path.insert(
        0, str(ISAAC_DIR / "vision_docking" / "config")
    )
    import vision_config

    camera = stage.GetPrimAtPath(vision_config.CAMERA_PATH)
    if not camera.IsValid():
        print(
            f"[run_sdg] WARNING camera {vision_config.CAMERA_PATH} not found; "
            "field of view left as authored",
            flush=True,
        )
        return

    geom = UsdGeom.Camera(camera)
    focal = float(geom.GetFocalLengthAttr().Get())
    horizontal = 2.0 * focal * math.tan(math.radians(TRAINING_HFOV_DEG) / 2.0)
    vertical = horizontal * vision_config.IMAGE_HEIGHT / vision_config.IMAGE_WIDTH
    geom.GetHorizontalApertureAttr().Set(horizontal)
    geom.GetVerticalApertureAttr().Set(vertical)
    print(
        f"[run_sdg] camera set to {TRAINING_HFOV_DEG:.1f} deg horizontal "
        f"(focal={focal:.2f}, aperture={horizontal:.4f} x {vertical:.4f})",
        flush=True,
    )


def attach_camera(stage):
    """Reference the docking camera into the scene before the SDG looks for it.

    vision_config.CAMERA_PATH points at a prim the factory USD does not
    contain: the bridge references it in from iw_hub_sensors.usd at startup,
    and without that step the generator stops with "Camera not found". Doing
    the same thing here, the same way, keeps the training camera identical to
    the one used at inference - which is the whole reason to generate from this
    scene rather than a synthetic backdrop.
    """
    import sys as _sys

    _sys.path.insert(0, str(ISAAC_DIR / "vision_docking" / "config"))
    import vision_config

    camera_path = vision_config.CAMERA_PATH
    if stage.GetPrimAtPath(camera_path).IsValid():
        print(f"[run_sdg] camera already present: {camera_path}", flush=True)
        return

    sensors_usd = str(ASSET_DIR / "iw_hub_sensors.usd")
    if not Path(sensors_usd).is_file():
        raise SystemExit(f"sensor USD not found: {sensors_usd}")

    name = camera_path.rsplit("/", 1)[-1]
    parent = camera_path.rsplit("/", 1)[0]
    stage.DefinePrim(parent.rsplit("/", 1)[0], "Xform")
    stage.DefinePrim(parent, "Xform")
    camera = stage.DefinePrim(camera_path, "Camera")
    camera.GetReferences().AddReference(
        sensors_usd, f"/Root/iw_hub_sensors/camera_mount/{name}"
    )
    for _ in range(60):
        simulation_app.update()
    if not stage.GetPrimAtPath(camera_path).IsValid():
        raise SystemExit(f"could not reference camera into {camera_path}")
    print(f"[run_sdg] referenced camera at {camera_path}", flush=True)


def main():
    print(f"[run_sdg] opening {FACTORY_USD}", flush=True)
    context = omni.usd.get_context()
    context.open_stage(FACTORY_USD)

    # The stage loads asynchronously; stepping until it settles avoids the
    # generator reading a half-populated scene and reporting the Dolly missing.
    for _ in range(200):
        simulation_app.update()
    stage = context.get_stage()
    if stage is None:
        raise SystemExit("stage did not open")
    print(f"[run_sdg] stage root: {stage.GetRootLayer().realPath}", flush=True)

    attach_camera(stage)

    source = SDG_SCRIPT.read_text(encoding="utf-8")
    # The trailing `asyncio.ensure_future(generate())` schedules the work on a
    # loop this process has not started yet. Dropping that line and awaiting
    # the coroutine directly keeps the generator itself untouched.
    marker = "asyncio.ensure_future("
    if marker in source:
        source = source[: source.rindex(marker)]

    # Frame-count overrides, so the pipeline can be proven on twenty frames
    # before committing to a run of over a thousand. The generator reads these
    # from module globals it sets from vision_config, so they are patched after
    # the source is compiled but before it executes.
    namespace = {"__name__": "__sdg__", "__file__": str(SDG_SCRIPT)}
    exec(compile(source, str(SDG_SCRIPT), "exec"), namespace)

    for env_name, global_name in (
        ("SDG_TRAIN_FRAMES", "TRAIN_NUM_FRAMES"),
        ("SDG_VAL_FRAMES", "VAL_NUM_FRAMES"),
    ):
        override = os.environ.get(env_name)
        if override:
            namespace[global_name] = int(override)
            print(
                f"[run_sdg] {global_name} overridden to {override}", flush=True
            )

    generate = namespace.get("generate")
    if generate is None:
        raise SystemExit("generate() not found in the SDG script")

    loop = asyncio.get_event_loop()
    task = loop.create_task(generate())

    # Replicator's step_async only advances when the app does, so the app has
    # to be pumped from here rather than the loop simply being run to
    # completion.
    ticks = 0
    while not task.done():
        simulation_app.update()
        loop.run_until_complete(asyncio.sleep(0))
        ticks += 1
        if ticks % 2000 == 0:
            print(f"[run_sdg] still running, {ticks} app ticks", flush=True)

    if task.exception() is not None:
        import traceback

        traceback.print_exception(
            type(task.exception()), task.exception(),
            task.exception().__traceback__,
        )
        simulation_app.close()
        raise SystemExit(1)

    print("[run_sdg] generation finished", flush=True)
    simulation_app.close()


main()
