"""
Export exact camera intrinsics from Isaac Sim CameraParams.

Run from Isaac Sim Script Editor while:
- repository vision scene is open
- Timeline is STOPPED

Output:
vision/isaac_sim/vision_docking/config/camera_intrinsics.npz
"""

import asyncio
from pathlib import Path

import numpy as np
import omni.replicator.core as rep
import omni.usd


CAMERA_PATH = (
    "/World/iw_hub_sensors/camera_mount/"
    "transporter_camera_first_person"
)

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720


stage = omni.usd.get_context().get_stage()
scene_path = Path(
    stage.GetRootLayer().realPath
).resolve()

isaac_sim_dir = None
for parent in scene_path.parents:
    if parent.name == "isaac_sim":
        isaac_sim_dir = parent
        break

if isaac_sim_dir is None:
    raise RuntimeError(
        f"'isaac_sim' directory not found from scene: {scene_path}"
    )

output_path = (
    isaac_sim_dir
    / "vision_docking"
    / "config"
    / "camera_intrinsics.npz"
)

render_product = rep.create.render_product(
    CAMERA_PATH,
    resolution=(
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
    ),
)

camera_params = rep.annotators.get(
    "CameraParams"
)

camera_params.attach(
    render_product
)


def camera_xyz_to_pixel(
    x_cv,
    y_cv,
    z_cv,
    projection,
    width,
    height,
):
    """
    OpenCV camera coordinates:
        +X right
        +Y down
        +Z forward

    USD Camera coordinates:
        +X right
        +Y up
        -Z forward
    """

    camera_h = np.array(
        [
            float(x_cv),
            float(-y_cv),
            float(-z_cv),
            1.0,
        ],
        dtype=np.float64,
    )

    clip = camera_h @ projection

    ndc_x = float(
        clip[0] / clip[3]
    )
    ndc_y = float(
        clip[1] / clip[3]
    )

    u = (
        (ndc_x + 1.0)
        * 0.5
        * width
    )

    v = (
        (1.0 - ndc_y)
        * 0.5
        * height
    )

    return float(u), float(v)


async def export():

    try:
        await rep.orchestrator.step_async(
            rt_subframes=8
        )

        params = (
            camera_params
            .get_data()
        )

        projection = np.asarray(
            params[
                "cameraProjection"
            ],
            dtype=np.float64,
        ).reshape(
            4,
            4,
        )

        resolution = np.asarray(
            params[
                "renderProductResolution"
            ]
        )

        width = int(
            resolution[0]
        )
        height = int(
            resolution[1]
        )

        # Derive K directly from the exact Render Product projection.
        cx, cy = camera_xyz_to_pixel(
            0.0,
            0.0,
            1.0,
            projection,
            width,
            height,
        )

        ux, _ = camera_xyz_to_pixel(
            1.0,
            0.0,
            1.0,
            projection,
            width,
            height,
        )

        _, vy = camera_xyz_to_pixel(
            0.0,
            1.0,
            1.0,
            projection,
            width,
            height,
        )

        fx = ux - cx
        fy = vy - cy

        K = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        output_path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        np.savez(
            output_path,
            K=K,
            width=width,
            height=height,
            distortion=np.zeros(
                5,
                dtype=np.float64,
            ),
            camera_path=np.array(
                CAMERA_PATH
            ),
        )

        print("")
        print(
            "CAMERA INTRINSICS EXPORT: PASS"
        )
        print(
            f"fx={fx:.6f}, fy={fy:.6f}"
        )
        print(
            f"cx={cx:.6f}, cy={cy:.6f}"
        )
        print(
            f"Saved: {output_path}"
        )

    except Exception as exc:
        print("")
        print(
            "CAMERA INTRINSICS EXPORT: ERROR"
        )
        print(
            type(exc).__name__,
            str(exc),
        )


asyncio.ensure_future(
    export()
)
