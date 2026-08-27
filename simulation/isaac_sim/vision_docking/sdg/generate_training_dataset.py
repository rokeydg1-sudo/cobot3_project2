"""
Production SDG v1 for Vision-based Safe Dolly Docking.

Generates a YOLO Pose dataset from the currently opened Isaac Sim scene.

Production v1:
- Camera: fixed IW Hub first-person camera
- Dolly distance: config DISTANCE_RANGE_M (2.0~3.5 m)
- Dolly lateral offset: config LATERAL_RANGE_M
- Dolly yaw: config YAW_RANGE_DEG
- Keypoints: P1~P8 in Dolly-local coordinates
- Self-occlusion: distance_to_image_plane depth check
- BBox: actual Dolly USD geometry bound projected to image
- Background: simple color-randomized backdrop
- Split: train / val
- Built-in fast validation at the end

Run this file from Isaac Sim Script Editor while the repository scene is open
and Timeline is STOPPED.
"""

import asyncio
import csv
import importlib.util
import os
import math
import random
import shutil
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np
import omni.replicator.core as rep
import omni.usd

from PIL import Image, ImageDraw
from pxr import Gf, Usd, UsdGeom


# ============================================================
# Stage / project paths
# ============================================================

stage = omni.usd.get_context().get_stage()

root_layer_path = stage.GetRootLayer().realPath

if not root_layer_path:
    raise RuntimeError("현재 Stage의 USD 파일 경로를 찾을 수 없습니다.")

scene_path = Path(root_layer_path).resolve()

isaac_sim_dir = None

for parent in scene_path.parents:
    if parent.name == "isaac_sim":
        isaac_sim_dir = parent
        break

if isaac_sim_dir is None:
    raise RuntimeError(
        f"'isaac_sim' directory not found from scene: {scene_path}"
    )

VISION_DOCKING_DIR = isaac_sim_dir / "vision_docking"

CONFIG_PATH = (
    VISION_DOCKING_DIR
    / "config"
    / "vision_config.py"
)

if not CONFIG_PATH.exists():
    raise RuntimeError(
        f"vision_config.py not found: {CONFIG_PATH}"
    )


# ============================================================
# Load config without generic "config" module-name collision
# ============================================================

spec = importlib.util.spec_from_file_location(
    "dolly_vision_project_config",
    CONFIG_PATH,
)

if spec is None or spec.loader is None:
    raise RuntimeError(
        f"Failed to load config: {CONFIG_PATH}"
    )

vision_config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision_config)


CAMERA_PATH = vision_config.CAMERA_PATH
DOLLY_PATH = vision_config.DOLLY_PATH

IMAGE_WIDTH = vision_config.IMAGE_WIDTH
IMAGE_HEIGHT = vision_config.IMAGE_HEIGHT

CLASS_ID = vision_config.CLASS_ID
CLASS_NAME = vision_config.CLASS_NAME

KEYPOINT_NAMES = vision_config.KEYPOINT_NAMES
DOLLY_KEYPOINTS_LOCAL = vision_config.DOLLY_KEYPOINTS_LOCAL

DISTANCE_RANGE_M = vision_config.DISTANCE_RANGE_M
LATERAL_RANGE_M = vision_config.LATERAL_RANGE_M
YAW_RANGE_DEG = vision_config.YAW_RANGE_DEG
DOLLY_NOMINAL_YAW_DEG = vision_config.DOLLY_NOMINAL_YAW_DEG

TRAIN_NUM_FRAMES = vision_config.TRAIN_NUM_FRAMES
VAL_NUM_FRAMES = vision_config.VAL_NUM_FRAMES

TRAIN_RANDOM_SEED = vision_config.TRAIN_RANDOM_SEED
VAL_RANDOM_SEED = vision_config.VAL_RANDOM_SEED

# Render against the real factory instead of a flat coloured card.
#
# The backdrop was the single biggest reason the first model did not transfer.
# It parks a solid-colour cube a few metres behind the Dolly, so every training
# image showed a Dolly against one of five flat greys while every inference
# image showed one against white floor, grey machinery and blue plant painted
# the same hue as the deck. The trained weights peaked at 0.71 confidence on a
# clean live frame and fired on roughly a quarter of them.
#
# Keeping the switch rather than deleting the backdrop outright: a flat card is
# still the right choice for a quick shape-only sanity dataset, and leaving the
# code in place makes the comparison reproducible.
USE_BACKDROP = os.environ.get("SDG_USE_BACKDROP", "0") in ("1", "true", "True")

# Extra lighting fights the factory's own. Off by default for the same reason.
USE_TEMP_LIGHT = os.environ.get("SDG_USE_TEMP_LIGHT", "0") in (
    "1", "true", "True",
)

BACKGROUND_COLORS = getattr(
    vision_config,
    "BACKGROUND_COLORS",
    [
        (0.75, 0.75, 0.75),
        (0.60, 0.60, 0.60),
        (0.85, 0.85, 0.90),
        (0.70, 0.75, 0.80),
    ],
)

# Derive the camera basis from its transform. Default changed from the fixed
# +X assumption after it produced 200 rejections out of 200 attempts.
#
# The comment this replaces said the optical axis had been checked to be +X on
# the ground plane. It is not: the bridge reports this camera's mount at
# yaw = -90 degrees, so it looks along -Y. With the basis hardcoded to +X the
# generator placed every Dolly ninety degrees away from where the camera was
# pointing, and every frame failed the keypoint visibility test.
#
# The error stayed hidden while a flat backdrop was in use, because the
# backdrop was positioned from the same wrong forward vector and so appeared
# behind the Dolly regardless. Removing the backdrop to render against the
# real factory is what exposed it.
#
# Set USE_FIXED_CAMERA_BASIS in vision_config to override, but measure first.
USE_FIXED_CAMERA_BASIS = getattr(
    vision_config,
    "USE_FIXED_CAMERA_BASIS",
    False,
)

FIXED_CAMERA_FORWARD_XY = np.asarray(
    getattr(
        vision_config,
        "FIXED_CAMERA_FORWARD_XY",
        (1.0, 0.0),
    ),
    dtype=np.float64,
)

FIXED_CAMERA_RIGHT_XY = np.asarray(
    getattr(
        vision_config,
        "FIXED_CAMERA_RIGHT_XY",
        (0.0, -1.0),
    ),
    dtype=np.float64,
)


# ============================================================
# Output paths
# ============================================================

DATASET_ROOT = (
    VISION_DOCKING_DIR
    / "outputs"
    / "dataset"
)

LOG_PATH = (
    VISION_DOCKING_DIR
    / "outputs"
    / "logs"
    / "production_sdg.log"
)

DATASET_YAML_PATH = (
    DATASET_ROOT
    / "dataset.yaml"
)

BACKDROP_PATH = "/World/SDG_Backdrop"

EXPECTED_LABEL_VALUES = (
    1 + 4 + len(KEYPOINT_NAMES) * 3
)

MIN_VISIBLE_KEYPOINTS = 4

# Only a few debug images are kept.
DEBUG_SAMPLES_PER_SPLIT = 5

# Depth surface must be at least this much closer than the keypoint
# to be treated as a real occluder.
OCCLUSION_MARGIN_M = 0.12


# ============================================================
# Logging
# ============================================================

LOG_PATH.parent.mkdir(
    parents=True,
    exist_ok=True,
)

with open(
    LOG_PATH,
    "w",
    encoding="utf-8",
) as f:
    f.write(
        "===== PRODUCTION SDG V2 TROUBLESHOOT =====\n"
    )
    f.write(
        f"Start: {datetime.now().isoformat()}\n"
    )
    f.write(
        f"Scene: {scene_path}\n\n"
    )


def log(message="", console=False):

    text = str(message)

    with open(
        LOG_PATH,
        "a",
        encoding="utf-8",
    ) as f:
        f.write(text + "\n")
        f.flush()

    if console:
        print(text)


# ============================================================
# Clean/create dataset output
# ============================================================

if DATASET_ROOT.exists():
    shutil.rmtree(DATASET_ROOT)

for split in ("train", "val"):

    (
        DATASET_ROOT
        / "images"
        / split
    ).mkdir(
        parents=True,
        exist_ok=True,
    )

    (
        DATASET_ROOT
        / "labels"
        / split
    ).mkdir(
        parents=True,
        exist_ok=True,
    )

    (
        DATASET_ROOT
        / "debug"
        / split
    ).mkdir(
        parents=True,
        exist_ok=True,
    )


# ============================================================
# Prim validation
# ============================================================

camera_prim = stage.GetPrimAtPath(
    CAMERA_PATH
)

dolly_prim = stage.GetPrimAtPath(
    DOLLY_PATH
)

if not camera_prim.IsValid():
    raise RuntimeError(
        f"Camera not found: {CAMERA_PATH}"
    )

if not dolly_prim.IsValid():
    raise RuntimeError(
        f"Dolly not found: {DOLLY_PATH}"
    )

if str(dolly_prim.GetParent().GetPath()) != "/World":
    raise RuntimeError(
        "Production SDG v1 assumes Dolly is directly under /World. "
        f"Current parent: {dolly_prim.GetParent().GetPath()}"
    )


# ============================================================
# Dolly keypoints
# ============================================================

keypoints_local = np.array(
    [
        DOLLY_KEYPOINTS_LOCAL[name]
        for name in KEYPOINT_NAMES
    ],
    dtype=np.float64,
)


# ============================================================
# Dolly transform ops
# ============================================================

dolly_xformable = UsdGeom.Xformable(
    dolly_prim
)

ordered_ops = (
    dolly_xformable
    .GetOrderedXformOps()
)

translate_op = None
orient_op = None

for op in ordered_ops:

    op_name = op.GetOpName()

    if op_name == "xformOp:translate":
        translate_op = op

    elif op_name == "xformOp:orient":
        orient_op = op


if translate_op is None:
    raise RuntimeError(
        "Dolly xformOp:translate not found."
    )

if orient_op is None:
    raise RuntimeError(
        "Dolly xformOp:orient not found."
    )


initial_translate_value = translate_op.Get()
initial_orient_value = orient_op.Get()

initial_cache = UsdGeom.XformCache()

dolly_initial_matrix = (
    initial_cache
    .GetLocalToWorldTransform(
        dolly_prim
    )
)

dolly_initial_position = (
    dolly_initial_matrix
    .ExtractTranslation()
)

DOLLY_BASE_Z = float(
    dolly_initial_position[2]
)


# ============================================================
# Actual Dolly geometry bound in Dolly-root local space
#
# ComputeUntransformedBound excludes the transform authored on
# /World/dolly_physics itself, while including child geometry.
# ============================================================

bbox_cache = UsdGeom.BBoxCache(
    Usd.TimeCode.Default(),
    [
        UsdGeom.Tokens.default_,
        UsdGeom.Tokens.render,
    ],
    useExtentsHint=True,
    ignoreVisibility=False,
)

dolly_untransformed_bbox = (
    bbox_cache
    .ComputeUntransformedBound(
        dolly_prim
    )
)

dolly_local_range = (
    dolly_untransformed_bbox
    .ComputeAlignedRange()
)

bbox_min = (
    dolly_local_range
    .GetMin()
)

bbox_max = (
    dolly_local_range
    .GetMax()
)

if dolly_local_range.IsEmpty():
    raise RuntimeError(
        "Could not compute Dolly geometry bound."
    )


def make_box_corners(
    min_point,
    max_point,
):

    corners = []

    for x in (
        float(min_point[0]),
        float(max_point[0]),
    ):
        for y in (
            float(min_point[1]),
            float(max_point[1]),
        ):
            for z in (
                float(min_point[2]),
                float(max_point[2]),
            ):
                corners.append(
                    [x, y, z]
                )

    return np.asarray(
        corners,
        dtype=np.float64,
    )


dolly_bbox_corners_local = (
    make_box_corners(
        bbox_min,
        bbox_max,
    )
)

log(
    "Dolly local geometry bound: "
    f"min={tuple(float(v) for v in bbox_min)}, "
    f"max={tuple(float(v) for v in bbox_max)}"
)


# ============================================================
# Render Product + Annotators
# ============================================================

render_product = rep.create.render_product(
    CAMERA_PATH,
    resolution=(
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
    ),
)

rgb_annotator = rep.annotators.get(
    "rgb"
)

camera_params_annotator = (
    rep.annotators.get(
        "CameraParams"
    )
)

depth_annotator = (
    rep.annotators.get(
        "distance_to_image_plane"
    )
)

rgb_annotator.attach(
    render_product
)

camera_params_annotator.attach(
    render_product
)

depth_annotator.attach(
    render_product
)


# ============================================================
# Helpers
# ============================================================

def normalize_xy(v):

    x = float(v[0])
    y = float(v[1])

    norm = math.sqrt(
        x * x
        + y * y
    )

    if norm < 1e-8:
        raise RuntimeError(
            "Cannot normalize ground direction."
        )

    return np.array(
        [
            x / norm,
            y / norm,
        ],
        dtype=np.float64,
    )


def make_vec3_for_op(
    op,
    x,
    y,
    z,
):

    if (
        op.GetPrecision()
        == UsdGeom.XformOp.PrecisionFloat
    ):
        return Gf.Vec3f(
            float(x),
            float(y),
            float(z),
        )

    return Gf.Vec3d(
        float(x),
        float(y),
        float(z),
    )


def make_yaw_quaternion(
    op,
    yaw_deg,
):

    yaw_rad = math.radians(
        float(yaw_deg)
    )

    half = yaw_rad * 0.5

    real = math.cos(half)
    imag_z = math.sin(half)

    if (
        op.GetPrecision()
        == UsdGeom.XformOp.PrecisionFloat
    ):
        return Gf.Quatf(
            float(real),
            Gf.Vec3f(
                0.0,
                0.0,
                float(imag_z),
            ),
        )

    return Gf.Quatd(
        float(real),
        Gf.Vec3d(
            0.0,
            0.0,
            float(imag_z),
        ),
    )


def set_dolly_pose(
    world_x,
    world_y,
    world_z,
    yaw_deg,
):

    translation = make_vec3_for_op(
        translate_op,
        world_x,
        world_y,
        world_z,
    )

    orientation = make_yaw_quaternion(
        orient_op,
        yaw_deg,
    )

    if not translate_op.Set(
        translation
    ):
        raise RuntimeError(
            "Failed to set Dolly translate."
        )

    if not orient_op.Set(
        orientation
    ):
        raise RuntimeError(
            "Failed to set Dolly orient."
        )


def transform_local_points_to_world(
    local_points,
):

    cache = UsdGeom.XformCache()

    local_to_world = (
        cache
        .GetLocalToWorldTransform(
            dolly_prim
        )
    )

    world_points = []

    for point in local_points:

        world = (
            local_to_world
            .Transform(
                Gf.Vec3d(
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                )
            )
        )

        world_points.append(
            [
                float(world[0]),
                float(world[1]),
                float(world[2]),
            ]
        )

    return np.asarray(
        world_points,
        dtype=np.float64,
    )


def project_world_points(
    world_points,
    camera_params,
):

    view = np.asarray(
        camera_params[
            "cameraViewTransform"
        ],
        dtype=np.float64,
    ).reshape(
        4,
        4,
    )

    projection = np.asarray(
        camera_params[
            "cameraProjection"
        ],
        dtype=np.float64,
    ).reshape(
        4,
        4,
    )

    resolution = np.asarray(
        camera_params[
            "renderProductResolution"
        ]
    )

    width = int(
        resolution[0]
    )

    height = int(
        resolution[1]
    )

    meters_per_scene_unit = float(
        camera_params.get(
            "metersPerSceneUnit",
            1.0,
        )
    )

    results = []

    for p in world_points:

        world_h = np.array(
            [
                p[0],
                p[1],
                p[2],
                1.0,
            ],
            dtype=np.float64,
        )

        camera_h = (
            world_h
            @ view
        )

        clip_h = (
            camera_h
            @ projection
        )

        clip_w = float(
            clip_h[3]
        )

        if clip_w <= 1e-8:

            results.append(
                {
                    "u": None,
                    "v": None,
                    "in_frame": False,
                    "visible": 0,
                    "keypoint_depth": None,
                }
            )

            continue

        ndc_x = (
            float(clip_h[0])
            / clip_w
        )

        ndc_y = (
            float(clip_h[1])
            / clip_w
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

        in_frame = (
            0.0 <= u < width
            and
            0.0 <= v < height
        )

        keypoint_depth = (
            -float(camera_h[2])
            * meters_per_scene_unit
        )

        results.append(
            {
                "u": float(u),
                "v": float(v),
                "in_frame": in_frame,
                "visible": (
                    2
                    if in_frame
                    else 0
                ),
                "keypoint_depth": keypoint_depth,
            }
        )

    return results


def apply_depth_visibility(
    projected,
    depth_map,
):

    """
    Production visibility convention:

    v=0:
        outside image / behind camera

    v=1:
        keypoint projection is inside image but another rendered
        surface is significantly closer (self-occluded)

    v=2:
        inside image and not depth-occluded
    """

    height, width = (
        depth_map.shape[:2]
    )

    for point in projected:

        point["rendered_depth"] = None
        point["depth_delta"] = None
        point["visibility_reason"] = "unknown"

        if not point.get(
            "in_frame",
            False,
        ):
            point["visible"] = 0
            point["visibility_reason"] = "out_of_frame"
            continue

        u = int(
            round(point["u"])
        )

        v = int(
            round(point["v"])
        )

        u = max(
            0,
            min(
                width - 1,
                u,
            ),
        )

        v = max(
            0,
            min(
                height - 1,
                v,
            ),
        )

        expected_depth = float(
            point["keypoint_depth"]
        )

        rendered_depth = float(
            depth_map[v, u]
        )

        point["rendered_depth"] = (
            rendered_depth
        )

        if (
            not np.isfinite(
                rendered_depth
            )
            or
            rendered_depth <= 0.0
        ):

            point["visible"] = 2
            point["visibility_reason"] = (
                "no_occluder"
            )
            continue

        depth_delta = (
            expected_depth
            - rendered_depth
        )

        point["depth_delta"] = (
            depth_delta
        )

        if (
            depth_delta
            > OCCLUSION_MARGIN_M
        ):

            point["visible"] = 1
            point["visibility_reason"] = (
                "occluded"
            )

        else:

            point["visible"] = 2
            point["visibility_reason"] = (
                "not_occluded"
            )

    return projected


def projected_bbox_from_geometry(
    projected_corners,
    width,
    height,
):

    valid = [
        p
        for p in projected_corners
        if (
            p["u"] is not None
            and
            p["v"] is not None
        )
    ]

    if len(valid) < 4:
        return None

    xs = np.asarray(
        [
            p["u"]
            for p in valid
        ],
        dtype=np.float64,
    )

    ys = np.asarray(
        [
            p["v"]
            for p in valid
        ],
        dtype=np.float64,
    )

    x_min = max(
        0.0,
        float(xs.min()),
    )

    x_max = min(
        float(width),
        float(xs.max()),
    )

    y_min = max(
        0.0,
        float(ys.min()),
    )

    y_max = min(
        float(height),
        float(ys.max()),
    )

    if (
        x_max - x_min < 2.0
        or
        y_max - y_min < 2.0
    ):
        return None

    return (
        x_min,
        y_min,
        x_max,
        y_max,
    )


def build_yolo_pose_label(
    bbox,
    projected,
    width,
    height,
):

    x_min, y_min, x_max, y_max = bbox

    bbox_cx = (
        x_min + x_max
    ) / 2.0

    bbox_cy = (
        y_min + y_max
    ) / 2.0

    bbox_w = (
        x_max - x_min
    )

    bbox_h = (
        y_max - y_min
    )

    values = [
        str(CLASS_ID),
        f"{bbox_cx / width:.6f}",
        f"{bbox_cy / height:.6f}",
        f"{bbox_w / width:.6f}",
        f"{bbox_h / height:.6f}",
    ]

    for point in projected:

        visibility = int(
            point["visible"]
        )

        if visibility in (1, 2):

            values.extend(
                [
                    f"{point['u'] / width:.6f}",
                    f"{point['v'] / height:.6f}",
                    str(visibility),
                ]
            )

        else:

            values.extend(
                [
                    "0.000000",
                    "0.000000",
                    "0",
                ]
            )

    return " ".join(
        values
    )


def save_debug_overlay(
    rgb_array,
    projected,
    bbox,
    output_path,
):

    image = (
        Image
        .fromarray(rgb_array)
        .convert("RGB")
    )

    draw = ImageDraw.Draw(
        image
    )

    x_min, y_min, x_max, y_max = bbox

    draw.rectangle(
        (
            int(x_min),
            int(y_min),
            int(x_max),
            int(y_max),
        ),
        outline=(0, 255, 0),
        width=3,
    )

    radius = 6

    for name, point in zip(
        KEYPOINT_NAMES,
        projected,
    ):

        if not point.get(
            "in_frame",
            False,
        ):
            continue

        x = int(
            round(point["u"])
        )

        y = int(
            round(point["v"])
        )

        if point["visible"] == 2:

            draw.ellipse(
                (
                    x - radius,
                    y - radius,
                    x + radius,
                    y + radius,
                ),
                fill=(0, 255, 0),
                outline=(255, 255, 255),
                width=2,
            )

            draw.text(
                (
                    x + 8,
                    y - 10,
                ),
                name,
                fill=(0, 255, 0),
            )

        elif point["visible"] == 1:

            draw.line(
                (
                    x - radius,
                    y - radius,
                    x + radius,
                    y + radius,
                ),
                fill=(255, 0, 0),
                width=3,
            )

            draw.line(
                (
                    x - radius,
                    y + radius,
                    x + radius,
                    y - radius,
                ),
                fill=(255, 0, 0),
                width=3,
            )

            draw.text(
                (
                    x + 8,
                    y - 10,
                ),
                f"{name} OCC",
                fill=(255, 0, 0),
            )

    image.save(
        output_path
    )


# ============================================================
# Backdrop
# ============================================================

TEMP_LIGHT_PATH = "/World/SDG_TempLight"


def create_backdrop(
    camera_position,
    forward_xy,
):

    existing = stage.GetPrimAtPath(
        BACKDROP_PATH
    )

    if existing.IsValid():
        stage.RemovePrim(
            BACKDROP_PATH
        )

    cube = UsdGeom.Cube.Define(
        stage,
        BACKDROP_PATH,
    )

    cube.CreateSizeAttr(
        2.0
    )

    prim = cube.GetPrim()

    xformable = UsdGeom.Xformable(
        prim
    )

    xformable.ClearXformOpOrder()

    translate = (
        xformable
        .AddTranslateOp(
            UsdGeom.XformOp.PrecisionDouble
        )
    )

    orient = (
        xformable
        .AddOrientOp(
            UsdGeom.XformOp.PrecisionFloat
        )
    )

    scale = (
        xformable
        .AddScaleOp(
            UsdGeom.XformOp.PrecisionFloat
        )
    )

    backdrop_distance = (
        float(DISTANCE_RANGE_M[1])
        + 3.0
    )

    center_xy = (
        np.array(
            [
                float(camera_position[0]),
                float(camera_position[1]),
            ]
        )
        +
        forward_xy
        * backdrop_distance
    )

    translate.Set(
        Gf.Vec3d(
            float(center_xy[0]),
            float(center_xy[1]),
            2.0,
        )
    )

    yaw_deg = math.degrees(
        math.atan2(
            float(forward_xy[1]),
            float(forward_xy[0]),
        )
    )

    yaw_rad = math.radians(
        yaw_deg
    )

    orient.Set(
        Gf.Quatf(
            float(
                math.cos(
                    yaw_rad / 2.0
                )
            ),
            Gf.Vec3f(
                0.0,
                0.0,
                float(
                    math.sin(
                        yaw_rad / 2.0
                    )
                ),
            ),
        )
    )

    # Large, thin wall well behind the dolly.
    scale.Set(
        Gf.Vec3f(
            0.02,
            6.0,
            3.0,
        )
    )

    gprim = UsdGeom.Gprim(
        prim
    )

    display_color_attr = (
        gprim
        .CreateDisplayColorAttr()
    )

    display_color_attr.Set(
        [
            Gf.Vec3f(
                *BACKGROUND_COLORS[0]
            )
        ]
    )

    return display_color_attr


def create_temp_light(
    camera_position,
    forward_xy,
):

    try:
        from pxr import UsdLux
    except Exception:
        return

    existing = stage.GetPrimAtPath(
        TEMP_LIGHT_PATH
    )

    if existing.IsValid():
        stage.RemovePrim(
            TEMP_LIGHT_PATH
        )

    light = UsdLux.DistantLight.Define(
        stage,
        TEMP_LIGHT_PATH,
    )

    light.CreateIntensityAttr(
        2500.0
    )

    light.CreateColorAttr(
        Gf.Vec3f(
            1.0,
            1.0,
            1.0,
        )
    )

    # Keep it simple: default distant light is enough for this compact SDG.


def set_backdrop_color(
    display_color_attr,
    color,
):

    display_color_attr.Set(
        [
            Gf.Vec3f(
                float(color[0]),
                float(color[1]),
                float(color[2]),
            )
        ]
    )


# ============================================================
# Dataset yaml
# ============================================================

def write_dataset_yaml():

    # Horizontal image flip pairs:
    # P1<->P2, P3<->P4, P5<->P7, P6<->P8
    flip_idx = [
        1, 0,
        3, 2,
        6, 7,
        4, 5,
    ]

    yaml_text = (
        f"path: {DATASET_ROOT}\n"
        "train: images/train\n"
        "val: images/val\n\n"
        f"kpt_shape: [{len(KEYPOINT_NAMES)}, 3]\n"
        f"flip_idx: {flip_idx}\n\n"
        "names:\n"
        f"  {CLASS_ID}: {CLASS_NAME}\n"
    )

    DATASET_YAML_PATH.write_text(
        yaml_text,
        encoding="utf-8",
    )


# ============================================================
# Fast built-in validation
# ============================================================

def validate_generated_dataset():

    errors = []

    split_targets = {
        "train": TRAIN_NUM_FRAMES,
        "val": VAL_NUM_FRAMES,
    }

    for split, expected in (
        split_targets.items()
    ):

        image_dir = (
            DATASET_ROOT
            / "images"
            / split
        )

        label_dir = (
            DATASET_ROOT
            / "labels"
            / split
        )

        images = sorted(
            image_dir.glob(
                "*.png"
            )
        )

        labels = sorted(
            label_dir.glob(
                "*.txt"
            )
        )

        if len(images) != expected:
            errors.append(
                f"{split}: images "
                f"{len(images)} != {expected}"
            )

        if len(labels) != expected:
            errors.append(
                f"{split}: labels "
                f"{len(labels)} != {expected}"
            )

        for label_path in labels:

            values = (
                label_path
                .read_text(
                    encoding="utf-8"
                )
                .strip()
                .split()
            )

            if (
                len(values)
                != EXPECTED_LABEL_VALUES
            ):
                errors.append(
                    f"{label_path.name}: "
                    f"{len(values)} values"
                )
                continue

            try:
                nums = [
                    float(v)
                    for v in values
                ]
            except ValueError:
                errors.append(
                    f"{label_path.name}: "
                    "non-numeric label"
                )
                continue

            for value in nums[1:5]:

                if not (
                    0.0
                    <= value
                    <= 1.0
                ):
                    errors.append(
                        f"{label_path.name}: "
                        "bbox outside [0,1]"
                    )
                    break

            kp = nums[5:]
            visible_count = 0

            for i in range(
                len(KEYPOINT_NAMES)
            ):

                offset = i * 3

                x = kp[offset]
                y = kp[offset + 1]
                v = int(
                    kp[offset + 2]
                )

                if v not in (
                    0,
                    1,
                    2,
                ):
                    errors.append(
                        f"{label_path.name}: "
                        f"invalid visibility={v}"
                    )
                    continue

                if v in (
                    1,
                    2,
                ):

                    if not (
                        0.0 <= x <= 1.0
                        and
                        0.0 <= y <= 1.0
                    ):
                        errors.append(
                            f"{label_path.name}: "
                            "keypoint outside [0,1]"
                        )

                if v == 2:
                    visible_count += 1

            if (
                visible_count
                < MIN_VISIBLE_KEYPOINTS
            ):
                errors.append(
                    f"{label_path.name}: "
                    f"visible={visible_count}"
                )

    if not DATASET_YAML_PATH.exists():
        errors.append(
            "dataset.yaml missing"
        )

    return errors


# ============================================================
# Split generation
# ============================================================

async def generate_split(
    split,
    target_count,
    seed,
    camera_position,
    forward_xy,
    right_xy,
    backdrop_color_attr,
):

    random.seed(seed)
    np.random.seed(seed)

    image_dir = (
        DATASET_ROOT
        / "images"
        / split
    )

    label_dir = (
        DATASET_ROOT
        / "labels"
        / split
    )

    debug_dir = (
        DATASET_ROOT
        / "debug"
        / split
    )

    metadata_path = (
        DATASET_ROOT
        / f"metadata_{split}.csv"
    )

    metadata_rows = []

    saved_count = 0
    attempt_count = 0
    # Why each attempt was thrown away, so a run of zero saves is diagnosable.
    rejected = {}

    max_attempts = (
        target_count * 10
    )

    while (
        saved_count < target_count
        and
        attempt_count < max_attempts
    ):

        attempt_count += 1

        distance = random.uniform(
            *DISTANCE_RANGE_M
        )

        lateral = random.uniform(
            *LATERAL_RANGE_M
        )

        yaw_offset = random.uniform(
            *YAW_RANGE_DEG
        )

        yaw_deg = (
            DOLLY_NOMINAL_YAW_DEG
            + yaw_offset
        )

        background_index = (
            random.randrange(
                len(
                    BACKGROUND_COLORS
                )
            )
        )

        background_color = (
            BACKGROUND_COLORS[
                background_index
            ]
        )

        target_xy = (
            np.array(
                [
                    float(
                        camera_position[0]
                    ),
                    float(
                        camera_position[1]
                    ),
                ],
                dtype=np.float64,
            )
            +
            forward_xy
            * distance
            +
            right_xy
            * lateral
        )

        set_dolly_pose(
            world_x=target_xy[0],
            world_y=target_xy[1],
            world_z=DOLLY_BASE_Z,
            yaw_deg=yaw_deg,
        )

        if backdrop_color_attr is not None:
            set_backdrop_color(
                backdrop_color_attr,
                background_color,
            )

        await rep.orchestrator.step_async(
            rt_subframes=2
        )

        camera_params = (
            camera_params_annotator
            .get_data()
        )

        rgb = (
            rgb_annotator
            .get_data()
            .copy()
        )

        depth = np.asarray(
            depth_annotator.get_data(),
            dtype=np.float32,
        ).squeeze()

        # --------------------------------------------
        # Keypoints
        # --------------------------------------------

        keypoints_world = (
            transform_local_points_to_world(
                keypoints_local
            )
        )

        projected_keypoints = (
            project_world_points(
                keypoints_world,
                camera_params,
            )
        )

        projected_keypoints = (
            apply_depth_visibility(
                projected_keypoints,
                depth,
            )
        )

        visible_count = sum(
            1
            for p in projected_keypoints
            if p["visible"] == 2
        )

        occluded_count = sum(
            1
            for p in projected_keypoints
            if p["visible"] == 1
        )

        if (
            visible_count
            < MIN_VISIBLE_KEYPOINTS
        ):
            rejected["too few visible keypoints"] = (
                rejected.get("too few visible keypoints", 0) + 1
            )
            continue

        # --------------------------------------------
        # Actual Dolly geometry bbox
        # --------------------------------------------

        bbox_world = (
            transform_local_points_to_world(
                dolly_bbox_corners_local
            )
        )

        projected_bbox_corners = (
            project_world_points(
                bbox_world,
                camera_params,
            )
        )

        bbox = projected_bbox_from_geometry(
            projected_bbox_corners,
            IMAGE_WIDTH,
            IMAGE_HEIGHT,
        )

        if bbox is None:
            rejected["bbox off screen"] = rejected.get("bbox off screen", 0) + 1
            continue

        x_min, y_min, x_max, y_max = bbox
        bbox_cx = (x_min + x_max) / 2.0
        bbox_cy = (y_min + y_max) / 2.0

        # Extra sanity check:
        # if the dolly is completely absurdly off-center, reject.
        if not (
            0.10 * IMAGE_WIDTH <= bbox_cx <= 0.90 * IMAGE_WIDTH
            and
            0.05 * IMAGE_HEIGHT <= bbox_cy <= 0.95 * IMAGE_HEIGHT
        ):
            rejected["bbox too far off centre"] = (
                rejected.get("bbox too far off centre", 0) + 1
            )
            continue

        # --------------------------------------------
        # Save sample
        # --------------------------------------------

        frame_name = (
            f"{split}_{saved_count:05d}"
        )

        image_path = (
            image_dir
            / f"{frame_name}.png"
        )

        label_path = (
            label_dir
            / f"{frame_name}.txt"
        )

        Image.fromarray(
            rgb
        ).convert(
            "RGB"
        ).save(
            image_path
        )

        label = build_yolo_pose_label(
            bbox,
            projected_keypoints,
            IMAGE_WIDTH,
            IMAGE_HEIGHT,
        )

        label_path.write_text(
            label + "\n",
            encoding="utf-8",
        )

        if (
            saved_count
            < DEBUG_SAMPLES_PER_SPLIT
        ):

            save_debug_overlay(
                rgb,
                projected_keypoints,
                bbox,
                (
                    debug_dir
                    / f"{frame_name}_overlay.png"
                ),
            )

        metadata_rows.append(
            {
                "frame": frame_name,
                "distance_m": distance,
                "lateral_m": lateral,
                "yaw_deg": yaw_deg,
                "background_index": (
                    background_index
                ),
                "visible_keypoints": (
                    visible_count
                ),
                "occluded_keypoints": (
                    occluded_count
                ),
                "bbox_xmin": x_min,
                "bbox_ymin": y_min,
                "bbox_xmax": x_max,
                "bbox_ymax": y_max,
            }
        )

        saved_count += 1

        log(
            f"[{split}] "
            f"{saved_count}/{target_count} "
            f"d={distance:.3f} "
            f"lat={lateral:.3f} "
            f"yaw={yaw_deg:.2f} "
            f"vis={visible_count} "
            f"occ={occluded_count}"
        )

        if (
            saved_count % 100 == 0
            or
            saved_count == target_count
        ):
            print(
                f"[{split}] "
                f"{saved_count}/{target_count}"
            )

    if (
        saved_count
        < target_count
    ):
        # Say which filter did it. "saved=0, attempts=200" on its own gives
        # nothing to act on, and guessing at the thresholds is how an
        # afternoon disappears.
        breakdown = ", ".join(
            f"{reason}={count}"
            for reason, count in sorted(
                rejected.items(), key=lambda kv: -kv[1]
            )
        ) or "no rejections recorded"
        log(f"[{split}] rejection breakdown: {breakdown}")
        print(f"[{split}] rejection breakdown: {breakdown}", flush=True)
        raise RuntimeError(
            f"{split} generation incomplete: "
            f"saved={saved_count}, "
            f"target={target_count}, "
            f"attempts={attempt_count} "
            f"({breakdown})"
        )

    if metadata_rows:

        with open(
            metadata_path,
            "w",
            newline="",
            encoding="utf-8",
        ) as csv_file:

            writer = csv.DictWriter(
                csv_file,
                fieldnames=(
                    metadata_rows[0]
                    .keys()
                ),
            )

            writer.writeheader()
            writer.writerows(
                metadata_rows
            )


# ============================================================
# Main
# ============================================================

async def generate():

    try:

        log(
            "===== PRODUCTION SDG START =====",
            console=True,
        )

        await rep.orchestrator.step_async(
            rt_subframes=8
        )

        cache = UsdGeom.XformCache()

        camera_to_world = (
            cache
            .GetLocalToWorldTransform(
                camera_prim
            )
        )

        camera_position = (
            camera_to_world
            .ExtractTranslation()
        )

        if USE_FIXED_CAMERA_BASIS:

            forward_xy = normalize_xy(
                FIXED_CAMERA_FORWARD_XY
            )

            right_xy = normalize_xy(
                FIXED_CAMERA_RIGHT_XY
            )

            log(
                f"Using FIXED camera basis: "
                f"forward_xy={forward_xy}, "
                f"right_xy={right_xy}"
            )

        else:

            camera_forward_world = (
                camera_to_world
                .TransformDir(
                    Gf.Vec3d(
                        0.0,
                        0.0,
                        -1.0,
                    )
                )
            )

            camera_right_world = (
                camera_to_world
                .TransformDir(
                    Gf.Vec3d(
                        1.0,
                        0.0,
                        0.0,
                    )
                )
            )

            forward_xy = normalize_xy(
                camera_forward_world
            )

            right_xy = normalize_xy(
                camera_right_world
            )

            log(
                f"Using TRANSFORM-derived camera basis: "
                f"forward_xy={forward_xy}, "
                f"right_xy={right_xy}"
            )

        backdrop_color_attr = None

        if USE_BACKDROP:
            backdrop_color_attr = (
                create_backdrop(
                    camera_position,
                    forward_xy,
                )
            )
            print("[SDG] flat backdrop ENABLED", flush=True)
        else:
            print(
                "[SDG] no backdrop: rendering against the real factory",
                flush=True,
            )

        if USE_TEMP_LIGHT:
            create_temp_light(
                camera_position,
                forward_xy,
            )

        await rep.orchestrator.step_async(
            rt_subframes=4
        )

        await generate_split(
            split="train",
            target_count=(
                TRAIN_NUM_FRAMES
            ),
            seed=TRAIN_RANDOM_SEED,
            camera_position=(
                camera_position
            ),
            forward_xy=forward_xy,
            right_xy=right_xy,
            backdrop_color_attr=(
                backdrop_color_attr
            ),
        )

        await generate_split(
            split="val",
            target_count=(
                VAL_NUM_FRAMES
            ),
            seed=VAL_RANDOM_SEED,
            camera_position=(
                camera_position
            ),
            forward_xy=forward_xy,
            right_xy=right_xy,
            backdrop_color_attr=(
                backdrop_color_attr
            ),
        )

        write_dataset_yaml()

        validation_errors = (
            validate_generated_dataset()
        )

        if validation_errors:

            for error in validation_errors:
                log(
                    "[VALIDATION ERROR] "
                    + error
                )

            raise RuntimeError(
                "Generated dataset failed validation. "
                f"errors={len(validation_errors)}"
            )

        log("")
        log(
            "===== PRODUCTION SDG PASS ====="
        )

        log(
            f"Train : {TRAIN_NUM_FRAMES}"
        )

        log(
            f"Val   : {VAL_NUM_FRAMES}"
        )

        log(
            f"Dataset: {DATASET_ROOT}"
        )

        log(
            f"YAML: {DATASET_YAML_PATH}"
        )

        print("")
        print(
            "PRODUCTION SDG: PASS"
        )

        print(
            f"Train / Val : "
            f"{TRAIN_NUM_FRAMES} / "
            f"{VAL_NUM_FRAMES}"
        )

        print(
            f"Dataset : {DATASET_ROOT}"
        )

        print(
            f"Log     : {LOG_PATH}"
        )

    except Exception:

        error_text = (
            traceback.format_exc()
        )

        log("")
        log(
            "===== PRODUCTION SDG ERROR ====="
        )

        log(
            error_text
        )

        print("")
        print(
            "PRODUCTION SDG: ERROR"
        )

        print(
            f"Full traceback: {LOG_PATH}"
        )

    finally:

        # Restore Dolly pose.
        try:
            translate_op.Set(
                initial_translate_value
            )
            orient_op.Set(
                initial_orient_value
            )
        except Exception:
            pass

        # Remove temporary backdrop / light.
        try:
            if stage.GetPrimAtPath(
                BACKDROP_PATH
            ).IsValid():
                stage.RemovePrim(
                    BACKDROP_PATH
                )
        except Exception:
            pass

        try:
            if stage.GetPrimAtPath(
                TEMP_LIGHT_PATH
            ).IsValid():
                stage.RemovePrim(
                    TEMP_LIGHT_PATH
                )
        except Exception:
            pass


asyncio.ensure_future(
    generate()
)
