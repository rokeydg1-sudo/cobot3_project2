"""
Pilot SDG generator for Vision-based Safe Dolly Docking.

Purpose
-------
Generate a small synthetic YOLO Pose dataset from the currently opened
Isaac Sim scene.

Randomization
-------------
- Dolly distance
- Dolly lateral offset
- Dolly yaw

Output
------
outputs/pilot/
    images/
    labels/
    debug/
    metadata.csv

Important
---------
This is Pilot SDG v0.1.

Current visibility:
    - point is in front of the camera
    - point is inside the image
    - no rendered surface is significantly closer than the keypoint depth

Depth is used as an occluder test, not as an exact surface-membership test.
This is appropriate because the 8 Dolly keypoints are virtual geometric anchors.
"""

import asyncio
import csv
import math
import random
import sys
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np
import omni.usd
import omni.replicator.core as rep

from pxr import UsdGeom, Gf
from PIL import Image, ImageDraw


# ============================================================
# Find project directory from currently opened USD
# ============================================================

stage = omni.usd.get_context().get_stage()

root_layer_path = stage.GetRootLayer().realPath

if not root_layer_path:
    raise RuntimeError(
        "현재 Stage의 USD 파일 경로를 찾을 수 없습니다."
    )

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

VISION_DOCKING_DIR = (
    isaac_sim_dir
    / "vision_docking"
)


# ============================================================
# Project config
#
# Isaac Sim 환경에는 이미 "config"라는 이름의 모듈이 존재할 수
# 있으므로:
#
#     from config.vision_config import ...
#
# 방식은 사용하지 않는다.
#
# vision_config.py 파일을 경로로 직접 로드해서
# 모듈 이름 충돌을 방지한다.
# ============================================================

import importlib.util


CONFIG_PATH = (
    VISION_DOCKING_DIR
    / "config"
    / "vision_config.py"
)

if not CONFIG_PATH.exists():
    raise RuntimeError(
        f"vision_config.py not found: {CONFIG_PATH}"
    )


spec = importlib.util.spec_from_file_location(
    "dolly_vision_project_config",
    CONFIG_PATH,
)

if spec is None or spec.loader is None:
    raise RuntimeError(
        f"Failed to create module spec: {CONFIG_PATH}"
    )


vision_config = importlib.util.module_from_spec(
    spec
)

spec.loader.exec_module(
    vision_config
)


# ============================================================
# Config values
# ============================================================

CAMERA_PATH = vision_config.CAMERA_PATH
DOLLY_PATH = vision_config.DOLLY_PATH

IMAGE_WIDTH = vision_config.IMAGE_WIDTH
IMAGE_HEIGHT = vision_config.IMAGE_HEIGHT

CLASS_ID = vision_config.CLASS_ID

KEYPOINT_NAMES = (
    vision_config.KEYPOINT_NAMES
)

DOLLY_KEYPOINTS_LOCAL = (
    vision_config.DOLLY_KEYPOINTS_LOCAL
)

PILOT_NUM_FRAMES = (
    vision_config.PILOT_NUM_FRAMES
)

DISTANCE_RANGE_M = (
    vision_config.DISTANCE_RANGE_M
)

LATERAL_RANGE_M = (
    vision_config.LATERAL_RANGE_M
)

YAW_RANGE_DEG = (
    vision_config.YAW_RANGE_DEG
)

RANDOM_SEED = (
    vision_config.RANDOM_SEED
)

DOLLY_NOMINAL_YAW_DEG = (
    vision_config.DOLLY_NOMINAL_YAW_DEG
)

print(
    "[OK] Vision config loaded:",
    CONFIG_PATH,
)


# ============================================================
# Output
# ============================================================

OUTPUT_ROOT = (
    VISION_DOCKING_DIR
    / "outputs"
    / "pilot"
)

IMAGE_DIR = OUTPUT_ROOT / "images"
LABEL_DIR = OUTPUT_ROOT / "labels"
DEBUG_DIR = OUTPUT_ROOT / "debug"

LOG_PATH = (
    VISION_DOCKING_DIR
    / "outputs"
    / "logs"
    / "pilot_sdg.log"
)

METADATA_PATH = (
    OUTPUT_ROOT
    / "metadata.csv"
)

for directory in [
    IMAGE_DIR,
    LABEL_DIR,
    DEBUG_DIR,
    LOG_PATH.parent,
]:
    directory.mkdir(
        parents=True,
        exist_ok=True,
    )


# ============================================================
# Logging
# ============================================================

with open(
    LOG_PATH,
    "w",
    encoding="utf-8",
) as f:
    f.write(
        "===== DOLLY POSE PILOT SDG =====\n"
    )
    f.write(
        f"Start: {datetime.now().isoformat()}\n\n"
    )


def log(
    message="",
    console=False,
):

    text = str(message)

    # 전체 로그는 항상 파일로 저장
    with open(
        LOG_PATH,
        "a",
        encoding="utf-8",
    ) as f:

        f.write(
            text + "\n"
        )

        f.flush()

    # 꼭 필요한 내용만 Script Editor 출력
    if console:
        print(text)


# ============================================================
# Clean old pilot files
# ============================================================

for pattern, directory in [
    ("frame_*.png", IMAGE_DIR),
    ("frame_*.txt", LABEL_DIR),
    ("frame_*_overlay.png", DEBUG_DIR),
]:

    for path in directory.glob(pattern):
        path.unlink()


# 이전 실행이 실패했을 때 남은 metadata가
# 새 결과처럼 보이지 않도록 함께 삭제한다.
if METADATA_PATH.exists():
    METADATA_PATH.unlink()


# ============================================================
# Random seed
# ============================================================

random.seed(RANDOM_SEED)
np.random.seed(RANDOM_SEED)


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

    log(
        "[WARNING] Dolly parent is not /World: "
        f"{dolly_prim.GetParent().GetPath()}"
    )


# ============================================================
# Keypoint array
# ============================================================

keypoints_local = np.array(
    [
        DOLLY_KEYPOINTS_LOCAL[name]
        for name in KEYPOINT_NAMES
    ],
    dtype=np.float64,
)


# ============================================================
# Baseline Dolly pose
# ============================================================

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

log("===== SCENE =====")
log(f"Scene       : {scene_path}")
log(f"Camera      : {CAMERA_PATH}")
log(f"Dolly       : {DOLLY_PATH}")
log(f"Dolly Z     : {DOLLY_BASE_Z:.6f}")
log("")


# ============================================================
# Dolly transform ops
#
# Dolly root의 실제 Xform 구조:
#
#   xformOp:translate
#   xformOp:orient
#   xformOp:scale
#
# 따라서 XformCommonAPI.SetRotate()를 사용하지 않고
# 기존 XformOp를 직접 수정한다.
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
scale_op = None

for op in ordered_ops:

    op_name = op.GetOpName()

    log(
        f"{op_name} "
        f"type={op.GetOpType()} "
        f"precision={op.GetPrecision()}"
    )

    if op_name == "xformOp:translate":
        translate_op = op

    elif op_name == "xformOp:orient":
        orient_op = op

    elif op_name == "xformOp:scale":
        scale_op = op


if translate_op is None:
    raise RuntimeError(
        "Dolly xformOp:translate not found."
    )

if orient_op is None:
    raise RuntimeError(
        "Dolly xformOp:orient not found."
    )


log(
    f"Initial Dolly translate: "
    f"{translate_op.Get()}"
)

log(
    f"Initial Dolly orient: "
    f"{orient_op.Get()}"
)

log("")


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

rgb_annotator = (
    rep.annotators.get(
        "rgb"
    )
)

camera_params_annotator = (
    rep.annotators.get(
        "CameraParams"
    )
)

rgb_annotator.attach(
    render_product
)

camera_params_annotator.attach(
    render_product
)

# ============================================================
# Depth annotator
#
# Camera optical axis 방향의 depth.
# Keypoint의 camera-space depth와 비교하여
# self-occlusion을 판정한다.
# ============================================================

depth_annotator = rep.annotators.get(
    "distance_to_image_plane"
)

depth_annotator.attach(
    render_product
)

# ============================================================
# Helper: normalize 2D vector
# ============================================================

def normalize_xy(v):

    x = float(v[0])
    y = float(v[1])

    norm = math.sqrt(
        x * x + y * y
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


# ============================================================
# Helper: XformOp precision에 맞는 Vec3 생성
# ============================================================

def make_vec3_for_op(
    op,
    x,
    y,
    z,
):

    precision = op.GetPrecision()

    if (
        precision
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


# ============================================================
# Helper: yaw -> quaternion
#
# Dolly는 rotateXYZ가 아니라
# xformOp:orient(quaternion)를 사용한다.
# ============================================================

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

    precision = op.GetPrecision()

    if (
        precision
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


# ============================================================
# Helper: set Dolly pose
# ============================================================

def set_dolly_pose(
    world_x,
    world_y,
    world_z,
    yaw_deg,
):

    # Dolly가 /World 바로 아래 있으므로
    # 현재 scene에서는 local translation == world translation
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


    # --------------------------------------------------------
    # Translation
    # --------------------------------------------------------

    translate_ok = (
        translate_op.Set(
            translation
        )
    )

    if not translate_ok:

        raise RuntimeError(
            "Failed to set Dolly "
            "xformOp:translate."
        )


    # --------------------------------------------------------
    # Orientation
    # --------------------------------------------------------

    orient_ok = (
        orient_op.Set(
            orientation
        )
    )

    if not orient_ok:

        raise RuntimeError(
            "Failed to set Dolly "
            "xformOp:orient."
        )


# ============================================================
# Helper: local keypoints -> world
# ============================================================

def get_keypoints_world():

    cache = UsdGeom.XformCache()

    dolly_to_world = (
        cache
        .GetLocalToWorldTransform(
            dolly_prim
        )
    )

    points_world = []

    for point in keypoints_local:

        p_world = (
            dolly_to_world
            .Transform(
                Gf.Vec3d(
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                )
            )
        )

        points_world.append(
            [
                float(p_world[0]),
                float(p_world[1]),
                float(p_world[2]),
            ]
        )

    return np.asarray(
        points_world,
        dtype=np.float64,
    )


# ============================================================
# Helper: CameraParams projection
# ============================================================

def project_world_points(
    world_points,
    camera_params,
):

    view = np.asarray(
        camera_params[
            "cameraViewTransform"
        ],
        dtype=np.float64,
    ).reshape(4, 4)

    projection = np.asarray(
        camera_params[
            "cameraProjection"
        ],
        dtype=np.float64,
    ).reshape(4, 4)

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
                    "camera": camera_h,
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

        # camera_h[2]는 카메라 전방에 있을 때 음수이다.
        # distance_to_image_plane은 meters 단위이므로
        # scene unit -> meter 변환까지 적용한다.
        meters_per_scene_unit = float(
            camera_params.get(
                "metersPerSceneUnit",
                1.0,
            )
        )

        keypoint_depth = (
            -float(camera_h[2])
            * meters_per_scene_unit
        )

        visibility = (
            2 if in_frame else 0
        )

        results.append(
            {
                "u": float(u),
                "v": float(v),

                "in_frame": in_frame,

                # 아직 여기서는 FOV만 판정.
                # 실제 visibility는 Depth 검사에서 다시 결정.
                "visible": visibility,

                "keypoint_depth": keypoint_depth,

                "camera": camera_h,
            }
        )

    return results



# ============================================================
# Helper: Depth-based keypoint visibility
# ============================================================

def apply_depth_visibility(
    projected,
    depth_map,
    occlusion_margin_m=0.12,
):

    """
    Depth-based self-occlusion check.

    핵심 원리
    ---------
    이 프로젝트의 P1~P8은 Dolly 표면의 RGB 픽셀 자체가 아니라
    Dolly-local frame에 고정된 "가상 기하학 기준점"이다.

    따라서 다음처럼 "렌더 depth와 keypoint depth가 정확히 같은가?"를
    검사하면 대부분의 점이 false negative가 될 수 있다.

        abs(rendered_depth - keypoint_depth) < tolerance

    우리가 실제로 알고 싶은 것은 오직:

        "카메라와 keypoint 사이에 더 가까운 surface가 있는가?"

    이다.

    visible=0 (occluded)
        rendered_depth가 keypoint depth보다
        occlusion_margin_m 이상 가까움.

    visible=2
        화면 안에 있고, keypoint 앞을 가리는 surface가 없음.

    distance_to_image_plane에서 depth=0은 infinity,
    즉 해당 pixel에 렌더된 object가 없다는 의미이므로
    occluder가 없는 것으로 처리한다.
    """

    height, width = depth_map.shape[:2]

    for point in projected:

        point["rendered_depth"] = None
        point["depth_delta"] = None
        point["visibility_reason"] = "unknown"

        # ----------------------------------------------------
        # FOV 밖
        # ----------------------------------------------------

        if not point.get(
            "in_frame",
            False,
        ):
            point["visible"] = 0
            point["visibility_reason"] = "out_of_frame"
            continue


        u = int(round(point["u"]))
        v = int(round(point["v"]))

        # round() 때문에 경계에서 1 pixel 벗어나는 것 방지
        u = max(
            0,
            min(width - 1, u),
        )

        v = max(
            0,
            min(height - 1, v),
        )

        expected_depth = float(
            point["keypoint_depth"]
        )

        rendered_depth = float(
            depth_map[v, u]
        )


        # ----------------------------------------------------
        # depth=0 / NaN / Inf
        #
        # distance_to_image_plane에서 0은 infinity.
        # 즉 keypoint 앞을 가리는 rendered surface가 없음.
        # ----------------------------------------------------

        if (
            not np.isfinite(rendered_depth)
            or
            rendered_depth <= 0.0
        ):

            point["visible"] = 2
            point["visibility_reason"] = "no_occluder"
            point["rendered_depth"] = rendered_depth
            continue


        # ----------------------------------------------------
        # Occlusion 판정
        # ----------------------------------------------------

        depth_delta = (
            expected_depth
            - rendered_depth
        )

        point["rendered_depth"] = rendered_depth
        point["depth_delta"] = depth_delta


        # rendered surface가 keypoint보다 충분히 앞에 있으면
        # self-occluded로 판단한다.
        if depth_delta > occlusion_margin_m:

            point["visible"] = 0
            point["visibility_reason"] = "occluded"

        else:

            point["visible"] = 2
            point["visibility_reason"] = "not_occluded"


    return projected


# ============================================================
# Helper: keypoints -> pilot bbox
#
# v0.1:
# bbox is estimated from projected keypoint extent.
#
# Final dataset may replace this with a tighter object bbox.
# ============================================================

def get_bbox(
    projected,
    width,
    height,
):

    points = [
        p
        for p in projected
        if (
            p["u"] is not None
            and
            p["v"] is not None
        )
    ]

    if len(points) < 4:
        return None

    xs = np.asarray(
        [p["u"] for p in points],
        dtype=np.float64,
    )

    ys = np.asarray(
        [p["v"] for p in points],
        dtype=np.float64,
    )

    x_min = float(xs.min())
    x_max = float(xs.max())

    y_min = float(ys.min())
    y_max = float(ys.max())

    box_width = (
        x_max - x_min
    )

    box_height = (
        y_max - y_min
    )

    # 5% pilot padding
    pad_x = (
        box_width * 0.05
    )

    pad_y = (
        box_height * 0.05
    )

    x_min -= pad_x
    x_max += pad_x

    y_min -= pad_y
    y_max += pad_y

    x_min = max(
        0.0,
        min(float(width), x_min),
    )

    x_max = max(
        0.0,
        min(float(width), x_max),
    )

    y_min = max(
        0.0,
        min(float(height), y_min),
    )

    y_max = max(
        0.0,
        min(float(height), y_max),
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


# ============================================================
# Helper: YOLO Pose label
# ============================================================

def build_yolo_pose_label(
    bbox,
    projected,
    width,
    height,
):

    x_min, y_min, x_max, y_max = bbox

    bbox_cx = (
        (x_min + x_max)
        / 2.0
    )

    bbox_cy = (
        (y_min + y_max)
        / 2.0
    )

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

        if point["visible"] == 2:

            values.extend(
                [
                    f"{point['u'] / width:.6f}",
                    f"{point['v'] / height:.6f}",
                    "2",
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

    return " ".join(values)


# ============================================================
# Helper: debug overlay
# ============================================================

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

    radius = 7

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


        # --------------------------------------------------------
        # Depth-visible
        # --------------------------------------------------------

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
                    x + 9,
                    y - 10,
                ),
                name,
                fill=(0, 255, 0),
            )


        # --------------------------------------------------------
        # In-frame but Depth-occluded
        # --------------------------------------------------------

        else:

            size = 7

            draw.line(
                (
                    x - size,
                    y - size,
                    x + size,
                    y + size,
                ),
                fill=(255, 0, 0),
                width=3,
            )

            draw.line(
                (
                    x - size,
                    y + size,
                    x + size,
                    y - size,
                ),
                fill=(255, 0, 0),
                width=3,
            )

            draw.text(
                (
                    x + 9,
                    y - 10,
                ),
                f"{name} OCC",
                fill=(255, 0, 0),
            )

    image.save(
        output_path
    )


# ============================================================
# Main generation
# ============================================================

async def generate():

    metadata_rows = []

    try:

        log(
            "===== PILOT SDG START =====",
            console=True,
        )

        # ----------------------------------------------------
        # Initial render
        # ----------------------------------------------------

        await rep.orchestrator.step_async(
            rt_subframes=8
        )

        # ----------------------------------------------------
        # Camera ground-plane directions
        #
        # USD Camera:
        #   +X = right
        #   +Y = up
        #   -Z = forward
        # ----------------------------------------------------

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
            "Camera world position: "
            f"{camera_position}"
        )

        log(
            "Camera ground forward: "
            f"{forward_xy}"
        )

        log(
            "Camera ground right: "
            f"{right_xy}"
        )

        log("")

        saved_count = 0
        attempt_count = 0

        max_attempts = (
            PILOT_NUM_FRAMES * 10
        )


        # ====================================================
        # Sample loop
        # ====================================================

        while (
            saved_count
            < PILOT_NUM_FRAMES
            and
            attempt_count
            < max_attempts
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


            # ------------------------------------------------
            # Place Dolly relative to camera
            # ------------------------------------------------

            target_xy = (
                np.array(
                    [
                        float(camera_position[0]),
                        float(camera_position[1]),
                    ]
                )
                +
                forward_xy * distance
                +
                right_xy * lateral
            )

            set_dolly_pose(
                world_x=target_xy[0],
                world_y=target_xy[1],
                world_z=DOLLY_BASE_Z,
                yaw_deg=yaw_deg,
            )


            # ------------------------------------------------
            # Render updated frame
            # ------------------------------------------------

            await rep.orchestrator.step_async(
                rt_subframes=4
            )


            # ------------------------------------------------
            # Data
            # ------------------------------------------------

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

            world_points = (
                get_keypoints_world()
            )

            projected = (
                project_world_points(
                    world_points,
                    camera_params,
                )
            )

            projected = apply_depth_visibility(
                projected,
                depth,
            )

            visible_count = sum(
                1
                for p in projected
                if p["visible"] == 2
            )

            in_frame_count = sum(
                1
                for p in projected
                if p.get("in_frame", False)
            )

            occluded_count = sum(
                1
                for p in projected
                if (
                    p.get("in_frame", False)
                    and
                    p["visible"] == 0
                )
            )


            # ------------------------------------------------
            # For pilot PnP geometry:
            # require at least four visible points.
            # ------------------------------------------------

            if visible_count < 4:

                log(
                    f"[SKIP attempt={attempt_count}] "
                    f"visible={visible_count}"
                )

                continue


            bbox = get_bbox(
                projected,
                IMAGE_WIDTH,
                IMAGE_HEIGHT,
            )

            if bbox is None:

                log(
                    f"[SKIP attempt={attempt_count}] "
                    "invalid bbox"
                )

                continue


            # ------------------------------------------------
            # File name
            # ------------------------------------------------

            frame_name = (
                f"frame_{saved_count:04d}"
            )


            # ------------------------------------------------
            # RGB
            # ------------------------------------------------

            image_path = (
                IMAGE_DIR
                / f"{frame_name}.png"
            )

            Image.fromarray(
                rgb
            ).convert(
                "RGB"
            ).save(
                image_path
            )


            # ------------------------------------------------
            # YOLO Pose label
            # ------------------------------------------------

            label = (
                build_yolo_pose_label(
                    bbox,
                    projected,
                    IMAGE_WIDTH,
                    IMAGE_HEIGHT,
                )
            )

            label_path = (
                LABEL_DIR
                / f"{frame_name}.txt"
            )

            with open(
                label_path,
                "w",
                encoding="utf-8",
            ) as f:

                f.write(
                    label + "\n"
                )


            # ------------------------------------------------
            # Debug overlay
            # ------------------------------------------------

            debug_path = (
                DEBUG_DIR
                / f"{frame_name}_overlay.png"
            )

            save_debug_overlay(
                rgb,
                projected,
                bbox,
                debug_path,
            )


            # ------------------------------------------------
            # Metadata
            # ------------------------------------------------

            x_min, y_min, x_max, y_max = bbox

            metadata_rows.append(
                {
                    "frame": frame_name,
                    "distance_m": distance,
                    "lateral_m": lateral,
                    "yaw_deg": yaw_deg,
                    "visible_keypoints": visible_count,
                    "in_frame_keypoints": in_frame_count,
                    "occluded_keypoints": occluded_count,
                    "bbox_xmin": x_min,
                    "bbox_ymin": y_min,
                    "bbox_xmax": x_max,
                    "bbox_ymax": y_max,
                }
            )


            log(
                f"[SAVE {saved_count + 1:02d}/"
                f"{PILOT_NUM_FRAMES}] "
                f"{frame_name} | "
                f"d={distance:.3f} m | "
                f"lat={lateral:.3f} m | "
                f"yaw={yaw_deg:.2f} deg | "
                f"visible={visible_count} | "
                f"occluded={occluded_count}"
            )

            saved_count += 1


        # ====================================================
        # Metadata CSV
        # ====================================================

        if metadata_rows:

            with open(
                METADATA_PATH,
                "w",
                newline="",
                encoding="utf-8",
            ) as csv_file:

                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=
                    metadata_rows[0].keys(),
                )

                writer.writeheader()
                writer.writerows(
                    metadata_rows
                )


        # ====================================================
        # Result
        # ====================================================

        if saved_count < PILOT_NUM_FRAMES:

            raise RuntimeError(
                "Pilot dataset generation incomplete: "
                f"saved={saved_count}, "
                f"requested={PILOT_NUM_FRAMES}, "
                f"attempts={attempt_count}"
            )

        log("")
        log(
            "===== PILOT SDG COMPLETE =====",
            console=True,
        )

        log(
            f"Saved frames : {saved_count}"
        )

        log(
            f"Attempts     : {attempt_count}"
        )

        log(
            f"Images       : {IMAGE_DIR}"
        )

        log(
            f"Labels       : {LABEL_DIR}"
        )

        log(
            f"Debug        : {DEBUG_DIR}"
        )

        log(
            f"Metadata     : {METADATA_PATH}"
        )

        log(
            f"Log          : {LOG_PATH}"
        )

        print(
            "\nPILOT SDG COMPLETE\n"
            f"Output : {OUTPUT_ROOT}\n"
            f"Log    : {LOG_PATH}"
        )

    except Exception:

        error_text = (
            traceback.format_exc()
        )

        log("")
        log(
            "===== PILOT SDG ERROR ====="
        )

        log(
            error_text
        )

        print(
            "PILOT SDG ERROR\n"
            f"Full traceback saved to:\n"
            f"{LOG_PATH}"
        )


asyncio.ensure_future(
    generate()
)
