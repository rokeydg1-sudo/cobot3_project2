"""
Vision-based Safe Dolly Docking configuration.

이 파일은 비전 파트에서 공통으로 사용하는
Prim path, image size, keypoint 정의, SDG 범위를 관리한다.
"""


# ============================================================
# Isaac Sim Prim Paths
# ============================================================

CAMERA_PATH = (
    "/World/_23/iw_hub_01/iw_hub_sensors/camera_mount/"
    "transporter_camera_first_person"
)

DOLLY_PATH = "/World/dolly_physics"


# ============================================================
# Render Configuration
# ============================================================

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720


# ============================================================
# YOLO Pose
# ============================================================

CLASS_ID = 0
CLASS_NAME = "dolly"

KEYPOINT_NAMES = [
    "P1",
    "P2",
    "P3",
    "P4",
    "P5",
    "P6",
    "P7",
    "P8",
]


# ============================================================
# Dolly Local-frame 3D Keypoints
#
# Coordinate:
#   Dolly-local frame
#
# Unit:
#   meter
# ============================================================

# Corners of the deck, measured off the asset rather than estimated.
# scripts/measure_dolly.py reports FOF_Mesh_Shelf_Cart_B_LOD0 as
# 1.242 x 0.865 x 0.257 m with the deck spanning z = 0.222 to 0.479, so the
# half extents are 0.4325 across and 0.621 along, and the two z planes are the
# underside and the top surface.
#
# The previous values (0.400/0.430 and 0.600/0.620 at z 0.230/0.470) were
# already within about 3 cm of these, which settles a question that came up
# during debugging: the first model's poor transfer was not caused by wrong
# keypoints. The backdrop was the problem.
DOLLY_KEYPOINTS_LOCAL = {
    "P1": ( 0.4325,  0.621, 0.222),
    "P2": ( 0.4325, -0.621, 0.222),
    "P3": (-0.4325,  0.621, 0.222),
    "P4": (-0.4325, -0.621, 0.222),

    "P5": ( 0.4325,  0.621, 0.479),
    "P6": (-0.4325,  0.621, 0.479),
    "P7": ( 0.4325, -0.621, 0.479),
    "P8": (-0.4325, -0.621, 0.479),
}


# ============================================================
# Pilot SDG
# ============================================================

PILOT_NUM_FRAMES = 20

# Dolly와 Camera 사이의 대략적인 접근 거리
# Widened to the range the robot actually meets a Dolly over. The old 2.0-3.5 m
# window was narrower than the approach it had to cover, and combined with the
# authored 30-degree lens it left almost no distance at which a Dolly both fit
# in frame and filled enough of it to pass MIN_BBOX_AREA_RATIO.
DISTANCE_RANGE_M = (
    1.5,
    6.0,
)

# AMR 진행축 기준 Dolly 좌우 offset
LATERAL_RANGE_M = (
    -0.30,
    0.30,
)

# Dolly yaw error
YAW_RANGE_DEG = (
    -15.0,
    15.0,
)


# ============================================================
# Production SDG
# ============================================================

TRAIN_NUM_FRAMES = 1000
VAL_NUM_FRAMES = 200


TRAIN_RANDOM_SEED = 42
VAL_RANDOM_SEED = 4242


# ============================================================
# Simple Background Randomization
#
# RGB values in 0.0 ~ 1.0
# ============================================================

BACKGROUND_COLORS = [
    (0.08, 0.08, 0.08),   # dark gray
    (0.20, 0.20, 0.20),   # gray
    (0.45, 0.45, 0.45),   # medium gray
    (0.70, 0.70, 0.65),   # warm light gray
    (0.30, 0.35, 0.40),   # blue gray
]


# ============================================================
# Reproducibility
# ============================================================

RANDOM_SEED = 42


# Dolly가 Camera 진행축과 정상 정렬되어 있을 때의 yaw
DOLLY_NOMINAL_YAW_DEG = 0.0


TARGET_EDGE_TOUCH_RATIO = 0.30
CENTER_MARGIN_RATIO = 0.20
BRIGHT_BG_ONLY = True
# Lowered to match the wider distance range. At 60 degrees a Dolly at 6 m
# covers about 1.2% of the frame, so the old 0.08 floor would have thrown away
# every long-range sample - exactly the ones the approach needs.
MIN_BBOX_AREA_RATIO = 0.004
MAX_BBOX_AREA_RATIO = 0.75
MAX_CONSECUTIVE_SKIP_RESET = 20
