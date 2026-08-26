"""
Vision-based Safe Dolly Docking configuration.

이 파일은 비전 파트에서 공통으로 사용하는
Prim path, image size, keypoint 정의, SDG 범위를 관리한다.
"""


# ============================================================
# Isaac Sim Prim Paths
# ============================================================

CAMERA_PATH = (
    "/World/iw_hub_sensors/camera_mount/"
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

DOLLY_KEYPOINTS_LOCAL = {
    "P1": ( 0.400,  0.600, 0.230),
    "P2": ( 0.400, -0.600, 0.230),
    "P3": (-0.400,  0.600, 0.230),
    "P4": (-0.400, -0.600, 0.230),

    "P5": ( 0.430,  0.620, 0.470),
    "P6": (-0.430,  0.620, 0.470),
    "P7": ( 0.430, -0.620, 0.470),
    "P8": (-0.430, -0.620, 0.470),
}


# ============================================================
# Pilot SDG
# ============================================================

PILOT_NUM_FRAMES = 20

# Dolly와 Camera 사이의 대략적인 접근 거리
DISTANCE_RANGE_M = (
    2.0,
    3.5,
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
MIN_BBOX_AREA_RATIO = 0.08
MAX_BBOX_AREA_RATIO = 0.75
MAX_CONSECUTIVE_SKIP_RESET = 20