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
# Pilot / production SDG metadata retained for reproducibility
# ============================================================

PILOT_NUM_FRAMES = 20
DISTANCE_RANGE_M = (2.0, 3.5)
LATERAL_RANGE_M = (-0.30, 0.30)
YAW_RANGE_DEG = (-15.0, 15.0)

TRAIN_NUM_FRAMES = 1000
VAL_NUM_FRAMES = 200
TRAIN_RANDOM_SEED = 42
VAL_RANDOM_SEED = 4242

BACKGROUND_COLORS = [
    (0.08, 0.08, 0.08),
    (0.20, 0.20, 0.20),
    (0.45, 0.45, 0.45),
    (0.70, 0.70, 0.65),
    (0.30, 0.35, 0.40),
]

RANDOM_SEED = 42
DOLLY_NOMINAL_YAW_DEG = 0.0

TARGET_EDGE_TOUCH_RATIO = 0.30
CENTER_MARGIN_RATIO = 0.20
BRIGHT_BG_ONLY = True
MIN_BBOX_AREA_RATIO = 0.08
MAX_BBOX_AREA_RATIO = 0.75
MAX_CONSECUTIVE_SKIP_RESET = 20
