#!/usr/bin/env python3
"""
Compact runtime Dolly docking node.

ROS 2 input:
    sensor_msgs/Image

Runtime pipeline:
    RGB -> YOLO Pose -> confidence filter -> solvePnPRansac
        -> distance / lateral / yaw error
        -> simple P controller
        -> geometry_msgs/Twist

Important:
---------
- First run with publish_cmd_vel:=false (dry run).
- The current model was trained for roughly 2.0~3.5 m.
- Vision alignment hands off near the configured handoff distance.
- Final under-Dolly straight entry is handled open-loop after handoff.
- Runtime tuning is expected to come from config/docking.yaml.

Run after sourcing ROS 2 Jazzy + the vision venv.

"""

from pathlib import Path
import importlib.util
import math
import threading
import time

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from interfaces.action import DockDolly
from sensor_msgs.msg import Image


# ============================================================
# Runtime package paths
# ============================================================

THIS_FILE = Path(__file__).resolve()
PACKAGE_NAME = "vision_docking"


def resolve_package_share_dir():
    """Return installed ROS package share dir, with source-tree fallback."""
    try:
        return Path(
            get_package_share_directory(
                PACKAGE_NAME
            )
        )
    except PackageNotFoundError:
        # Source-tree fallback:
        # ros2_ws/src/vision_docking/vision_docking/dolly_docking_node.py
        return THIS_FILE.parents[1]


PACKAGE_SHARE_DIR = resolve_package_share_dir()

MODEL_PATH_DEFAULT = (
    PACKAGE_SHARE_DIR
    / "models"
    / "dolly_pose_v1_best.pt"
)

INTRINSICS_PATH_DEFAULT = (
    PACKAGE_SHARE_DIR
    / "config"
    / "camera_intrinsics.npz"
)

VISION_CONFIG_PATH = (
    PACKAGE_SHARE_DIR
    / "config"
    / "vision_config.py"
)


# ============================================================
# Docking tuning
# ============================================================
#
# Adjust normal docking behavior HERE.
#
# The current scene was calibrated from the successful live test:
# - vision handoff works reliably around 2.30 m
# - 2.55 m of open-loop final entry stopped around the Dolly entrance
# - add ~0.65 m so the IW Hub reaches farther under the Dolly
#
# NOTE:
# final-entry "distance" is an open-loop commanded-distance estimate:
#     speed [m/s] x time [s]
# It is not odometry feedback.
#

DEFAULT_IMAGE_TOPIC = "/vision/front_camera/image_raw"

# 정렬 -> 직진 전환 거리
HANDOFF_DISTANCE_M = 2.30

DONE_LATERAL_M = 0.20
DONE_YAW_DEG = 10.0

DRIVE_LATERAL_LIMIT_M = 0.20
DRIVE_YAW_LIMIT_DEG = 12.0

# 직진 속도(선속도) 범위
MAX_LINEAR_MPS = 0.12
MIN_LINEAR_MPS = 0.04

# 정렬 속도(각속도) 범위
MAX_ANGULAR_RPS = 0.25
MIN_ALIGN_ANGULAR_RPS = 0.08

# Final straight-entry calibration frozen from the current successful node.
# This value is also mirrored in config/docking.yaml.
FINAL_ENTRY_DISTANCE_M = 4.60

# Slightly faster than the previous 0.15 m/s.
# Keep below the current Isaac graph 0.20 m/s linear-speed clamp.
FINAL_ENTRY_SPEED_MPS = 0.18

FALLBACK_DISTANCE_M = 2.35
FALLBACK_LATERAL_M = 0.23
FALLBACK_YAW_DEG = 11.0
INVALID_FRAMES_BEFORE_HANDOFF = 3


# ============================================================
# Helpers
# ============================================================

def clamp(value, minimum, maximum):
    return max(
        minimum,
        min(
            maximum,
            value,
        ),
    )


def wrap_deg(angle):
    return (
        angle + 180.0
    ) % 360.0 - 180.0


def load_vision_config():

    spec = (
        importlib.util
        .spec_from_file_location(
            "dolly_vision_runtime_config",
            VISION_CONFIG_PATH,
        )
    )

    if (
        spec is None
        or spec.loader is None
    ):
        raise RuntimeError(
            f"Cannot load config: {VISION_CONFIG_PATH}"
        )

    module = (
        importlib.util
        .module_from_spec(
            spec
        )
    )

    spec.loader.exec_module(
        module
    )

    return module


# ============================================================
# Runtime node
# ============================================================

class DollyDockingNode(Node):

    def __init__(self):

        super().__init__(
            "dolly_docking_node"
        )

        # ----------------------------------------------------
        # ROS parameters
        # ----------------------------------------------------

        self.declare_parameter(
            "image_topic",
            DEFAULT_IMAGE_TOPIC,
        )

        self.declare_parameter(
            "cmd_vel_topic",
            "/cmd_vel",
        )

        self.declare_parameter(
            "publish_cmd_vel",
            False,
        )

        self.declare_parameter(
            "model_path",
            str(MODEL_PATH_DEFAULT),
        )

        self.declare_parameter(
            "intrinsics_path",
            str(INTRINSICS_PATH_DEFAULT),
        )

        self.declare_parameter(
            "process_hz",
            10.0,
        )

        self.declare_parameter(
            "detection_conf",
            0.25,
        )

        self.declare_parameter(
            "keypoint_conf",
            0.35,
        )

        self.declare_parameter(
            "pnp_reprojection_error_px",
            12.0,
        )

        # Current SDG/model operating range starts at ~2.0 m.
        self.declare_parameter(
            "handoff_distance_m",
            HANDOFF_DISTANCE_M,
        )

        # After vision alignment reaches the handoff point,
        # move straight open-loop until the AMR reference is
        # approximately centered under the Dolly.
        self.declare_parameter(
            "enable_final_entry",
            True,
        )

        self.declare_parameter(
            "final_entry_distance_m",
            FINAL_ENTRY_DISTANCE_M,
        )

        self.declare_parameter(
            "final_entry_speed_mps",
            FINAL_ENTRY_SPEED_MPS,
        )

        self.declare_parameter(
            "image_watchdog_timeout_s",
            0.75,
        )

        # Near the Dolly, keypoints can leave the camera FOV.
        # If several consecutive PnP frames fail after the last
        # valid pose was already close/aligned enough, hand off
        # to straight entry instead of waiting forever.
        self.declare_parameter(
            "invalid_frames_before_handoff",
            INVALID_FRAMES_BEFORE_HANDOFF,
        )

        self.declare_parameter(
            "fallback_distance_m",
            FALLBACK_DISTANCE_M,
        )

        self.declare_parameter(
            "fallback_lateral_m",
            FALLBACK_LATERAL_M,
        )

        self.declare_parameter(
            "fallback_yaw_deg",
            FALLBACK_YAW_DEG,
        )

        self.declare_parameter(
            "done_lateral_m",
            DONE_LATERAL_M,
        )

        self.declare_parameter(
            "done_yaw_deg",
            DONE_YAW_DEG,
        )

        # Stop forward motion when alignment error is too large.
        self.declare_parameter(
            "drive_lateral_limit_m",
            DRIVE_LATERAL_LIMIT_M,
        )

        self.declare_parameter(
            "drive_yaw_limit_deg",
            DRIVE_YAW_LIMIT_DEG,
        )

        # Compact P control.
        self.declare_parameter(
            "k_distance",
            0.25,
        )

        self.declare_parameter(
            "k_yaw",
            0.9,
        )

        self.declare_parameter(
            "k_lateral",
            1.2,
        )

        self.declare_parameter(
            "max_linear_mps",
            MAX_LINEAR_MPS,
        )

        self.declare_parameter(
            "min_linear_mps",
            MIN_LINEAR_MPS,
        )

        self.declare_parameter(
            "max_angular_rps",
            MAX_ANGULAR_RPS,
        )

        self.declare_parameter(
            "min_align_angular_rps",
            MIN_ALIGN_ANGULAR_RPS,
        )

        self.declare_parameter(
            "angular_sign",
            1.0,
        )

        # PnP smoke test showed that the raw estimated yaw must
        # be flipped to match the project Dolly yaw convention.
        self.declare_parameter(
            "yaw_sign",
            -1.0,
        )

        # ----------------------------------------------------
        # Read parameters
        # ----------------------------------------------------

        self.image_topic = str(
            self.get_parameter(
                "image_topic"
            ).value
        )

        self.cmd_vel_topic = str(
            self.get_parameter(
                "cmd_vel_topic"
            ).value
        )

        self.publish_cmd_vel = bool(
            self.get_parameter(
                "publish_cmd_vel"
            ).value
        )

        self.process_hz = float(
            self.get_parameter(
                "process_hz"
            ).value
        )

        self.detection_conf = float(
            self.get_parameter(
                "detection_conf"
            ).value
        )

        self.keypoint_conf = float(
            self.get_parameter(
                "keypoint_conf"
            ).value
        )

        self.pnp_reprojection_error_px = float(
            self.get_parameter(
                "pnp_reprojection_error_px"
            ).value
        )

        self.handoff_distance_m = float(
            self.get_parameter(
                "handoff_distance_m"
            ).value
        )

        self.enable_final_entry = bool(
            self.get_parameter(
                "enable_final_entry"
            ).value
        )

        self.final_entry_distance_m = float(
            self.get_parameter(
                "final_entry_distance_m"
            ).value
        )

        self.final_entry_speed_mps = float(
            self.get_parameter(
                "final_entry_speed_mps"
            ).value
        )

        self.image_watchdog_timeout_s = float(
            self.get_parameter(
                "image_watchdog_timeout_s"
            ).value
        )

        self.invalid_frames_before_handoff = int(
            self.get_parameter(
                "invalid_frames_before_handoff"
            ).value
        )

        self.fallback_distance_m = float(
            self.get_parameter(
                "fallback_distance_m"
            ).value
        )

        self.fallback_lateral_m = float(
            self.get_parameter(
                "fallback_lateral_m"
            ).value
        )

        self.fallback_yaw_deg = float(
            self.get_parameter(
                "fallback_yaw_deg"
            ).value
        )

        self.done_lateral_m = float(
            self.get_parameter(
                "done_lateral_m"
            ).value
        )

        self.done_yaw_deg = float(
            self.get_parameter(
                "done_yaw_deg"
            ).value
        )

        self.drive_lateral_limit_m = float(
            self.get_parameter(
                "drive_lateral_limit_m"
            ).value
        )

        self.drive_yaw_limit_deg = float(
            self.get_parameter(
                "drive_yaw_limit_deg"
            ).value
        )

        self.k_distance = float(
            self.get_parameter(
                "k_distance"
            ).value
        )

        self.k_yaw = float(
            self.get_parameter(
                "k_yaw"
            ).value
        )

        self.k_lateral = float(
            self.get_parameter(
                "k_lateral"
            ).value
        )

        self.max_linear_mps = float(
            self.get_parameter(
                "max_linear_mps"
            ).value
        )

        self.min_linear_mps = float(
            self.get_parameter(
                "min_linear_mps"
            ).value
        )

        self.max_angular_rps = float(
            self.get_parameter(
                "max_angular_rps"
            ).value
        )

        self.min_align_angular_rps = float(
            self.get_parameter(
                "min_align_angular_rps"
            ).value
        )

        self.angular_sign = float(
            self.get_parameter(
                "angular_sign"
            ).value
        )

        self.yaw_sign = float(
            self.get_parameter(
                "yaw_sign"
            ).value
        )

        model_path = Path(
            str(
                self.get_parameter(
                    "model_path"
                ).value
            )
        )

        intrinsics_path = Path(
            str(
                self.get_parameter(
                    "intrinsics_path"
                ).value
            )
        )

        if not model_path.exists():
            raise FileNotFoundError(
                f"Model not found: {model_path}"
            )

        if not intrinsics_path.exists():
            raise FileNotFoundError(
                f"Intrinsics not found: {intrinsics_path}"
            )

        # ----------------------------------------------------
        # Vision / PnP setup
        # ----------------------------------------------------

        config = load_vision_config()

        self.object_points_all = np.asarray(
            [
                config.DOLLY_KEYPOINTS_LOCAL[
                    name
                ]
                for name in config.KEYPOINT_NAMES
            ],
            dtype=np.float64,
        )

        intrinsics = np.load(
            intrinsics_path
        )

        self.K = np.asarray(
            intrinsics["K"],
            dtype=np.float64,
        )

        self.dist_coeffs = np.asarray(
            intrinsics["distortion"],
            dtype=np.float64,
        )

        self.model = YOLO(
            str(model_path)
        )

        self.bridge = CvBridge()

        # ----------------------------------------------------
        # Docking mission synchronization
        # ----------------------------------------------------

        self.mission_lock = threading.Lock()
        self.docking_done_event = threading.Event()

        self.goal_in_progress = False
        self.docking_active = False
        self.docking_state = "IDLE"
        self.docking_success = False
        self.docking_message = ""

        self.latest_distance_m = float("nan")
        self.latest_lateral_m = float("nan")
        self.latest_yaw_deg = float("nan")

        # Action execute waits in its own executor thread. A reentrant
        # group lets cancel requests be processed while it is waiting.
        self.action_group = ReentrantCallbackGroup()
        self.image_group = MutuallyExclusiveCallbackGroup()
        self.watchdog_group = MutuallyExclusiveCallbackGroup()

        # ----------------------------------------------------
        # ROS interfaces
        # ----------------------------------------------------

        self.cmd_pub = (
            self.create_publisher(
                Twist,
                self.cmd_vel_topic,
                10,
            )
        )

        self.image_sub = (
            self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                10,
                callback_group=self.image_group,
            )
        )

        self.dock_action_server = ActionServer(
            self,
            DockDolly,
            "/dock_dolly",
            execute_callback=self.execute_docking_callback,
            goal_callback=self.docking_goal_callback,
            cancel_callback=self.docking_cancel_callback,
            callback_group=self.action_group,
        )

        self.min_process_period = (
            1.0
            / max(
                1e-6,
                self.process_hz,
            )
        )

        self.last_process_time = 0.0

        self.last_log_time = 0.0

        self.last_image_time = time.monotonic()

        self.final_entry_active = False
        self.final_entry_complete = False
        self.final_entry_start_time = None

        self.last_valid_pose = None
        self.invalid_pose_count = 0

        self.watchdog_timer = self.create_timer(
            0.10,
            self.watchdog_callback,
            callback_group=self.watchdog_group,
        )

        self.get_logger().info(
            "Dolly docking node ready"
        )

        self.get_logger().info(
            f"image_topic={self.image_topic}"
        )

        self.get_logger().info(
            f"cmd_vel_topic={self.cmd_vel_topic}"
        )

        self.get_logger().info(
            "dock_action=/dock_dolly"
        )

        self.get_logger().info(
            f"package_share={PACKAGE_SHARE_DIR}"
        )

        self.get_logger().info(
            f"model_path={model_path}"
        )

        self.get_logger().info(
            f"intrinsics_path={intrinsics_path}"
        )

        self.get_logger().info(
            f"publish_cmd_vel={self.publish_cmd_vel}"
        )

        self.get_logger().info(
            f"handoff_distance={self.handoff_distance_m:.2f} m"
        )

        self.get_logger().info(
            f"approach speed <= {self.max_linear_mps:.2f} m/s | "
            f"align angular <= {self.max_angular_rps:.2f} rad/s"
        )

        self.get_logger().info(
            f"final_entry={self.enable_final_entry} | "
            f"command_distance={self.final_entry_distance_m:.2f} m | "
            f"speed={self.final_entry_speed_mps:.2f} m/s"
        )

        if not self.publish_cmd_vel:

            self.get_logger().warn(
                "DRY RUN: cmd_vel publishing is disabled."
            )

    # ========================================================
    # DockDolly Action Server
    # ========================================================

    def reset_docking_runtime_locked(self):
        """Reset mission-specific runtime state while holding mission_lock."""
        self.docking_done_event.clear()
        self.docking_state = "WAITING_FOR_VISION"
        self.docking_success = False
        self.docking_message = ""

        self.latest_distance_m = float("nan")
        self.latest_lateral_m = float("nan")
        self.latest_yaw_deg = float("nan")

        self.last_process_time = 0.0
        self.last_log_time = 0.0
        self.last_image_time = time.monotonic()

        self.final_entry_active = False
        self.final_entry_complete = False
        self.final_entry_start_time = None

        self.last_valid_pose = None
        self.invalid_pose_count = 0

    def docking_goal_callback(self, goal_request):
        """Accept one docking mission at a time and reserve the controller."""
        with self.mission_lock:
            if self.goal_in_progress:
                self.get_logger().warn(
                    "DockDolly Goal rejected: docking is already active."
                )
                return GoalResponse.REJECT

            self.goal_in_progress = True
            self.docking_active = True
            self.reset_docking_runtime_locked()

        self.get_logger().info(
            "DockDolly Goal accepted | "
            f"amr_id={goal_request.amr_id} | "
            f"task_id={goal_request.task_id}"
        )

        return GoalResponse.ACCEPT

    def docking_cancel_callback(self, _goal_handle):
        """Stop immediately and wake the execute callback on cancellation."""
        with self.mission_lock:
            active = self.goal_in_progress

        if not active:
            return CancelResponse.REJECT

        self.finish_docking(
            success=False,
            message="DOCKING_CANCELED",
            state="CANCELED",
        )
        self.get_logger().warn("DockDolly cancel requested -> STOP")

        return CancelResponse.ACCEPT

    def docking_feedback_snapshot(self):
        with self.mission_lock:
            return (
                self.docking_state,
                self.latest_distance_m,
                self.latest_lateral_m,
                self.latest_yaw_deg,
            )

    def update_docking_feedback(self, state, pose=None):
        with self.mission_lock:
            if not self.docking_active:
                return

            self.docking_state = state

            if pose is not None:
                self.latest_distance_m = float(pose["distance_m"])
                self.latest_lateral_m = float(pose["lateral_m"])
                self.latest_yaw_deg = float(pose["yaw_deg"])

    def is_docking_active(self):
        with self.mission_lock:
            return self.docking_active

    def finish_docking(self, success, message, state):
        """Atomically stop controller ownership and complete the mission."""
        with self.mission_lock:
            if not self.goal_in_progress or self.docking_done_event.is_set():
                return

            self.docking_active = False
            self.docking_success = bool(success)
            self.docking_message = str(message)
            self.docking_state = str(state)

        # Publish STOP after revoking non-zero command ownership.
        self.stop_robot()
        self.docking_done_event.set()

    def execute_docking_callback(self, goal_handle):
        """Wait for image callbacks to finish docking and publish feedback."""
        result = DockDolly.Result()

        try:
            while not self.docking_done_event.wait(timeout=0.20):
                if goal_handle.is_cancel_requested:
                    self.finish_docking(
                        success=False,
                        message="DOCKING_CANCELED",
                        state="CANCELED",
                    )
                    break

                state, distance_m, lateral_m, yaw_deg = (
                    self.docking_feedback_snapshot()
                )

                feedback = DockDolly.Feedback()
                feedback.state = state
                feedback.distance_m = distance_m
                feedback.lateral_m = lateral_m
                feedback.yaw_deg = yaw_deg
                goal_handle.publish_feedback(feedback)

            with self.mission_lock:
                success = self.docking_success
                message = self.docking_message

            result.success = success
            result.message = message

            if goal_handle.is_cancel_requested or message == "DOCKING_CANCELED":
                goal_handle.canceled()
            elif success:
                goal_handle.succeed()
            else:
                goal_handle.abort()

            return result

        except Exception as exc:
            self.finish_docking(
                success=False,
                message=(
                    "ACTION_RUNTIME_ERROR: "
                    f"{type(exc).__name__}: {exc}"
                ),
                state="FAILED",
            )

            result.success = False
            result.message = self.docking_message
            goal_handle.abort()
            return result

        finally:
            with self.mission_lock:
                self.docking_active = False
                self.goal_in_progress = False
                self.docking_state = "IDLE"

    # ========================================================
    # Safety / final straight entry
    # ========================================================

    def watchdog_callback(self):

        if not self.is_docking_active():
            return

        now = time.monotonic()

        if (
            now
            - self.last_image_time
            > self.image_watchdog_timeout_s
        ):
            self.finish_docking(
                success=False,
                message="CAMERA_WATCHDOG_TIMEOUT",
                state="FAILED",
            )

            self.get_logger().error(
                "Camera watchdog timeout during docking -> STOP"
            )


    def start_final_entry(self):

        if self.final_entry_complete:
            return

        self.final_entry_active = True
        self.final_entry_start_time = (
            time.monotonic()
        )
        self.update_docking_feedback(
            "FINAL_ENTRY"
        )

        duration = (
            self.final_entry_distance_m
            / max(
                self.final_entry_speed_mps,
                1e-6,
            )
        )

        self.get_logger().info(
            "FINAL_ENTRY START | "
            f"distance={self.final_entry_distance_m:.2f} m | "
            f"speed={self.final_entry_speed_mps:.2f} m/s | "
            f"duration={duration:.1f} s"
        )


    def run_final_entry(
        self,
        now,
    ):

        if self.final_entry_start_time is None:
            self.start_final_entry()

        elapsed = (
            now
            - self.final_entry_start_time
        )

        duration = (
            self.final_entry_distance_m
            / max(
                self.final_entry_speed_mps,
                1e-6,
            )
        )

        if elapsed >= duration:

            self.final_entry_active = False
            self.final_entry_complete = True

            self.publish_twist(
                0.0,
                0.0,
            )

            self.get_logger().info(
                "DOCKING COMPLETE | "
                "final straight entry finished -> STOP"
            )

            self.finish_docking(
                success=True,
                message="DOCKING_COMPLETE",
                state="DOCKING_COMPLETE",
            )

            return

        self.publish_twist(
            self.final_entry_speed_mps,
            0.0,
        )

        if (
            now
            - self.last_log_time
            > 0.5
        ):

            remaining = max(
                0.0,
                self.final_entry_distance_m
                - (
                    self.final_entry_speed_mps
                    * elapsed
                ),
            )

            self.get_logger().info(
                "FINAL_ENTRY | "
                f"elapsed={elapsed:.1f}s | "
                f"remaining≈{remaining:.2f} m | "
                f"v={self.final_entry_speed_mps:.3f} | "
                "w=+0.000"
            )

            self.last_log_time = now


    # ========================================================
    # Motion
    # ========================================================

    def publish_twist(
        self,
        linear_x,
        angular_z,
    ):

        if not self.publish_cmd_vel:
            return

        is_nonzero = (
            abs(float(linear_x)) > 1e-9
            or abs(float(angular_z)) > 1e-9
        )

        if is_nonzero and not self.is_docking_active():
            return

        msg = Twist()

        msg.linear.x = float(
            linear_x
        )

        msg.angular.z = float(
            angular_z
        )

        self.cmd_pub.publish(
            msg
        )


    def stop_robot(self):

        self.publish_twist(
            0.0,
            0.0,
        )


    def compute_control(
        self,
        distance_m,
        lateral_m,
        yaw_deg,
    ):

        distance_error = (
            distance_m
            - self.handoff_distance_m
        )

        # Dolly is to the right when lateral > 0.
        # A differential-drive AMR must initially steer right,
        # so the lateral steering term has a negative sign.
        bearing_rad = math.atan2(
            lateral_m,
            max(
                distance_m,
                0.1,
            ),
        )

        yaw_rad = math.radians(
            yaw_deg
        )

        angular_z = (
            self.k_yaw
            * yaw_rad
            -
            self.k_lateral
            * bearing_rad
        )

        angular_z *= (
            self.angular_sign
        )

        angular_z = clamp(
            angular_z,
            -self.max_angular_rps,
            self.max_angular_rps,
        )

        aligned = (
            abs(lateral_m)
            <= self.done_lateral_m
            and
            abs(yaw_deg)
            <= self.done_yaw_deg
        )

        at_handoff = (
            distance_m
            <= self.handoff_distance_m
        )

        if (
            aligned
            and
            at_handoff
        ):

            return (
                "HANDOFF_READY",
                0.0,
                0.0,
            )

        # Large-error alignment:
        # Do NOT mix lateral and yaw terms here because they can
        # cancel each other (e.g. lat=+0.21 m, yaw=+7 deg),
        # leaving an almost-zero angular command and stalling.
        #
        # If lateral is outside the drive corridor, prioritize
        # lateral centering with a guaranteed minimum turn rate.
        if (
            abs(lateral_m)
            > self.drive_lateral_limit_m
        ):

            lateral_turn = (
                self.k_lateral
                * abs(
                    bearing_rad
                )
            )

            lateral_turn = max(
                self.min_align_angular_rps,
                lateral_turn,
            )

            lateral_turn = min(
                self.max_angular_rps,
                lateral_turn,
            )

            angular_z = (
                -math.copysign(
                    lateral_turn,
                    lateral_m,
                )
                * self.angular_sign
            )

            return (
                "ALIGN_LATERAL",
                0.0,
                angular_z,
            )

        # If lateral is already acceptable but yaw is too large,
        # correct yaw in place with a minimum turn rate.
        if (
            abs(yaw_deg)
            > self.drive_yaw_limit_deg
        ):

            yaw_turn = (
                self.k_yaw
                * abs(
                    yaw_rad
                )
            )

            yaw_turn = max(
                self.min_align_angular_rps,
                yaw_turn,
            )

            yaw_turn = min(
                self.max_angular_rps,
                yaw_turn,
            )

            angular_z = (
                math.copysign(
                    yaw_turn,
                    yaw_deg,
                )
                * self.angular_sign
            )

            return (
                "ALIGN_YAW",
                0.0,
                angular_z,
            )

        linear_x = (
            self.k_distance
            * max(
                0.0,
                distance_error,
            )
        )

        linear_x = clamp(
            linear_x,
            self.min_linear_mps,
            self.max_linear_mps,
        )

        # Never drive forward after crossing the current
        # vision-model handoff distance.
        if distance_error <= 0.0:
            linear_x = 0.0

        return (
            "APPROACH",
            linear_x,
            angular_z,
        )

    # ========================================================
    # YOLO + PnP
    # ========================================================

    def estimate_pose(
        self,
        image,
    ):

        result = self.model.predict(
            source=image,
            imgsz=640,
            conf=self.detection_conf,
            device=0,
            verbose=False,
        )[0]

        if (
            result.boxes is None
            or
            len(result.boxes) == 0
            or
            result.keypoints is None
        ):
            return None

        box_conf = (
            result.boxes.conf
            .detach()
            .cpu()
            .numpy()
        )

        best_index = int(
            np.argmax(
                box_conf
            )
        )

        xy = (
            result.keypoints.xy[
                best_index
            ]
            .detach()
            .cpu()
            .numpy()
        )

        kp_conf_tensor = (
            result.keypoints.conf
        )

        if kp_conf_tensor is None:

            kp_conf = np.ones(
                len(xy),
                dtype=np.float32,
            )

        else:

            kp_conf = (
                kp_conf_tensor[
                    best_index
                ]
                .detach()
                .cpu()
                .numpy()
            )

        keep = np.where(
            kp_conf
            >= self.keypoint_conf
        )[0]

        if len(keep) < 4:

            keep = np.argsort(
                kp_conf
            )[-4:]

        object_points = (
            self.object_points_all[
                keep
            ]
        )

        image_points = (
            xy[
                keep
            ]
            .astype(
                np.float64
            )
        )

        ok, rvec, tvec, inliers = (
            cv2.solvePnPRansac(
                objectPoints=object_points,
                imagePoints=image_points,
                cameraMatrix=self.K,
                distCoeffs=self.dist_coeffs,
                iterationsCount=200,
                reprojectionError=(
                    self.pnp_reprojection_error_px
                ),
                confidence=0.99,
                flags=cv2.SOLVEPNP_EPNP,
            )
        )

        if not ok:
            return None

        if (
            inliers is not None
            and
            len(inliers) >= 4
        ):

            idx = (
                inliers
                .reshape(-1)
            )

            cv2.solvePnP(
                objectPoints=(
                    object_points[
                        idx
                    ]
                ),
                imagePoints=(
                    image_points[
                        idx
                    ]
                ),
                cameraMatrix=self.K,
                distCoeffs=self.dist_coeffs,
                rvec=rvec,
                tvec=tvec,
                useExtrinsicGuess=True,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )

        R, _ = cv2.Rodrigues(
            rvec
        )

        tvec = (
            tvec
            .reshape(3)
        )

        distance_m = float(
            tvec[2]
        )

        lateral_m = float(
            tvec[0]
        )

        # Dolly local +X axis in OpenCV camera frame.
        dolly_axis_camera = (
            R[:, 0]
        )

        yaw_raw_deg = math.degrees(
            math.atan2(
                float(
                    dolly_axis_camera[0]
                ),
                float(
                    dolly_axis_camera[2]
                ),
            )
        )

        yaw_deg = wrap_deg(
            self.yaw_sign
            * yaw_raw_deg
        )

        return {
            "distance_m": distance_m,
            "lateral_m": lateral_m,
            "yaw_deg": yaw_deg,
            "bbox_conf": float(
                box_conf[
                    best_index
                ]
            ),
            "used_keypoints": int(
                len(keep)
            ),
            "inliers": (
                0
                if inliers is None
                else int(
                    len(inliers)
                )
            ),
        }

    # ========================================================
    # ROS callback
    # ========================================================

    def image_callback(
        self,
        msg,
    ):

        now = time.monotonic()

        # The camera may run continuously, but only an accepted Action Goal
        # grants this node ownership of non-zero /cmd_vel commands.
        if not self.is_docking_active():
            return

        self.last_image_time = now

        # Once final straight entry begins, stop using vision.
        # Camera frames are used only as a heartbeat so the
        # watchdog can fail-safe stop if the stream disappears.
        if self.final_entry_complete:

            self.publish_twist(
                0.0,
                0.0,
            )
            return

        if self.final_entry_active:

            self.run_final_entry(
                now
            )
            return

        if (
            now
            - self.last_process_time
            < self.min_process_period
        ):
            return

        self.last_process_time = now

        try:

            image = (
                self.bridge
                .imgmsg_to_cv2(
                    msg,
                    desired_encoding="bgr8",
                )
            )

            pose = self.estimate_pose(
                image
            )

            if pose is None:

                self.invalid_pose_count += 1
                self.update_docking_feedback(
                    "WAITING_FOR_VISION"
                )

                # Expected close-range behavior:
                # keypoints may leave the FOV before 2.0 m.
                # If the last valid pose was already sufficiently
                # close and aligned, transition immediately to
                # the open-loop final straight entry.
                if (
                    self.enable_final_entry
                    and
                    self.last_valid_pose is not None
                    and
                    self.invalid_pose_count
                    >= self.invalid_frames_before_handoff
                    and
                    self.last_valid_pose["distance_m"]
                    <= self.fallback_distance_m
                    and
                    abs(
                        self.last_valid_pose["lateral_m"]
                    )
                    <= self.fallback_lateral_m
                    and
                    abs(
                        self.last_valid_pose["yaw_deg"]
                    )
                    <= self.fallback_yaw_deg
                ):

                    self.get_logger().warn(
                        "VISION HANDOFF FALLBACK | "
                        f"last d={self.last_valid_pose['distance_m']:.3f} m | "
                        f"lat={self.last_valid_pose['lateral_m']:+.3f} m | "
                        f"yaw={self.last_valid_pose['yaw_deg']:+.2f} deg"
                    )

                    self.start_final_entry()
                    self.run_final_entry(
                        now
                    )
                    return

                self.stop_robot()

                if (
                    now
                    - self.last_log_time
                    > 1.0
                ):

                    self.get_logger().warn(
                        "Vision/PnP invalid -> STOP"
                    )

                    self.last_log_time = now

                return

            self.invalid_pose_count = 0
            self.last_valid_pose = dict(
                pose
            )

            state, linear_x, angular_z = (
                self.compute_control(
                    distance_m=(
                        pose["distance_m"]
                    ),
                    lateral_m=(
                        pose["lateral_m"]
                    ),
                    yaw_deg=(
                        pose["yaw_deg"]
                    ),
                )
            )

            self.update_docking_feedback(
                state,
                pose,
            )

            if (
                state == "HANDOFF_READY"
                and
                self.enable_final_entry
            ):

                self.start_final_entry()

                self.run_final_entry(
                    now
                )

                return

            if state == "HANDOFF_READY":
                self.finish_docking(
                    success=True,
                    message="DOCKING_COMPLETE",
                    state="DOCKING_COMPLETE",
                )
                return

            self.publish_twist(
                linear_x,
                angular_z,
            )

            if (
                now
                - self.last_log_time
                > 0.5
            ):

                self.get_logger().info(
                    f"{state} | "
                    f"d={pose['distance_m']:.3f} m | "
                    f"lat={pose['lateral_m']:+.3f} m | "
                    f"yaw={pose['yaw_deg']:+.2f} deg | "
                    f"v={linear_x:.3f} | "
                    f"w={angular_z:+.3f} | "
                    f"kp={pose['used_keypoints']} | "
                    f"inliers={pose['inliers']} | "
                    f"conf={pose['bbox_conf']:.2f}"
                )

                self.last_log_time = now

        except Exception as exc:

            self.finish_docking(
                success=False,
                message=(
                    "VISION_RUNTIME_ERROR: "
                    f"{type(exc).__name__}: {exc}"
                ),
                state="FAILED",
            )

            self.get_logger().error(
                f"Runtime docking error: "
                f"{type(exc).__name__}: {exc}"
            )


# ============================================================
# Entry point
# ============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = DollyDockingNode()

    executor = MultiThreadedExecutor(
        num_threads=4
    )
    executor.add_node(node)

    try:

        executor.spin()

    except KeyboardInterrupt:

        pass

    finally:

        # In ROS 2 Jazzy, Ctrl+C may already shut down the
        # default rclpy context before execution reaches here.
        # Only publish the final stop and call shutdown while
        # the context is still valid.
        if rclpy.ok():

            try:
                node.stop_robot()
            except Exception:
                pass

        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()
