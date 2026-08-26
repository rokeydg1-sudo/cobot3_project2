#!/usr/bin/env bash
set -e

PROJECT_ROOT="${HOME}/cobot3_projects/cobot3_project2"

source /opt/ros/jazzy/setup.bash
source "${PROJECT_ROOT}/vision/.venv/bin/activate"

echo "===== RUNTIME ENV CHECK ====="

python - <<'PY'
import sys
print("python      :", sys.executable)

import torch
print("torch       :", torch.__version__)
print("cuda        :", torch.cuda.is_available())
if torch.cuda.is_available():
    print("gpu         :", torch.cuda.get_device_name(0))

import ultralytics
print("ultralytics :", ultralytics.__version__)

import cv2
print("opencv      :", cv2.__version__)

import rclpy
print("rclpy       : OK")

from cv_bridge import CvBridge
print("cv_bridge   : OK")

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
print("ROS msgs    : OK")
PY
