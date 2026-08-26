#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$PROJECT_ROOT/ros2_ws"
LOG_DIR="$WORKSPACE/log/gpu_free_tests"
mkdir -p "$LOG_DIR"

PASS_COUNT=0
WARN_COUNT=0
SKIP_COUNT=0
FAIL_COUNT=0

pass_result() {
    PASS_COUNT=$((PASS_COUNT + 1))
    echo "PASS: $1"
}

warn_result() {
    WARN_COUNT=$((WARN_COUNT + 1))
    echo "WARN: $1"
}

skip_result() {
    SKIP_COUNT=$((SKIP_COUNT + 1))
    echo "SKIP: $1"
}

fail_result() {
    FAIL_COUNT=$((FAIL_COUNT + 1))
    echo "FAIL: $1"
}

run_logged() {
    local label="$1"
    local log_name="$2"
    shift 2
    if "$@" >"$LOG_DIR/$log_name" 2>&1; then
        pass_result "$label"
        return 0
    fi
    fail_result "$label (log: $LOG_DIR/$log_name)"
    return 1
}

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    fail_result "ROS 2 Jazzy setup missing"
    exit 1
fi
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID="${COBOT3_ROS_DOMAIN_ID:-129}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || \
   (( ROS_DOMAIN_ID < 129 || ROS_DOMAIN_ID > 135 )); then
    fail_result "ROS_DOMAIN_ID must be 129-135"
    exit 1
fi

if [[ -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
    warn_result "root .venv detected; tests intentionally use ROS system Python"
else
    warn_result "root .venv absent; using /usr/bin/python3 for GPU-free tests"
fi

NAV_TEMP=""
if ros2 pkg prefix nav2_msgs >/dev/null 2>&1; then
    pass_result "system nav2_msgs available"
else
    warn_result "system nav2_msgs missing; preparing non-root /tmp overlay"
    NAV_TEMP="$(mktemp -d /tmp/cobot3_nav2_msgs.XXXXXX)"
    if (
        cd "$NAV_TEMP" || exit 1
        apt-get download \
            ros-jazzy-geographic-msgs \
            ros-jazzy-nav2-msgs
    ) >"$LOG_DIR/nav2_msgs_download.log" 2>&1; then
        for NAV_DEB in "$NAV_TEMP"/*.deb; do
            dpkg-deb -x "$NAV_DEB" "$NAV_TEMP/overlay"
        done
        source "$NAV_TEMP/overlay/opt/ros/jazzy/share/geographic_msgs/local_setup.bash"
        source "$NAV_TEMP/overlay/opt/ros/jazzy/share/nav2_msgs/local_setup.bash"
        pass_result "temporary nav2_msgs overlay"
    else
        fail_result "nav2_msgs unavailable and temporary download failed"
    fi
fi

if ros2 interface show nav2_msgs/action/NavigateThroughPoses \
    >"$LOG_DIR/navigate_through_poses.interface" 2>&1; then
    pass_result "NavigateThroughPoses interface"
else
    fail_result "NavigateThroughPoses interface"
fi

cd "$WORKSPACE" || exit 1
if colcon build --symlink-install --packages-select \
    interfaces amr_control fms mission_mock cobot3_bringup \
    >"$LOG_DIR/colcon_build.log" 2>&1; then
    pass_result "ROS package build"
    source "$WORKSPACE/install/setup.bash"
else
    fail_result "ROS package build (log: $LOG_DIR/colcon_build.log)"
fi

for INTERFACE_NAME in \
    interfaces/srv/RequestTask \
    interfaces/action/DockDolly \
    interfaces/action/LiftDolly; do
    SAFE_NAME="${INTERFACE_NAME//\//_}"
    if ros2 interface show "$INTERFACE_NAME" \
        >"$LOG_DIR/$SAFE_NAME.interface" 2>&1; then
        pass_result "$INTERFACE_NAME interface"
    else
        fail_result "$INTERFACE_NAME interface"
    fi
done

cd "$PROJECT_ROOT" || exit 1
run_logged \
    "Python syntax" \
    "python_syntax.log" \
    python3 -m py_compile \
    ros2_ws/src/fms/fms/fleet_management_system.py \
    ros2_ws/src/fms/fms/route_contract.py \
    ros2_ws/src/amr_control/amr_control/amr_node.py \
    ros2_ws/src/amr_control/amr_control/mission_utils.py \
    ros2_ws/src/mission_mock/mission_mock/mock_environment.py \
    ros2_ws/src/mission_mock/mission_mock/runner.py \
    ros2_ws/src/cobot3_bringup/launch/integration.launch.py

PYTHONPATH="ros2_ws/src/fms:${PYTHONPATH:-}" run_logged \
    "FMS two-route tests" \
    "fms_route_tests.log" \
    python3 -m pytest -q -rs ros2_ws/src/fms/test/test_route_contract.py

PYTHONPATH="ros2_ws/src/amr_control:${PYTHONPATH:-}" run_logged \
    "NavigateThroughPoses yaw/reverse tests" \
    "amr_mission_utils_tests.log" \
    python3 -m pytest -q ros2_ws/src/amr_control/test/test_mission_utils.py

for SCENARIO in success dock_failure lift_up_failure reverse_timeout; do
    if ros2 run mission_mock mission_mock_runner --scenario "$SCENARIO" \
        >"$LOG_DIR/mock_${SCENARIO}.log" 2>&1; then
        pass_result "mock mission $SCENARIO"
    else
        fail_result "mock mission $SCENARIO (log: $LOG_DIR/mock_${SCENARIO}.log)"
    fi
done

skip_result "cuOpt GPU solver runtime unavailable"
skip_result "Isaac Sim runtime unavailable"
skip_result "YOLO/Torch GPU runtime unavailable"

echo
echo "GPU-FREE TEST SUMMARY"
echo "PASS=$PASS_COUNT"
echo "WARN=$WARN_COUNT"
echo "SKIP=$SKIP_COUNT"
echo "FAIL=$FAIL_COUNT"
echo "LOG_DIR=$LOG_DIR"

if (( FAIL_COUNT > 0 )); then
    exit 1
fi
