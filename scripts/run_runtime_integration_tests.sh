#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${REPO_ROOT}/ros2_ws/log/runtime_integration"
LOG_FILE="${LOG_DIR}/runtime_health.log"
ISAAC_ROOT="${ISAACSIM_ROOT:-/home/rokey/isaacsim}"

mkdir -p "${LOG_DIR}"
: > "${LOG_FILE}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-129}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ "${1:-}" == "--full-mission" ]]; then
    echo "FAIL: full mission disabled: RUNTIME_BLOCKER_DOLLY_LIFT_CONTACT" | tee -a "${LOG_FILE}"
    exit 2
fi

pass_count=0
fail_count=0

pass() {
    echo "PASS: $*" | tee -a "${LOG_FILE}"
    pass_count=$((pass_count + 1))
}

fail() {
    echo "FAIL: $*" | tee -a "${LOG_FILE}"
    fail_count=$((fail_count + 1))
}

check_file() {
    if [[ -f "$1" ]]; then
        pass "$2=$1"
    else
        fail "$2 missing=$1"
    fi
}

check_topic() {
    local topic="$1"
    local expected_type="$2"
    local actual_type
    actual_type="$(timeout 5 ros2 topic type "${topic}" 2>>"${LOG_FILE}" || true)"
    if [[ "${actual_type}" == "${expected_type}" ]]; then
        pass "topic ${topic} [${actual_type}]"
    else
        fail "topic ${topic} expected=${expected_type} actual=${actual_type:-missing}"
    fi
}

check_action() {
    local action="$1"
    local expected_type="$2"
    if timeout 5 ros2 action list -t 2>>"${LOG_FILE}" \
        | grep -Fqx "${action} [${expected_type}]"; then
        pass "action ${action} [${expected_type}]"
    else
        fail "action ${action} [${expected_type}]"
    fi
}

check_file "${ISAAC_ROOT}/isaac-sim.sh" "Isaac launcher"
check_file "${ISAAC_ROOT}/python.sh" "Isaac Python"
check_file \
    "${REPO_ROOT}/simulation/isaac_sim/worlds/Collected_AF2_FLAT/AF2_FLAT.usd" \
    "Main USD"
check_file \
    "${REPO_ROOT}/simulation/isaac_sim/config/AF2_FLAT_integration.usda" \
    "Integration layer"

if [[ "${ROS_DOMAIN_ID}" =~ ^(129|130|131|132|133|134|135)$ ]]; then
    pass "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
else
    fail "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} outside 129..135"
fi

if [[ "${RMW_IMPLEMENTATION}" == "rmw_fastrtps_cpp" ]]; then
    pass "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
else
    fail "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
fi

source /opt/ros/jazzy/setup.bash
if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
    source "${REPO_ROOT}/ros2_ws/install/setup.bash"
fi
set -u

check_topic "/clock" "rosgraph_msgs/msg/Clock"
check_topic "/amr1/odom" "nav_msgs/msg/Odometry"
check_topic "/amr1/cmd_vel" "geometry_msgs/msg/Twist"
check_topic "/amr1/scan" "sensor_msgs/msg/LaserScan"
check_topic "/vision/front_camera/image_raw" "sensor_msgs/msg/Image"
check_topic "/vision/front_camera/camera_info" "sensor_msgs/msg/CameraInfo"
check_topic "/amr1/joint_states" "sensor_msgs/msg/JointState"
check_topic "/amr1/joint_commands" "sensor_msgs/msg/JointState"

check_action "/navigate_through_poses" "nav2_msgs/action/NavigateThroughPoses"
check_action "/dock_dolly" "interfaces/action/DockDolly"
check_action "/lift_dolly" "interfaces/action/LiftDolly"

if timeout 5 ros2 run tf2_ros tf2_echo map amr1/base_link -r 1 -p 3 \
    >>"${LOG_FILE}" 2>&1; then
    pass "TF map->amr1/base_link"
else
    if grep -q "Translation:" "${LOG_FILE}"; then
        pass "TF map->amr1/base_link"
    else
        fail "TF map->amr1/base_link"
    fi
fi

echo "RESULT: PASS=${pass_count} FAIL=${fail_count} LOG=${LOG_FILE}"
if ((fail_count > 0)); then
    exit 1
fi
