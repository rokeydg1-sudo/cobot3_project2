#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INTERFACE_WS="$SCRIPT_DIR/ExtNodeMapBuild/ros2_interfaces"
INTERFACE_SETUP="$INTERFACE_WS/install/setup.bash"
ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-/home/rokey/Desktop/PRJT/IsaacSim_base/isaac-sim-standalone-6.0.1-linux-x86_64}"
ISAAC_SIM_EXECUTABLE="$ISAAC_SIM_DIR/isaac-sim.sh"

if [[ ! -d "$INTERFACE_WS/src/interfaces" ]]; then
    echo "ROS 2 interface package not found: $INTERFACE_WS/src/interfaces"
    exit 1
fi

if [[ ! -x "$ISAAC_SIM_EXECUTABLE" ]]; then
    echo "Isaac Sim executable not found: $ISAAC_SIM_EXECUTABLE"
    echo "Set ISAAC_SIM_DIR to the Isaac Sim installation directory."
    exit 1
fi

source /opt/ros/jazzy/setup.bash

echo "Building ExtNodeMapBuild ROS 2 interfaces..."
pushd "$INTERFACE_WS" > /dev/null
colcon build --packages-select interfaces
popd > /dev/null

source "$INTERFACE_SETUP"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "Launching Isaac Sim: $ISAAC_SIM_EXECUTABLE"

exec "$ISAAC_SIM_EXECUTABLE" "$@"
