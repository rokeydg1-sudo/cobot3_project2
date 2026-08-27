#!/usr/bin/env bash
# Launch the Isaac Sim bridge with the exact environment every other process
# uses. Logs to /tmp/bridge.log so the caller can poll for "BRIDGE RUNNING".
# No `set -u` here: /opt/ros/jazzy/setup.bash reads unset variables by design.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/rosenv.sh"

export HEADLESS="${HEADLESS:-0}"
export FLEET="${FLEET:-amr1,amr2}"
export TASK_IDS="${TASK_IDS:-T1,T2,T3,T4}"

cd "$REPO"
echo "[bridge] fleet=$FLEET tasks=$TASK_IDS headless=$HEADLESS"
exec "$HOME/isaacsim/python.sh" simulation/isaac_sim/standalone_factory_bridge.py
