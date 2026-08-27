#!/usr/bin/env bash
# Snapshot vision node: recognises the Dolly and answers the docking request.
#
# Paths are derived from this script's own location so the repository can be
# cloned anywhere. Hardcoding them is what kept the previous launchers stuck in
# /tmp, outside version control and unavailable to anyone else.
# No `set -u`: /opt/ros/jazzy/setup.bash reads unset variables by
# design and aborts under it. run_bridge.sh carries the same note.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/rosenv.sh"

WIDTH="${CAMERA_WIDTH:-640}"
INTRINSICS="${CAMERA_INTRINSICS:-$REPO/simulation/isaac_sim/vision_docking/config/camera_intrinsics_${WIDTH}.npz}"

exec "$REPO/.venv_vision/bin/python" \
    "$REPO/simulation/isaac_sim/vision_docking/runtime/dolly_snapshot_node.py" \
    --ros-args -p "intrinsics:=$INTRINSICS"
