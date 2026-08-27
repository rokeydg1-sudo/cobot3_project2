#!/usr/bin/env bash
# View a published image topic. Defaults to the vision overlay.
# No `set -u`: /opt/ros/jazzy/setup.bash reads unset variables by
# design and aborts under it. run_bridge.sh carries the same note.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/rosenv.sh"
exec ros2 run rqt_image_view rqt_image_view \
    "${1:-/vision/dolly_docking/debug_image}"
