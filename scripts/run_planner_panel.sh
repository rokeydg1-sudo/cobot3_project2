#!/usr/bin/env bash
# Fleet plan comparison panel, published as an image for rqt_image_view.
# No `set -u`: /opt/ros/jazzy/setup.bash reads unset variables by
# design and aborts under it. run_bridge.sh carries the same note.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/rosenv.sh"
exec "$REPO/.venv_vision/bin/python" "$REPO/scripts/planner_panel.py"
