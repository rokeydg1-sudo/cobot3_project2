#!/usr/bin/env bash
# Fleet plan drawn on a map of the factory, plus a side-by-side comparison.
#
#   /planner/map           chosen plan, with live robot positions
#   /planner/map_compare   naive assignment beside cuOpt's
#
# No `set -u`: /opt/ros/jazzy/setup.bash reads unset variables by design.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/rosenv.sh"
exec "$REPO/.venv_vision/bin/python" "$REPO/scripts/planner_map.py"
