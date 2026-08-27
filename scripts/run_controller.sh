#!/usr/bin/env bash
# One AMR mission controller.
#
#   scripts/run_controller.sh amr1 --vision
#   scripts/run_controller.sh amr2
#
# --vision hands the first mission's docking approach to the camera. Only one
# robot uses it: the bridge builds a single docking camera, so a second
# subscriber would be measuring the first robot's view.
# No `set -u`: /opt/ros/jazzy/setup.bash reads unset variables by
# design and aborts under it. run_bridge.sh carries the same note.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/scripts/rosenv.sh"
source "$REPO/ros2_ws/install/setup.bash"

AMR="${1:?usage: run_controller.sh <amr1|amr2|amr3> [--vision]}"
shift || true

ARGS=(-p "amr:=$AMR")
case "${1:-}" in
    --vision)
        # First mission only.
        ARGS+=(-p vision_dock_mission:=0)
        ARGS+=(-p "snapshot_standoff_m:=${SNAPSHOT_STANDOFF_M:-4.6}")
        ;;
    --vision-all)
        # Every mission, so one run yields one snapshot per dock rather than
        # one in total. A single-sample detection rate says nothing.
        ARGS+=(-p vision_dock_all_missions:=true)
        ARGS+=(-p "snapshot_standoff_m:=${SNAPSHOT_STANDOFF_M:-4.6}")
        ;;
esac

exec ros2 run amr_control amr_mission_controller --ros-args "${ARGS[@]}"
