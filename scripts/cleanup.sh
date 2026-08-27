#!/usr/bin/env bash
# Kill every leftover demo process.
#
# Stale controllers or a second bridge republish /odom, and the robot then
# chases two conflicting pose streams and thrashes. Always run this first.
#
# `pkill -f` matches against whole command lines, which includes the command
# line of whatever shell invoked this script. Typing any of these names in the
# same command was enough to make cleanup kill its own caller, so the process
# tree from this script up to init is collected first and excluded.

self_tree() {
    local pid=$$
    while [ "$pid" -gt 1 ]; do
        echo "$pid"
        pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
        [ -z "$pid" ] && break
    done
}

mapfile -t protected < <(self_tree)

is_protected() {
    local candidate=$1
    for pid in "${protected[@]}"; do
        [ "$candidate" = "$pid" ] && return 0
    done
    return 1
}

patterns='standalone_factory_bridge|amr_mission_controller|dolly_docking_node'
patterns="$patterns|rqt_image_view|capture_frames\.py|watch\.py"

killed=0
for pid in $(pgrep -f "$patterns"); do
    is_protected "$pid" && continue
    kill -9 "$pid" 2>/dev/null && killed=$((killed + 1))
done

sleep 3
echo "[cleanup] killed $killed process(es)"
remaining=$(pgrep -f "$patterns" | grep -vxF "$(self_tree | tr '\n' '|' | sed 's/|$//' | tr '|' '\n')" | wc -l)
echo "[cleanup] remaining: $remaining"
