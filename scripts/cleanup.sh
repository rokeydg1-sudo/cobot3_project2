#!/usr/bin/env bash
# Kill every leftover demo process.
#
# Stale controllers or a second bridge republish /odom, and the robot then
# chases two conflicting pose streams and thrashes. Always run this first.
#
# The bracket in each pattern stops pkill from matching the shell that is
# running this very script, which otherwise kills the caller.

for pattern in 'standalone_factory_bridg[e]' 'amr_mission_controlle[r]' \
               'dolly_docking_nod[e]' 'rqt_image_vie[w]' 'watch\.p[y]'; do
    pkill -9 -f "$pattern" 2>/dev/null
done

sleep 3
echo "[cleanup] remaining:"
pgrep -af 'standalone_factory_bridg[e]|amr_mission_controlle[r]|dolly_docking_nod[e]' || echo "  none"
