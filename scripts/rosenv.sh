# Shared ROS 2 environment for every process in the vision-docking demo.
#
# Scripts and non-interactive shells never read ~/.bashrc, so a bridge launched
# from a script once ended up on domain 30 with no Fast DDS whitelist while the
# operator's terminal sat on domain 130. The two could not see each other and
# the run looked like a bridge failure. Source this file everywhere instead of
# exporting by hand, so the bridge, the controllers and the vision node always
# agree.
#
#   source ~/cobot3_project2/scripts/rosenv.sh

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=130
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_whitelist.xml"

# Same as the isaac_ros helper in ~/.bashrc. Without it the Isaac Sim ROS 2
# bridge extension fails to load and no topic is ever published.
ISAAC_ROS_LIB="$HOME/isaacsim/exts/isaacsim.ros2.bridge/jazzy/lib"
case ":$LD_LIBRARY_PATH:" in
    *":$ISAAC_ROS_LIB:"*) ;;
    *) export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$ISAAC_ROS_LIB" ;;
esac

export DISPLAY="${DISPLAY:-:1}"

echo "[rosenv] domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION display=$DISPLAY"
