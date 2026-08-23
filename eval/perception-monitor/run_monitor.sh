#!/bin/bash
# run_monitor.sh [--gt] [--bridge] [bag_dir]
#
# Attaches perception supervision to whatever is already running, or replays a bag.
#
#   ./run_monitor.sh                    # on the Jetson, next to a live perception
#   ./run_monitor.sh --bridge           # ... and start foxglove_bridge too
#   ./run_monitor.sh --gt <bag_dir>     # replay a two-car mocap bag with GT panels
#
# --gt starts live_gt_error.py as well. Leave it off on the car: without mocap it
# publishes nothing and the two [GT] panels simply stay empty.
set -u
M="$(cd "$(dirname "$0")" && pwd)"
GT=false; BRIDGE=false; BAG=""
for a in "$@"; do
    case "$a" in
        --gt) GT=true ;;
        --bridge) BRIDGE=true ;;
        *) BAG="$a" ;;
    esac
done
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
SIM=false; [ -n "$BAG" ] && SIM=true

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null; pkill -f "[r]os2 bag play" 2>/dev/null; }
trap cleanup EXIT INT TERM

python3 "$M/perception_monitor.py" --ros-args -p use_sim_time:=$SIM &
pids+=($!)
echo "perception_monitor -> /pmon/*"

if $GT; then
    python3 "$M/../opponent-benchmark-0821/live_gt_error.py" \
        --ros-args -p use_sim_time:=$SIM &
    pids+=($!)
    echo "live_gt_error -> /bench/*"
fi

if $BRIDGE; then
    if pgrep -f "[f]oxglove_bridge" > /dev/null; then
        echo "foxglove_bridge already running"
    else
        ros2 launch foxglove_bridge foxglove_bridge_launch.xml > /dev/null 2>&1 &
        pids+=($!)
        echo "foxglove_bridge started"
    fi
    echo "  connect Foxglove to ws://<jetson-ip>:8765"
fi

sleep 3
if [ -n "$BAG" ]; then
    echo "replaying $BAG"
    ros2 bag play "$BAG" --clock
    echo "bag finished; ctrl-c to stop"
fi
wait
