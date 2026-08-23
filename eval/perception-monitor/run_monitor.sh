#!/bin/bash
# run_monitor.sh [bag_dir]
# Brings up the perception supervision stack. With a bag argument it replays it;
# without one it just attaches to whatever is already running (live car).
#
#   ./run_monitor.sh                                  # live
#   ./run_monitor.sh /path/to/bag                     # replay
#
# Then open Foxglove -> ws://localhost:8765 -> Layout -> Import from file ->
# foxglove_perception_monitor.json
set -u
M="$(cd "$(dirname "$0")" && pwd)"
BAG="${1:-}"
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

SIM=false
[ -n "$BAG" ] && SIM=true

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null; pkill -f "[r]os2 bag play" 2>/dev/null; }
trap cleanup EXIT INT TERM

python3 "$M/perception_monitor.py" --ros-args -p use_sim_time:=$SIM &
pids+=($!)

# GT panels only work when the bag carries both cars in mocap; harmless otherwise
# (the node just never publishes, and those two panels stay empty).
python3 "$M/../opponent-benchmark-0821/live_gt_error.py" \
    --ros-args -p use_sim_time:=$SIM &
pids+=($!)

if ! pgrep -f "[f]oxglove_bridge" > /dev/null; then
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml > /dev/null 2>&1 &
    pids+=($!)
    echo "foxglove_bridge started -> ws://localhost:8765"
else
    echo "foxglove_bridge already running -> ws://localhost:8765"
fi

sleep 3
if [ -n "$BAG" ]; then
    echo "replaying $BAG"
    ros2 bag play "$BAG" --clock
    echo "bag finished; ctrl-c to stop the monitors"
fi
wait
