#!/bin/bash
# Record S10 (SMPPI base vs barrier+grip). Launches racecar_simulator (with its own
# rviz) + cuda_mppi controller. Ctrl-C stops everything.
#
#   bash run_smppi_sim.sh base       # pre-fix params (b71dc0f^) -- expect corner departures
#   bash run_smppi_sim.sh after      # current params.yaml (barrier 2000 + curvature grip + margin)
#
# Runs on its own ROS_DOMAIN_ID so it can't mix with a live/other session.
# NOTE: no `set -u` -- ROS's setup.bash reads unset vars and dies silently under it.
VER=${1:-}
if [ "$VER" != "base" ] && [ "$VER" != "after" ]; then
    echo "usage: bash run_smppi_sim.sh <base|after>"
    exit 1
fi

WS=/home/a/f1tenth

# install/.../params.yaml is a plain copy (not symlinked), so editing it wouldn't
# reach the running node without a rebuild, and would need restoring after. Cleaner:
# cuda_mppi.launch.py exposes `param_file` as a launch argument (default = install
# share path) -- pass an override directly, nothing in the workspace is touched.
if [ "$VER" = "base" ]; then
    PARAM_FILE="$WS/eval/comparison-video-capture/params_base.yaml"
    echo "=== BASE (pre-fix params, b71dc0f^) -- expect corner departures ==="
else
    PARAM_FILE=""   # empty -> let the launch arg default to the current install yaml
    echo "=== AFTER (barrier 2000 + curvature grip + margin 0.15) -- current default ==="
fi

cleanup() {
    echo; echo "[cleanup] killing nodes"
    kill $SIM_PID $CTRL_PID 2>/dev/null
    pkill -9 -f "[r]acecar_simulator" 2>/dev/null
    pkill -9 -f "[s]mppi_node" 2>/dev/null
    pkill -9 -f "[p]ath_publisher" 2>/dev/null
    pkill -9 -f "[r]viz2" 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export ROS_DOMAIN_ID=61

echo "=== launching sim (has its own rviz) ==="
ros2 launch racecar_simulator simulator.launch.py &
SIM_PID=$!
sleep 5

echo "=== launching controller (map1, is_simulation=True hardcoded in launch) ==="
if [ -n "$PARAM_FILE" ]; then
    ros2 launch smppi_cuda_controller cuda_mppi.launch.py param_file:="$PARAM_FILE" &
else
    ros2 launch smppi_cuda_controller cuda_mppi.launch.py &
fi
CTRL_PID=$!

echo
echo "=== running ==="
echo "!!! KNOWN SIM BUG: centerPathCallback (simulator.cpp:917-936) spawns the car"
echo "    at the END of the centerline (poses.size()-3) but with the heading taken"
echo "    from the START (poses[1]) -- position and orientation don't correspond,"
echo "    so the car heads straight off the map on spawn regardless of base/after."
echo "    FIX: in rviz, click '2D Pose Estimate', click a point ON the track, drag"
echo "    to set heading along the track direction, release. car0RvizCallback sets"
echo "    car_state0_ from that click directly (position+heading match, no bug) --"
echo "    the car should track normally from that point on."
echo "=== Ctrl-C to stop and restore params. ==="
wait $CTRL_PID
