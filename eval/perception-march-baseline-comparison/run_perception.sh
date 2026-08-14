#!/bin/bash
# Run one perception version against one 0730 bag, with rviz, for side-by-side viewing/recording.
#
#   bash run_perception.sh current 22_48_25      # today's code (min_age 0.3 etc.)
#   bash run_perception.sh march   22_48_25      # March original (4b3f601), single-track, no filters
#
# Ctrl-C stops everything. Runs on its own ROS_DOMAIN_ID so it cannot mix with a live session.
# NOTE: no `set -u` here. ROS's own setup.bash reads unset variables
# (AMENT_TRACE_SETUP_FILES etc.), so -u kills the script at the source line
# with no output at all. Args are checked explicitly below instead.

VER=${1:-}
BAGKEY=${2:-}
if [ -z "$VER" ] || [ -z "$BAGKEY" ]; then
    echo "usage: bash run_perception.sh <current|march> <bag_key>"
    echo "  bag_key e.g. 22_48_25  22_52_38  22_55_09  22_25_24  22_35_37"
    exit 1
fi

case "$VER" in
    current) WS=/home/a/f1tenth ;;
    march)   WS=/home/a/f1tenth_march ;;
    *) echo "first arg must be 'current' or 'march'"; exit 1 ;;
esac

BAG=$(ls -d /home/a/Downloads/0730/rosbag2_2026_07_30-${BAGKEY} 2>/dev/null)
[ -z "$BAG" ] && { echo "bag not found: $BAGKEY"; exit 1; }

# The centerline csv must match the track in the bag; it is track config, not tuning.
case "$BAGKEY" in
    22_48_25|22_52_38|22_55_09) LANE=map1_path ;;
    *)                          LANE=map7_path ;;
esac

YAML=$WS/src/perception/config/perception/f1tenth.yaml
RVIZ=$WS/src/perception/config/perception/f1test.rviz
BACKUP=$(mktemp)
cp "$YAML" "$BACKUP"
# ponytail: sed over the one line beats a yaml round-trip that would reflow every comment
sed -i "s|^\( *csv_path: \).*|\1\"$LANE\"|" "$YAML"

cleanup() {
    echo
    echo "[cleanup] restoring yaml and killing nodes"
    cp "$BACKUP" "$YAML"; rm -f "$BACKUP"
    kill $LAUNCH_PID $RVIZ_PID 2>/dev/null
    # bracket trick: keeps this script's own argv from matching its own pkill
    pkill -9 -f "[p]erception_node" 2>/dev/null
    pkill -9 -f "[r]viz2" 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export ROS_DOMAIN_ID=57   # isolated from any live session

echo "=== version : $VER  ($WS)"
echo "=== bag     : $BAGKEY   lane: $LANE"
echo "=== domain  : $ROS_DOMAIN_ID"
echo

# The March launch declares no arguments (use_sim_time is hardcoded True inside it),
# so passing key:=value there is an error -- only the current launch takes them.
if [ "$VER" = "march" ]; then
    ros2 launch f1_perception f1_perception.launch.py &
else
    ros2 launch f1_perception f1_perception.launch.py \
        use_sim_time:=true use_rviz:=false csv_path:="$LANE" &
fi
LAUNCH_PID=$!
sleep 4

rviz2 -d "$RVIZ" --ros-args -p use_sim_time:=true &
RVIZ_PID=$!
sleep 3

echo "=== playing bag (Ctrl-C to stop) ==="
ros2 bag play "$BAG" --clock 200

echo "=== bag finished; leaving rviz up. Ctrl-C to quit. ==="
wait $RVIZ_PID
