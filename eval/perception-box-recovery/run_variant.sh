#!/bin/bash
# run_variant.sh <bag_key> <out_prefix> <wall_recovery:true|false>
# Box-recall / off-ROI-FP validation run. size_threshold stays at the yaml default
# (0.0 = legacy cap disabled) for both variants -- only wall_recovery_enabled toggles,
# to isolate its marginal effect on top of the already-decided cap removal.
# NOTE: no `set -u` -- ROS's setup.bash reads unset vars and dies silently under it.
M="$(cd "$(dirname "$0")" && pwd)"
BAGKEY=${1:-}; PFX=${2:-}; RECOVERY=${3:-}
if [ -z "$BAGKEY" ] || [ -z "$PFX" ] || [ -z "$RECOVERY" ]; then
    echo "usage: bash run_variant.sh <bag_key> <out_prefix> <true|false>"
    exit 1
fi
BAG=$(ls -d /home/a/Downloads/0730/rosbag2_2026_07_30-${BAGKEY} 2>/dev/null)
[ -z "$BAG" ] && { echo "bag not found: $BAGKEY"; exit 1; }
case "$BAGKEY" in
    22_48_25|22_52_38|22_55_09) LANE=map1_path ;;
    *)                          LANE=map7_path ;;
esac

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=71

pkill -9 -f "[p]erception_node" 2>/dev/null
pkill -9 -f "[e]val_logger.py" 2>/dev/null
pkill -9 -f "[r]os2 bag play" 2>/dev/null
sleep 1

ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    csv_path:="$LANE" wall_recovery_enabled:="$RECOVERY" \
    > "$M/${PFX}_perception.log" 2>&1 &
PERCPID=$!
sleep 4

rm -f "$M"/${PFX}_*.csv "$M"/${PFX}_static_obstacles.txt
python3 /home/a/f1tenth/eval/tools/record_ros_topics.py --out "$M/${PFX}" --ros-args -p use_sim_time:=true \
    > "$M/${PFX}_logger.log" 2>&1 &
LPID=$!
ros2 topic echo /f1/perception/static_obstacles > "$M/${PFX}_static_obstacles.txt" 2>&1 &
STATPID=$!
sleep 2

ros2 bag play "$BAG" --clock 200 > "$M/${PFX}_bag.log" 2>&1
sleep 3

kill -INT $LPID 2>/dev/null; sleep 2; kill -9 $LPID 2>/dev/null
kill -9 $STATPID 2>/dev/null
kill -9 $PERCPID 2>/dev/null
pkill -9 -f "[p]erception_node" 2>/dev/null

TR=$(wc -l < "$M/${PFX}_tracks.csv" 2>/dev/null || echo 0)
echo "[$PFX] lane=$LANE recovery=$RECOVERY tracks=$TR static_msgs=$(grep -c '^f1_state_arr:' "$M/${PFX}_static_obstacles.txt" 2>/dev/null || echo 0)"
