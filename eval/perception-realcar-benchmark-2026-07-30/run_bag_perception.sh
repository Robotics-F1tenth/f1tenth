#!/bin/bash
# run_bag_perception.sh <bag_path> <out_prefix>
# Launches perception fresh (consumes the bag's own baked /tf + /map, no live
# localization needed) + record_ros_topics.py, replays the FULL bag (no --topics
# filter -- perception needs /tf /map /scan /odom, eval_logger needs
# /f1/perception/object/obstacles/arr /mocap_pose1 /mocap_pose2).
set -e
L="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=42
BAG=$1; PFX=$2

# The occupancy map and lane CSV are separate assets. Keep them matched for
# the 0730 sessions; otherwise the CSV corridor fallback uses the wrong track.
case "$(basename "$BAG")" in
  *22_48_25|*22_52_38|*22_55_09) LANE=map1_path ;;
  *)                              LANE=map7_path ;;
esac
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "record_ros_topics.py" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 1

ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
  csv_path:="$LANE" \
  > "$L/${PFX}_perception.log" 2>&1 &
PERCPID=$!
sleep 3

rm -f "$L"/${PFX}_*.csv "$L"/${PFX}_static_obstacles.txt
cd /home/a/f1tenth
python3 eval/tools/record_ros_topics.py --out "$L/${PFX}" --ros-args -p use_sim_time:=true \
  > "$L/${PFX}_logger.log" 2>&1 &
LPID=$!
ros2 topic echo /f1/perception/static_obstacles > "$L/${PFX}_static_obstacles.txt" 2>&1 &
STATPID=$!
sleep 2

ros2 bag play "$BAG" --clock 200 > "$L/${PFX}_bag.log" 2>&1
sleep 3

kill -INT $LPID 2>/dev/null || true; sleep 2; kill -9 $LPID 2>/dev/null || true
kill -9 $STATPID 2>/dev/null || true
kill -9 $PERCPID 2>/dev/null || true
pkill -9 -f "perception_node" 2>/dev/null || true
echo "[$PFX] rows: tracks=$(wc -l < "$L/${PFX}_tracks.csv" 2>/dev/null || echo 0) gt_opp=$(wc -l < "$L/${PFX}_gt_opp.csv" 2>/dev/null || echo 0) static_msgs=$(grep -c '^f1_state_arr:' "$L/${PFX}_static_obstacles.txt" 2>/dev/null || echo 0)"
