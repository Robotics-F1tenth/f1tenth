#!/bin/bash
# Test A: baked /newmcl_pose (as it actually ran live on the car that day) vs GT.
# Test B: fresh live MCL computed against the replayed bag (excludes baked /tf +
#         /newmcl_pose) vs GT -- checks whether the backward bias seen replaying
#         record5 also shows up replaying this new real-car bag.
set -e
L="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=42
BAG=/home/a/Downloads/0730/rosbag2_2026_07_30-22_25_24
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

pkill -9 -f "als_ros/mcl" 2>/dev/null || true
pkill -9 -f "record_ros_topics.py" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 1

echo "=== Test A: baked /newmcl_pose (real, recorded live on car) vs GT ==="
rm -f "$L"/A_*.csv
cd /home/a/f1tenth
python3 eval/tools/record_ros_topics.py --out "$L/A" --ros-args -p use_sim_time:=true \
  > "$L/A_logger.log" 2>&1 &
LPID=$!
sleep 2
ros2 bag play "$BAG" --clock 200 --topics /newmcl_pose /mocap_pose1 /mocap_pose2 \
  > "$L/A_bag.log" 2>&1
sleep 3
kill -INT $LPID 2>/dev/null || true; sleep 2; kill -9 $LPID 2>/dev/null || true
echo "[A] rows: mcl=$(wc -l < "$L/A_mcl.csv" 2>/dev/null || echo 0) gt=$(wc -l < "$L/A_gt_ego.csv" 2>/dev/null || echo 0)"

echo
echo "=== Test B: fresh live MCL replay (bag's own /map, no baked /tf) vs GT ==="
rm -f "$L"/B_*.csv
MCL_PARAMS="$L/mcl_params_0730.yaml" ros2 launch "$L/eval_mcl_launch_bagmap.py" \
  > "$L/B_mcl_console.log" 2>&1 &
MCLPID=$!
sleep 5
python3 eval/tools/record_ros_topics.py --out "$L/B" --ros-args -p use_sim_time:=true \
  > "$L/B_logger.log" 2>&1 &
LPID=$!
sleep 2
ros2 bag play "$BAG" --clock 200 --topics /scan /odom /ekf/imu/data /map /mocap_pose1 /mocap_pose2 \
  > "$L/B_bag.log" 2>&1
sleep 5
kill -INT $LPID 2>/dev/null || true; sleep 2; kill -9 $LPID $MCLPID 2>/dev/null || true
pkill -9 -f "als_ros/mcl" 2>/dev/null || true
pkill -9 -f "install/als_ros" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
echo "[B] rows: mcl=$(wc -l < "$L/B_mcl.csv" 2>/dev/null || echo 0) gt=$(wc -l < "$L/B_gt_ego.csv" 2>/dev/null || echo 0)"

echo
echo "########## Test A analysis (baked, real-time) ##########"
python3 eval/tools/analyze_localization.py "$L/A_mcl.csv" "$L/A_gt_ego.csv" --lane src/perception/data/lane/map7_path.csv || true
echo
echo "########## Test B analysis (fresh replay-computed) ##########"
python3 eval/tools/analyze_localization.py "$L/B_mcl.csv" "$L/B_gt_ego.csv" --lane src/perception/data/lane/map7_path.csv || true
