#!/bin/bash
set -e
LE="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=42
BAG=/home/a/dataset_custom/F1tenth/paper/T_iv/record5/rosbag2_2026_03_13-21_52_51
NAME=$1; PARAMS=$2; EXTRA_WS=$3
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
[ -n "$EXTRA_WS" ] && source "$EXTRA_WS/install/setup.bash" 2>/dev/null
echo "[run_one] $NAME  params=$PARAMS  mcl_pkg=$(ros2 pkg prefix als_ros)"
rm -f "$LE/${NAME}"_*.csv
MCL_PARAMS=$PARAMS ros2 launch "$LE/eval_mcl_launch2.py" > "$LE/${NAME}_mcl_console.log" 2>&1 &
MCLPID=$!
sleep 5
cd /home/a/f1tenth
python3 eval/tools/record_ros_topics.py --out "$LE/${NAME}" --ros-args -p use_sim_time:=true > "$LE/${NAME}_logger.log" 2>&1 &
LPID=$!
sleep 2
ros2 bag play "$BAG" --clock 200 --topics /tf_static /scan /odom /ekf/imu/data /mocap_pose1 /mocap_pose2 > "$LE/${NAME}_bag.log" 2>&1
sleep 5
kill -INT $LPID 2>/dev/null || true; sleep 2
kill -9 $LPID $MCLPID 2>/dev/null || true
pkill -9 -f "als_ros/mcl" 2>/dev/null || true
pkill -9 -f "install/als_ros" 2>/dev/null || true
pkill -9 -f "map_server" 2>/dev/null || true
pkill -9 -f "lifecycle_manager" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
echo "[run_one] $NAME rows: mcl=$(wc -l < "$LE/${NAME}_mcl.csv") gt=$(wc -l < "$LE/${NAME}_gt_ego.csv")"
