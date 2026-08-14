#!/bin/bash
set -e
LE="$(cd "$(dirname "$0")" && pwd)"
BAG=/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38
NAME=$1; RETRO=$2   # NAME=출력prefix, RETRO=true|false
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

rm -f "$LE/${NAME}"_*.csv
RETRO=$RETRO ros2 launch "$LE/ekf_replay_launch.py" > "$LE/${NAME}_console.log" 2>&1 &
EKFPID=$!
sleep 3

python3 "$LE/retro_logger.py" --out "$LE/$NAME" --ros-args -p use_sim_time:=true > "$LE/${NAME}_logger.log" 2>&1 &
LPID=$!
sleep 2

echo "[run_one] $NAME (retro=$RETRO) bag play..."
ros2 bag play "$BAG" --clock 200 --topics /odom /ekf/imu/data /newmcl_pose /mocap_pose1 > "$LE/${NAME}_bag.log" 2>&1
sleep 3

kill -INT $LPID 2>/dev/null || true; sleep 2
kill -9 $LPID $EKFPID 2>/dev/null || true
pkill -9 -f "ekf_pose_estimator" 2>/dev/null || true
echo "[run_one] $NAME rows: mcl=$(wc -l < "$LE/${NAME}_mcl.csv" 2>/dev/null) gt=$(wc -l < "$LE/${NAME}_gt_ego.csv" 2>/dev/null)"
