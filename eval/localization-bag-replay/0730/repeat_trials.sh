#!/bin/bash
# Repeat the same config N times to separate genuine effects from the
# global-recovery lottery that a mismatched initial pose was causing.
# usage: repeat_trials.sh <params.yaml> <name-prefix> <trials>
set -o pipefail
L="/tmp/claude-1000/-home-a-f1tenth/3fda53a2-f142-4ee1-ba13-00bfbdc2ff66/scratchpad"
LAUNCH="/home/a/f1tenth/eval/localization-bag-replay/0730/eval_mcl_launch_bagmap.py"
BAG="/home/a/Downloads/f1tenth_bag/0730/rosbag2_2026_07_30-22_52_38"
PARAMS="$1"; PFX="$2"; TRIALS="${3:-3}"
export ROS_DOMAIN_ID=61
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

for t in $(seq 1 "$TRIALS"); do
  NAME="${PFX}_t${t}"
  rm -f "$L"/${NAME}_*.csv "$L"/${NAME}_*.log

  MCL_PARAMS="$PARAMS" ros2 launch "$LAUNCH" > "$L/${NAME}_mcl.log" 2>&1 &
  MCLPID=$!
  sleep 3
  python3 /home/a/f1tenth/eval/tools/record_ros_topics.py --out "$L/${NAME}" \
    --ros-args -p use_sim_time:=true > "$L/${NAME}_rec.log" 2>&1 &
  RECPID=$!
  sleep 2

  timeout 110 ros2 bag play "$BAG" --clock 200 --disable-keyboard-controls \
    --topics /tf_static /scan /odom /ekf/imu/data /map /mocap_pose1 /mocap_pose2 \
    < /dev/null > "$L/${NAME}_bag.log" 2>&1

  sleep 2
  kill -INT $RECPID 2>/dev/null; sleep 1
  kill -9 $MCLPID 2>/dev/null
  for p in $(pgrep -f "install/als_ros/lib/als_ros/mcl" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
  sleep 1
  echo "[$NAME] rows=$(wc -l < "$L/${NAME}_mcl.csv" 2>/dev/null)"
done
echo "trials done"
