#!/bin/bash
# 0820 car4 high-speed bag: live MCL (+ optional EKF) vs mocap GT.
# usage: run_0820.sh <params.yaml> <prefix> <trials> [withekf]
set -o pipefail
L="/tmp/claude-1000/-home-a-f1tenth/3fda53a2-f142-4ee1-ba13-00bfbdc2ff66/scratchpad"
LAUNCH="/home/a/f1tenth/eval/localization-bag-replay/0730/eval_mcl_launch_bagmap.py"
BAG="/home/a/Downloads/f1tenth_bag/0820/car4_manual_loc_perception_20260820_002846"
EKF_PARAMS="/home/a/f1tenth/install/ekf_pose/share/ekf_pose/config/ekf_pose.yaml"
PARAMS="$1"; PFX="$2"; TRIALS="${3:-3}"; WITHEKF="${4:-}"
export ROS_DOMAIN_ID=70
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

for t in $(seq 1 "$TRIALS"); do
  NAME="${PFX}_t${t}"
  rm -f "$L"/${NAME}_*.csv "$L"/${NAME}_*.log

  MCL_PARAMS="$PARAMS" ros2 launch "$LAUNCH" > "$L/${NAME}_mcl.log" 2>&1 &
  sleep 3
  EKF_ARG=""
  if [ -n "$WITHEKF" ]; then
    ros2 run ekf_pose ekf_pose_estimator --ros-args --params-file "$EKF_PARAMS" \
      -p use_sim_time:=true -p imu_topic:=/car4/imu/data -p odom_topic:=/car4/odom \
      -p mcl_pose_topic:=/newmcl_pose > "$L/${NAME}_ekfnode.log" 2>&1 &
    EKF_ARG="/ekf_pose"
    sleep 2
  fi
  python3 "$L/rec3.py" "$L/${NAME}" /newmcl_pose /mocap_cal/car4/base $EKF_ARG \
    > "$L/${NAME}_rec.log" 2>&1 &
  sleep 2

  timeout 90 ros2 bag play "$BAG" --clock 200 --disable-keyboard-controls \
    --topics /tf_static /car4/scan /car4/odom /car4/imu/data /map /mocap_cal/car4/base \
    < /dev/null > "$L/${NAME}_bag.log" 2>&1

  sleep 2
  for p in $(pgrep -f "rec3.py" 2>/dev/null); do kill -INT "$p" 2>/dev/null; done
  sleep 2
  for pat in "ekf_pose_estimator" "install/als_ros/lib/als_ros/mcl" "ros2 launch.*eval_mcl" "rec3.py"; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
  done
  sleep 1
  echo "[$NAME] mcl=$(wc -l < "$L/${NAME}_mcl.csv" 2>/dev/null) gt=$(wc -l < "$L/${NAME}_gt_ego.csv" 2>/dev/null) ekf=$(wc -l < "$L/${NAME}_ekf.csv" 2>/dev/null)"
done
echo "0820 run done"
