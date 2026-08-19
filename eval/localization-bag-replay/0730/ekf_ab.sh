#!/bin/bash
# A-3: EKF vs raw MCL, both against mocap GT, on the same live replay.
# Live MCL + ekf_pose run together, so both estimators see identical input.
# usage: ekf_ab.sh <name-prefix> <trials>
set -o pipefail
L="/tmp/claude-1000/-home-a-f1tenth/3fda53a2-f142-4ee1-ba13-00bfbdc2ff66/scratchpad"
LAUNCH="/home/a/f1tenth/eval/localization-bag-replay/0730/eval_mcl_launch_bagmap.py"
BAG="/home/a/Downloads/f1tenth_bag/0730/rosbag2_2026_07_30-22_52_38"
PARAMS="$L/params_2252_correct.yaml"
EKF_PARAMS="/home/a/f1tenth/install/ekf_pose/share/ekf_pose/config/ekf_pose.yaml"
PFX="$1"; TRIALS="${2:-3}"
export ROS_DOMAIN_ID=65
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

for t in $(seq 1 "$TRIALS"); do
  NAME="${PFX}_t${t}"
  rm -f "$L"/${NAME}_*.csv "$L"/${NAME}_*.log

  MCL_PARAMS="$PARAMS" ros2 launch "$LAUNCH" > "$L/${NAME}_mcl.log" 2>&1 &
  sleep 3
  ros2 run ekf_pose ekf_pose_estimator --ros-args --params-file "$EKF_PARAMS" \
    -p use_sim_time:=true > "$L/${NAME}_ekf.log" 2>&1 &
  sleep 2
  # record MCL(/newmcl_pose) and GT; ekf recorded separately below
  python3 /home/a/f1tenth/eval/tools/record_ros_topics.py --out "$L/${NAME}" \
    --ros-args -p use_sim_time:=true > "$L/${NAME}_rec.log" 2>&1 &
  python3 "$L/ekf_recorder.py" "$L/${NAME}_ekf.csv" > "$L/${NAME}_ekfrec.log" 2>&1 &
  sleep 2

  timeout 110 ros2 bag play "$BAG" --clock 200 --disable-keyboard-controls \
    --topics /tf_static /scan /odom /ekf/imu/data /map /mocap_pose1 /mocap_pose2 \
    < /dev/null > "$L/${NAME}_bag.log" 2>&1

  sleep 2
  for pat in "record_ros_topics" "ekf_recorder" ; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do kill -INT "$p" 2>/dev/null; done
  done
  sleep 2
  for pat in "ekf_pose_estimator" "install/als_ros/lib/als_ros/mcl" "ros2 launch.*eval_mcl" "record_ros_topics" "ekf_recorder"; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
  done
  sleep 1
  echo "[$NAME] mcl=$(wc -l < "$L/${NAME}_mcl.csv" 2>/dev/null) ekf=$(wc -l < "$L/${NAME}_ekf.csv" 2>/dev/null) gt=$(wc -l < "$L/${NAME}_gt_ego.csv" 2>/dev/null)"
done
echo "ekf ab done"
