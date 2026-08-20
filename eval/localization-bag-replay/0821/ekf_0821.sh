#!/bin/bash
# EKF on today's bags: replay + ekf_pose only (MCL comes baked from the car).
# usage: ekf_0821.sh <bagdir> <prefix> [rate] [extra ekf -p args...]
set -o pipefail
L="/tmp/claude-1000/-home-a-f1tenth/3fda53a2-f142-4ee1-ba13-00bfbdc2ff66/scratchpad"
EKF_PARAMS="/home/a/f1tenth/install/ekf_pose/share/ekf_pose/config/ekf_pose.yaml"
BAG="$1"; PFX="$2"; RATE="${3:-2.0}"; shift 3 2>/dev/null || shift $#
export ROS_DOMAIN_ID=72
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
rm -f "$L"/${PFX}_*.csv "$L"/${PFX}_*.log

ros2 run ekf_pose ekf_pose_estimator --ros-args --params-file "$EKF_PARAMS" \
  -p use_sim_time:=true "$@" > "$L/${PFX}_ekfnode.log" 2>&1 &
sleep 2
python3 "$L/rec3.py" "$L/${PFX}" /newmcl_pose /mocap/car4/pose /ekf_pose > "$L/${PFX}_rec.log" 2>&1 &
sleep 2

DUR=$(python3 -c "
import yaml;print(int(yaml.safe_load(open('$BAG/metadata.yaml'))['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9/$RATE)+30)")
timeout $DUR ros2 bag play "$BAG" --clock 200 -r "$RATE" --disable-keyboard-controls \
  --topics /tf_static /newmcl_pose /odom /ekf/imu/data /imu/data /mocap/car4/pose /map \
  < /dev/null > "$L/${PFX}_bag.log" 2>&1

sleep 3
for p in $(pgrep -f "rec3.py"); do kill -INT $p 2>/dev/null; done
sleep 2
for pat in "ekf_pose_estimator" "rec3.py"; do
  for p in $(pgrep -f "$pat"); do kill -9 $p 2>/dev/null; done
done
echo "[$PFX] mcl=$(wc -l < "$L/${PFX}_mcl.csv") ekf=$(wc -l < "$L/${PFX}_ekf.csv") gt=$(wc -l < "$L/${PFX}_gt_ego.csv")"
