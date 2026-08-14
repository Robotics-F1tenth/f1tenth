#!/bin/bash
# run_bag.sh <bag_path> <mcl_params.yaml> <lane_csv> <out_prefix>
# Test A: baked /newmcl_pose (as it actually ran live on the car) vs GT.
# Test B: fresh live MCL computed against the replayed bag (bag's own /map, no baked
#         /tf/newmcl_pose) vs GT -- checks whether any replay-induced bias shows up.
set -e
L="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=42
BAG=$1; PARAMS=$2; LANE=$3; PFX=$4
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

pkill -9 -f "als_ros/mcl" 2>/dev/null || true
pkill -9 -f "record_ros_topics.py" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 1

echo "=== [$PFX] Test A: baked /newmcl_pose (real, recorded live on car) vs GT ==="
rm -f "$L"/${PFX}_A_*.csv
cd /home/a/f1tenth
python3 eval/tools/record_ros_topics.py --out "$L/${PFX}_A" --ros-args -p use_sim_time:=true \
  > "$L/${PFX}_A_logger.log" 2>&1 &
LPID=$!
sleep 2
ros2 bag play "$BAG" --clock 200 --topics /tf_static /newmcl_pose /mocap_pose1 /mocap_pose2 \
  > "$L/${PFX}_A_bag.log" 2>&1
sleep 3
kill -INT $LPID 2>/dev/null || true; sleep 2; kill -9 $LPID 2>/dev/null || true
echo "[$PFX/A] rows: mcl=$(wc -l < "$L/${PFX}_A_mcl.csv" 2>/dev/null || echo 0) gt=$(wc -l < "$L/${PFX}_A_gt_ego.csv" 2>/dev/null || echo 0)"

echo
echo "=== [$PFX] Test B: fresh live MCL replay (bag's own /map, no baked /tf) vs GT ==="
rm -f "$L"/${PFX}_B_*.csv
MCL_PARAMS="$PARAMS" ros2 launch "$L/eval_mcl_launch_bagmap.py" \
  > "$L/${PFX}_B_mcl_console.log" 2>&1 &
MCLPID=$!
sleep 5
python3 eval/tools/record_ros_topics.py --out "$L/${PFX}_B" --ros-args -p use_sim_time:=true \
  > "$L/${PFX}_B_logger.log" 2>&1 &
LPID=$!
sleep 2
ros2 bag play "$BAG" --clock 200 --topics /tf_static /scan /odom /ekf/imu/data /map /mocap_pose1 /mocap_pose2 \
  > "$L/${PFX}_B_bag.log" 2>&1
sleep 5
kill -INT $LPID 2>/dev/null || true; sleep 2; kill -9 $LPID $MCLPID 2>/dev/null || true
pkill -9 -f "als_ros/mcl" 2>/dev/null || true
pkill -9 -f "install/als_ros" 2>/dev/null || true
pkill -9 -f "static_transform_publisher" 2>/dev/null || true
echo "[$PFX/B] rows: mcl=$(wc -l < "$L/${PFX}_B_mcl.csv" 2>/dev/null || echo 0) gt=$(wc -l < "$L/${PFX}_B_gt_ego.csv" 2>/dev/null || echo 0)"

echo
echo "########## [$PFX] Test A analysis (baked, real-time) ##########"
python3 eval/tools/analyze_localization.py "$L/${PFX}_A_mcl.csv" "$L/${PFX}_A_gt_ego.csv" --lane "$LANE" || true
echo
echo "########## [$PFX] Test B analysis (fresh replay-computed) ##########"
python3 eval/tools/analyze_localization.py "$L/${PFX}_B_mcl.csv" "$L/${PFX}_B_gt_ego.csv" --lane "$LANE" || true
