#!/bin/bash
# C-03 풀스택 A/B: MCL(라이브) + perception(라이브) 를 bag 센서로 구동하고
# perception 트래커 출력과 자차 MCL pose 를 mocap GT 와 비교한다.
# bag 의 baked /tf 는 재생하지 않는다(라이브 MCL 이 map->base_link 를 발행하므로).
set +e
L="$(cd "$(dirname "$0")" && pwd)"
export ROS_DOMAIN_ID=43
NAME=$1; PARAMS=$2
BAG=/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

pkill -9 -f "als_ros/mcl" 2>/dev/null; pkill -9 -f "perception_node" 2>/dev/null
pkill -9 -f "ros2 bag play" 2>/dev/null; sleep 2

MCL_PARAMS=$PARAMS ros2 launch "$L/eval_mcl_launch_bagmap.py" > "$L/${NAME}_mcl.log" 2>&1 &
sleep 3
ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
  csv_path:=map1_path > "$L/${NAME}_perc.log" 2>&1 &
sleep 4

rm -f "$L"/${NAME}_*.csv
cd /home/a/f1tenth
python3 eval/tools/record_ros_topics.py --out "$L/${NAME}" --ros-args -p use_sim_time:=true \
  > "$L/${NAME}_rec.log" 2>&1 &
sleep 2

ros2 bag play "$BAG" --clock \
  --topics /tf_static /scan /odom /ekf/imu/data /map /mocap_pose1 /mocap_pose2 \
  > "$L/${NAME}_bag.log" 2>&1
sleep 4
pkill -9 -f "record_ros_topics" 2>/dev/null; sleep 1
pkill -9 -f "als_ros/mcl" 2>/dev/null; pkill -9 -f "perception_node" 2>/dev/null
echo "[$NAME] rows: mcl=$(wc -l < "$L/${NAME}_mcl.csv" 2>/dev/null) tracks=$(wc -l < "$L/${NAME}_tracks.csv" 2>/dev/null) gt_opp=$(wc -l < "$L/${NAME}_gt_opp.csv" 2>/dev/null)"
