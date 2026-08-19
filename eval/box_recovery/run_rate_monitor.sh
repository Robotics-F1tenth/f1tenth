#!/bin/bash
# run_rate_monitor.sh <seconds> [with_rviz]
M="$(cd "$(dirname "$0")" && pwd)"
DUR=${1:-240}
WITH_RVIZ=${2:-no}
BAG=/home/a/Downloads/0730/rosbag2_2026_07_30-22_48_25
OUT="$M/rate_monitor_${WITH_RVIZ}.log"

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=77
bash "$M/kill_probe.sh"

ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    csv_path:=map1_path > "$M/rate_perception_${WITH_RVIZ}.log" 2>&1 &
sleep 5
if [ "$WITH_RVIZ" = "rviz" ]; then
    rviz2 -d /home/a/f1tenth/src/perception/config/perception/f1test.rviz \
        --ros-args -p use_sim_time:=true > "$M/rate_rviz.log" 2>&1 &
    sleep 8
fi
ros2 bag play "$BAG" --clock 200 > /dev/null 2>&1 &
sleep 3

python3 "$M/rate_monitor.py" "$DUR" 2>&1 | tee "$OUT"

bash "$M/kill_probe.sh"
echo "done -> $OUT"
