#!/bin/bash
# topic_stall_probe.sh
# Both earlier probes lost /static_obstacle_markers hz at ~95s. Is the topic
# actually stalling, or was that just `ros2 topic hz` struggling? Runs WITHOUT
# rviz (so rviz load can't be the cause) and checks several topics at intervals,
# reporting message counts observed in a fixed window rather than relying on hz.
M="$(cd "$(dirname "$0")" && pwd)"
BAG=/home/a/Downloads/0730/rosbag2_2026_07_30-22_48_25
OUT="$M/topic_stall.log"

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=76
bash "$M/kill_probe.sh"

ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    csv_path:=map1_path > "$M/topic_stall_perception.log" 2>&1 &
sleep 5
ros2 bag play "$BAG" --clock 200 --loop > /dev/null 2>&1 &
sleep 4

# msgs seen in a 5s window == a rate that does not depend on `ros2 topic hz` internals
count5() { timeout 5 ros2 topic echo "$1" 2>/dev/null | grep -c '^---' ; }

echo "elapsed_s static_markers_msgs5s obstacles_arr_msgs5s scan_msgs5s perc_rss_kb" | tee "$OUT"
START=$(date +%s)
for i in $(seq 1 10); do
    PID=$(pgrep -f "[p]erception_node" | head -1)
    A=$(count5 /static_obstacle_markers)
    B=$(count5 /f1/perception/object/obstacles/arr)
    C=$(count5 /scan)
    R=$(awk '/VmRSS/{print $2}' /proc/$PID/status 2>/dev/null)
    echo "$(( $(date +%s) - START )) ${A:-0} ${B:-0} ${C:-0} ${R:-NA}" | tee -a "$OUT"
    sleep 12
done

bash "$M/kill_probe.sh"
echo "done -> $OUT"
