#!/bin/bash
# rviz_inst_probe.sh <minutes>
# The previous probe used `ps -o %cpu`, which is the average over the process's
# whole lifetime -- it rises toward the true value regardless of degradation, so
# it cannot answer "does it get worse over time". This samples INSTANTANEOUS cpu
# from /proc/<pid>/stat jiffy deltas instead, plus marker publish rate, so a real
# ratchet would be visible as a rising instantaneous %.
M="$(cd "$(dirname "$0")" && pwd)"
MINUTES=${1:-3}
BAG=/home/a/Downloads/0730/rosbag2_2026_07_30-22_48_25
OUT="$M/rviz_inst.log"
RVIZ_CFG=/home/a/f1tenth/src/perception/config/perception/f1test.rviz

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=75
bash "$M/kill_probe.sh"

ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    csv_path:=map1_path > "$M/rviz_inst_perception.log" 2>&1 &
sleep 5
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true > "$M/rviz_inst_rviz.log" 2>&1 &
sleep 8
ros2 bag play "$BAG" --clock 200 --loop > /dev/null 2>&1 &
sleep 4

PIDR=$(pgrep -f "[r]viz2" | head -1)
PIDP=$(pgrep -f "[p]erception_node" | head -1)
HZ_TICK=$(getconf CLK_TCK)

jif() { awk '{print $14+$15}' /proc/$1/stat 2>/dev/null; }

echo "rviz=$PIDR perception=$PIDP clk_tck=$HZ_TICK" | tee "$OUT"
echo "elapsed_s rviz_inst_cpu% perc_inst_cpu% rviz_rss_kb marker_hz" | tee -a "$OUT"

END=$(( $(date +%s) + MINUTES*60 ))
START=$(date +%s)
PR0=$(jif $PIDR); PP0=$(jif $PIDP); T0=$(date +%s.%N)
while [ "$(date +%s)" -lt "$END" ]; do
    sleep 15
    PR1=$(jif $PIDR); PP1=$(jif $PIDP); T1=$(date +%s.%N)
    DT=$(echo "$T1 - $T0" | bc)
    RCPU=$(echo "scale=1; 100*($PR1-$PR0)/$HZ_TICK/$DT" | bc 2>/dev/null)
    PCPU=$(echo "scale=1; 100*($PP1-$PP0)/$HZ_TICK/$DT" | bc 2>/dev/null)
    RSS=$(awk '/VmRSS/{print $2}' /proc/$PIDR/status 2>/dev/null)
    HZ=$(timeout 8 ros2 topic hz /static_obstacle_markers 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
    echo "$(( $(date +%s) - START )) ${RCPU:-NA} ${PCPU:-NA} ${RSS:-NA} ${HZ:-NA}" | tee -a "$OUT"
    PR0=$PR1; PP0=$PP1; T0=$T1
done

bash "$M/kill_probe.sh"
echo "done -> $OUT"
