#!/bin/bash
# rviz_consumer_probe.sh <bag_key> <minutes>
# Measures RVIZ ITSELF (the consumer), not the publisher. The publisher probe
# already showed perception's markers plateau and its RSS stays flat, so if the
# lag is real it has to show up here: rviz RSS climbing or its CPU% ratcheting up
# over minutes while the same bag loops.
# NOTE: no `set -u` -- ROS setup.bash reads unset vars and dies silently under it.
M="$(cd "$(dirname "$0")" && pwd)"
BAGKEY=${1:-22_48_25}
MINUTES=${2:-5}
BAG=$(ls -d /home/a/Downloads/0730/rosbag2_2026_07_30-${BAGKEY} 2>/dev/null)
[ -z "$BAG" ] && { echo "bag not found: $BAGKEY"; exit 1; }
case "$BAGKEY" in
    22_48_25|22_52_38|22_55_09) LANE=map1_path ;;
    *)                          LANE=map7_path ;;
esac
OUT="$M/rviz_consumer_${BAGKEY}.log"
RVIZ_CFG=/home/a/f1tenth/src/perception/config/perception/f1test.rviz

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=74

pkill -9 -f "[p]erception_node" 2>/dev/null
pkill -9 -f "[r]os2 bag play" 2>/dev/null
pkill -9 -f "[r]viz2" 2>/dev/null
sleep 1

ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    csv_path:="$LANE" > "$M/rviz_consumer_perception.log" 2>&1 &
sleep 5
rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true > "$M/rviz_consumer_rviz.log" 2>&1 &
sleep 8
ros2 bag play "$BAG" --clock 200 --loop > /dev/null 2>&1 &
sleep 4

PPID_PERC=$(pgrep -f "[p]erception_node" | head -1)
PID_RVIZ=$(pgrep -f "[r]viz2" | head -1)
echo "perception=$PPID_PERC rviz=$PID_RVIZ bag=$BAGKEY dur=${MINUTES}min" | tee "$OUT"
echo "elapsed_s rviz_rss_kb rviz_cpu% perc_rss_kb perc_cpu% marker_hz" | tee -a "$OUT"

END=$(( $(date +%s) + MINUTES*60 ))
START=$(date +%s)
while [ "$(date +%s)" -lt "$END" ]; do
    EL=$(( $(date +%s) - START ))
    RR=$(awk '/VmRSS/{print $2}' /proc/$PID_RVIZ/status 2>/dev/null)
    PR=$(awk '/VmRSS/{print $2}' /proc/$PPID_PERC/status 2>/dev/null)
    RC=$(ps -p $PID_RVIZ -o %cpu= 2>/dev/null | tr -d ' ')
    PC=$(ps -p $PPID_PERC -o %cpu= 2>/dev/null | tr -d ' ')
    HZ=$(timeout 6 ros2 topic hz /static_obstacle_markers 2>/dev/null | grep -m1 "average rate" | awk '{print $3}')
    echo "$EL ${RR:-NA} ${RC:-NA} ${PR:-NA} ${PC:-NA} ${HZ:-NA}" | tee -a "$OUT"
    sleep 15
done

bash "$M/kill_probe.sh"
echo "done -> $OUT"
