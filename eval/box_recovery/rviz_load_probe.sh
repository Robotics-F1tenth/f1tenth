#!/bin/bash
# rviz_load_probe.sh <bag_key> <minutes>
# Does perception's RViz-facing load grow over time? Replays a bag on --loop and
# samples, once per 20s: marker counts per marker topic, publish rates, and the
# perception process's RSS. Growth in marker count or RSS => accumulation
# (the thing that would make RViz progressively laggy while topics look fine).
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
OUT="$M/rviz_probe_${BAGKEY}.log"

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=73

pkill -9 -f "[p]erception_node" 2>/dev/null
pkill -9 -f "[r]os2 bag play" 2>/dev/null
sleep 1

ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    csv_path:="$LANE" > "$M/rviz_probe_perception.log" 2>&1 &
LPID=$!
sleep 5
ros2 bag play "$BAG" --clock 200 --loop > /dev/null 2>&1 &
BPID=$!
sleep 5

PID=$(pgrep -f "[p]erception_node" | head -1)
echo "perception pid=$PID  bag=$BAGKEY  duration=${MINUTES}min" | tee "$OUT"
echo "elapsed_s rss_kb static_markers box_markers fit_markers track_markers static_obs_count" | tee -a "$OUT"

END=$(( $(date +%s) + MINUTES*60 ))
START=$(date +%s)
while [ "$(date +%s)" -lt "$END" ]; do
    EL=$(( $(date +%s) - START ))
    RSS=$(awk '/VmRSS/{print $2}' /proc/$PID/status 2>/dev/null || echo 0)
    # markers[] length in one sample of each marker topic
    SM=$(timeout 4 ros2 topic echo /static_obstacle_markers --once 2>/dev/null | grep -c '^- header:' || echo 0)
    BM=$(timeout 4 ros2 topic echo /bounding_box_markers --once 2>/dev/null | grep -c '^- header:' || echo 0)
    FM=$(timeout 4 ros2 topic echo /fit_debug_markers --once 2>/dev/null | grep -c '^- header:' || echo 0)
    TM=$(timeout 4 ros2 topic echo /track_state_markers --once 2>/dev/null | grep -c '^- header:' || echo 0)
    SO=$(timeout 4 ros2 topic echo /f1/perception/static_obstacles --once 2>/dev/null | grep -c '^- header:' || echo 0)
    echo "$EL $RSS $SM $BM $FM $TM $SO" | tee -a "$OUT"
    sleep 20
done

kill -9 $LPID $BPID 2>/dev/null
pkill -9 -f "[p]erception_node" 2>/dev/null
pkill -9 -f "[r]os2 bag play" 2>/dev/null
echo "done -> $OUT"
