#!/bin/bash
# bench.sh <current|march> <bag_key>
# Headless benchmark run of one version against one bag, using the SAME eval harness
# for both (the March original publishes to the same /f1/perception/object/obstacles/arr,
# so record_ros_topics.py attaches unmodified -- that is what makes the comparison fair).
# Results land in eval/perception-march-baseline-comparison/<ver>_<bag>_*.csv and are read by analyze_perception_tracking.py.
# NOTE: no `set -u` here. ROS's own setup.bash reads unset variables
# (AMENT_TRACE_SETUP_FILES etc.), so -u kills the script at the source line
# with no output at all. Args are checked explicitly below instead.
M="$(cd "$(dirname "$0")" && pwd)"
VER=${1:-}; BAGKEY=${2:-}
[ -z "$VER" ] || [ -z "$BAGKEY" ] && { echo "usage: bash bench.sh <current|march> <bag_key>"; exit 1; }

case "$VER" in
    current) WS=/home/a/f1tenth ;;
    march)   WS=/home/a/f1tenth_march ;;
    *) echo "first arg must be 'current' or 'march'"; exit 1 ;;
esac
BAG=$(ls -d /home/a/Downloads/0730/rosbag2_2026_07_30-${BAGKEY} 2>/dev/null)
[ -z "$BAG" ] && { echo "bag not found: $BAGKEY"; exit 1; }
case "$BAGKEY" in
    22_48_25|22_52_38|22_55_09) LANE=map1_path ;;
    *)                          LANE=map7_path ;;
esac

PFX="${VER}_${BAGKEY}"
YAML=$WS/src/perception/config/perception/f1tenth.yaml
BACKUP="$M/.yaml_backup_$PFX"
cp "$YAML" "$BACKUP"
sed -i "s|^\( *csv_path: \).*|\1\"$LANE\"|" "$YAML"

source /opt/ros/humble/setup.bash 2>/dev/null
source $WS/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=59

pkill -9 -f "[p]erception_node" 2>/dev/null
pkill -9 -f "[e]val_logger.py" 2>/dev/null
pkill -9 -f "[r]os2 bag play" 2>/dev/null
sleep 1

if [ "$VER" = "march" ]; then
    ros2 launch f1_perception f1_perception.launch.py > "$M/${PFX}_perception.log" 2>&1 &
else
    ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
        csv_path:="$LANE" \
        > "$M/${PFX}_perception.log" 2>&1 &
fi
PERCPID=$!
sleep 4

rm -f "$M"/${PFX}_*.csv
python3 /home/a/f1tenth/eval/tools/record_ros_topics.py --out "$M/${PFX}" --ros-args -p use_sim_time:=true \
    > "$M/${PFX}_logger.log" 2>&1 &
LPID=$!
sleep 2

ros2 bag play "$BAG" --clock 200 > "$M/${PFX}_bag.log" 2>&1
sleep 3

kill -INT $LPID 2>/dev/null; sleep 2; kill -9 $LPID 2>/dev/null
kill -9 $PERCPID 2>/dev/null
pkill -9 -f "[p]erception_node" 2>/dev/null
cp "$BACKUP" "$YAML"; rm -f "$BACKUP"

TR=$(wc -l < "$M/${PFX}_tracks.csv" 2>/dev/null || echo 0)
echo "[$PFX] tracks=$TR gt_opp=$(wc -l < "$M/${PFX}_gt_opp.csv" 2>/dev/null || echo 0)"
[ "$TR" -lt 100 ] && echo "  !! tracks row count is tiny -- treat as a capture failure and re-run, not a result"
python3 /home/a/f1tenth/eval/tools/analyze_perception_tracking.py "$M/${PFX}" 2>/dev/null | head -30
