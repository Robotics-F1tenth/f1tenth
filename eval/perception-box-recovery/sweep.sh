#!/bin/bash
# sweep.sh <tag> <bag_key> [yaml.path=value ...]
# Applies yaml overrides, replays, restores the yaml, and prints one summary line.
# csv_path is forced to match the bag (production yaml points at the new track).
M="$(cd "$(dirname "$0")" && pwd)"
Y=/home/a/f1tenth/src/perception/config/perception/f1tenth.yaml
TAG=$1; BAGKEY=$2; shift 2
BAG=$(ls -d /home/a/Downloads/f1tenth_bag/0730/rosbag2_2026_07_30-${BAGKEY} 2>/dev/null)
[ -z "$BAG" ] && { echo "bag not found: $BAGKEY"; exit 1; }
case "$BAGKEY" in
    22_48_25|22_52_38|22_55_09) LANE=map1_path ;;
    *)                          LANE=map7_path ;;
esac

BK=$(mktemp); cp "$Y" "$BK"
restore() { cp "$BK" "$Y"; rm -f "$BK"; }
trap restore EXIT

sed -i "s|^      csv_path: .*|      csv_path: \"$LANE\"|" "$Y"
for kv in "$@"; do
    key="${kv%%=*}"; val="${kv#*=}"
    # last path element is the yaml key; match it at any indent
    leaf="${key##*.}"
    sed -i "0,/^\( *\)${leaf}:.*/s//\1${leaf}: ${val}/" "$Y"
done
# yaml edits reach the node through the symlinked install share dir; no rebuild needed

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=79
pkill -9 -f "[p]erception_node" 2>/dev/null
pkill -9 -f "[r]os2 bag play" 2>/dev/null
sleep 1

ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    > "$M/${TAG}_perception.log" 2>&1 &
sleep 4
rm -f "$M/${TAG}_static_obstacles.txt"
ros2 topic echo /f1/perception/static_obstacles > "$M/${TAG}_static_obstacles.txt" 2>&1 &
SPID=$!
sleep 2
ros2 bag play "$BAG" --clock 200 > /dev/null 2>&1
sleep 3
kill -9 $SPID 2>/dev/null
pkill -9 -f "[p]erception_node" 2>/dev/null
sleep 1

LM=$(python3 "$M/summarize_box.py" "$TAG" 2>/dev/null | head -1 | grep -o '총 [0-9]*' | grep -o '[0-9]*')
BOX=$(python3 "$M/summarize_box.py" "$TAG" 2>/dev/null | grep -c "BOX HIT")
echo "$TAG bag=$BAGKEY LM=${LM:-?} box=${BOX:-0} :: $*"
