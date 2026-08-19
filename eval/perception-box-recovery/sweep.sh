#!/bin/bash
# sweep.sh <tag> <bag_key> [section.leaf=value ...]
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

# ponytail: localization bag replays share this machine's CPU. Timing-sensitive
# MCL measurements (cycle time etc) get skewed by a concurrent perception replay
# even on a different ROS_DOMAIN_ID -- that's contention, not DDS crosstalk, so
# domain separation doesn't fix it. Wait instead of racing them.
localization_running() {
    pgrep -f "mcl --ros-args" > /dev/null ||
    pgrep -f "mcl_node" > /dev/null ||
    pgrep -f "bag_mcl" > /dev/null
}
while localization_running; do
    echo "waiting: localization mcl process is running, not stealing CPU from it..." >&2
    sleep 15
done

BK=$(mktemp); cp "$Y" "$BK"
restore() { cp "$BK" "$Y"; rm -f "$BK"; }
trap restore EXIT

sed -i "s|^      csv_path: .*|      csv_path: \"$LANE\"|" "$Y"
for kv in "$@"; do
    keypath="${kv%%=*}"; val="${kv#*=}"
    # keypath is "section.leaf" (e.g. devide_wall.line_dist) -- matches the leaf
    # only inside the block opened by "section:", so a leaf name that repeats
    # under a different section (line_dist under both devide_wall and
    # concatnate_wall) can't silently hit the wrong one.
    section="${keypath%.*}"; leaf="${keypath##*.}"
    python3 - "$Y" "$section" "$leaf" "$val" <<'PYEOF'
import re, sys
path, section, leaf, val = sys.argv[1:5]
lines = open(path).read().split("\n")
sec_re = re.compile(r"^( *)" + re.escape(section) + r":\s*$")
leaf_re = re.compile(r"^( *)" + re.escape(leaf) + r":.*$")
in_section = False
section_indent = None
done = False
for i, line in enumerate(lines):
    if not in_section:
        m = sec_re.match(line)
        if m:
            in_section = True
            section_indent = len(m.group(1))
        continue
    indent = len(line) - len(line.lstrip(" "))
    if line.strip() and indent <= section_indent:
        break  # left the section without finding the leaf
    m = leaf_re.match(line)
    if m:
        rest = line[len(m.group(0)):] if False else ""
        # preserve trailing comment
        comment = ""
        hashpos = line.find("#")
        if hashpos != -1:
            comment = "  " + line[hashpos:]
        lines[i] = f"{m.group(1)}{leaf}: {val}{comment}"
        done = True
        break
if not done:
    sys.exit(f"sweep.sh: {section}.{leaf} not found in {path}")
open(path, "w").write("\n".join(lines))
PYEOF
    [ $? -ne 0 ] && { restore; exit 1; }
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
