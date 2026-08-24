#!/bin/bash
# replay_0824.sh <tag> [yaml overrides...]
# Replays the 0824 bag with debug outputs on and records what
# tools/plot_dynamic_track_layers.py needs (/f1/.../arr and /fit_debug_markers).
M="$(cd "$(dirname "$0")" && pwd)"
Y=/home/a/f1tenth/src/perception/config/perception/f1tenth.yaml
BAG="/home/a/dataset_custom/F1tenth/2026 IFAC/0824/0824/rosbag2_2026_08_24-00_28_49"
TAG=$1; shift
BK=$(mktemp); cp "$Y" "$BK"
restore() { cp "$BK" "$Y"; rm -f "$BK"; }
trap restore EXIT
for kv in "$@"; do
    key="${kv%%=*}"; val="${kv#*=}"
    section="${key%.*}"; leaf="${key##*.}"
    python3 - "$Y" "$section" "$leaf" "$val" <<'PY'
import re, sys
path, section, leaf, val = sys.argv[1:5]
lines = open(path).read().split("\n")
sec = re.compile(r"^( *)" + re.escape(section) + r":\s*$")
lf = re.compile(r"^( *)" + re.escape(leaf) + r":.*$")
inside = False; ind = None
for i, line in enumerate(lines):
    if not inside:
        m = sec.match(line)
        if m: inside = True; ind = len(m.group(1))
        continue
    cur = len(line) - len(line.lstrip(" "))
    if line.strip() and cur <= ind: break
    m = lf.match(line)
    if m:
        h = line.find("#")
        lines[i] = f"{m.group(1)}{leaf}: {val}" + ("  " + line[h:] if h != -1 else "")
        open(path, "w").write("\n".join(lines)); sys.exit(0)
sys.exit(f"{section}.{leaf} not found")
PY
    [ $? -ne 0 ] && { echo "param inject failed: $kv"; exit 1; }
done
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=91
pkill -9 -f "[p]erception_node" 2>/dev/null || true
pkill -9 -f "[r]os2 bag" 2>/dev/null || true
sleep 1
ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    publish_debug_outputs:=true publish_race_visualization:=true \
    > "$M/${TAG}.log" 2>&1 &
sleep 5
rm -rf "$M/replay_$TAG"
ros2 bag record -o "$M/replay_$TAG" \
    /f1/perception/object/obstacles/arr /fit_debug_markers > /dev/null 2>&1 &
REC=$!
sleep 2
ros2 bag play "$BAG" --clock 200 > /dev/null 2>&1
sleep 3
kill -INT $REC 2>/dev/null || true
sleep 4
pkill -9 -f "[p]erception_node" 2>/dev/null || true
echo "done -> $M/replay_$TAG"
