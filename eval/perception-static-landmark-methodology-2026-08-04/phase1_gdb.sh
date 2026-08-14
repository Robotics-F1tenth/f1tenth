#!/bin/bash
# Reproduce the assoc_dist>=0.70 segfault under gdb and capture a backtrace.
# perception_node is launched directly (not via ros2 launch) so gdb owns the process.
M=/home/a/f1tenth/eval/perception-static-landmark-methodology-2026-08-04
IY=/home/a/f1tenth/install/f1_perception/share/f1_perception/config/perception/f1tenth.yaml
BAG=/home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null

pkill -9 -f "perception_node" 2>/dev/null || true
pkill -9 -f "ros2 bag play" 2>/dev/null || true
sleep 1

cp "$IY" "$M/.yaml_backup_gdb"
python3 "$M/setparam.py" "$IY" static_landmark.assoc_dist 0.70 > /dev/null

cat > "$M/gdb_cmds.txt" <<'GEOF'
set pagination off
set confirm off
run --ros-args -r __node:=perception_node --params-file /home/a/f1tenth/install/f1_perception/share/f1_perception/config/perception/f1tenth.yaml -p use_sim_time:=true
echo \n===== BACKTRACE =====\n
bt
echo \n===== FRAME DETAIL =====\n
bt full
echo \n===== REGISTERS/FAULT =====\n
info frame
quit
GEOF

gdb -q -x "$M/gdb_cmds.txt" --args /home/a/f1tenth/install/f1_perception/lib/f1_perception/perception_node \
  > "$M/phase1_gdb_out.log" 2>&1 &
GDBPID=$!
sleep 6
ros2 bag play "$BAG" --clock 200 > "$M/phase1_bag.log" 2>&1 &
BAGPID=$!
# the crash historically happens within the first seconds of playback
sleep 45
kill -9 $BAGPID 2>/dev/null || true
sleep 2
kill -9 $GDBPID 2>/dev/null || true
pkill -9 -f "perception_node" 2>/dev/null || true

cp "$M/.yaml_backup_gdb" "$IY"
echo "restored assoc_dist=$(python3 "$M/setparam.py" "$IY" --get static_landmark.assoc_dist)"
