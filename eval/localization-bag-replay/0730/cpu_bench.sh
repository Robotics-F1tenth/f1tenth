#!/bin/bash
# Absolute CPU cost per processed scan: total utime+stime of the mcl process
# divided by scans it actually processed. Percentages from perf report can't
# answer "did total work go down" -- this can.
set -o pipefail
L="/tmp/claude-1000/-home-a-f1tenth/3fda53a2-f142-4ee1-ba13-00bfbdc2ff66/scratchpad"
LAUNCH="/home/a/f1tenth/eval/localization-bag-replay/0730/eval_mcl_launch_bagmap.py"
BAG="/home/a/Downloads/f1tenth_bag/0730/rosbag2_2026_07_30-22_52_38"
PARAMS="$L/params_2252_correct.yaml"
PFX="$1"; TRIALS="${2:-3}"
export ROS_DOMAIN_ID=63
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
HZ=$(getconf CLK_TCK)

for t in $(seq 1 "$TRIALS"); do
  NAME="${PFX}_cpu${t}"
  MCL_PARAMS="$PARAMS" ros2 launch "$LAUNCH" > "$L/${NAME}.log" 2>&1 &
  sleep 4
  PID=$(pgrep -f "install/als_ros/lib/als_ros/mcl" | head -1)
  [ -z "$PID" ] && { echo "[$NAME] mcl not found"; continue; }

  timeout 110 ros2 bag play "$BAG" --clock 200 --disable-keyboard-controls \
    --topics /tf_static /scan /odom /ekf/imu/data /map < /dev/null > /dev/null 2>&1

  # read CPU jiffies before killing
  STATS=$(cat /proc/$PID/stat 2>/dev/null)
  UT=$(echo "$STATS" | awk '{print $14}')
  ST=$(echo "$STATS" | awk '{print $15}')
  PROC=$(grep "MCL stats" "$L/${NAME}.log" | tail -1 | sed -n 's/.*processed=\([0-9]*\).*/\1/p')

  for p in $(pgrep -f "install/als_ros/lib/als_ros/mcl" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
  for p in $(pgrep -f "ros2 launch.*eval_mcl" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
  sleep 1

  python3 -c "
ut, st, hz, proc = $UT, $ST, $HZ, ${PROC:-0}
cpu = (ut + st) / hz
print(f'[$NAME] cpu={cpu:.2f}s  scans={proc}  per_scan={cpu/proc*1000:.3f} ms' if proc else f'[$NAME] cpu={cpu:.2f}s scans=0')
"
done
echo "bench done"
