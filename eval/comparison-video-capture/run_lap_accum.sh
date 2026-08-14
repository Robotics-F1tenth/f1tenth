#!/bin/bash
# Record S7 (lap accumulation before/after the EKF cold-start fix). Same current
# codebase both times -- the "before" behaviour is reproduced exactly by disabling
# min_age (0.0), which is data-verified equivalent to the pre-fix commit's behaviour
# (see perception-workplan-0805.md B-6). No worktree/rebuild needed either way.
#
#   bash run_lap_accum.sh before 22_48_25   # min_age=0.0 -- watch orange disks pile up
#   bash run_lap_accum.sh after  22_48_25   # min_age=0.3 (current default) -- stays flat
#
# Best bags for this: 22_48_25 or 22_52_38 (both have real static obstacles + a lapped
# track short enough to show 3+ laps in a couple minutes).
# NOTE: no `set -u` -- ROS's setup.bash reads unset vars and dies silently under it.
VER=${1:-}; BAGKEY=${2:-}
if [ "$VER" != "before" ] && [ "$VER" != "after" ]; then
    echo "usage: bash run_lap_accum.sh <before|after> <bag_key>"
    echo "  recommended bag_key: 22_48_25 or 22_52_38"
    exit 1
fi
[ -z "$BAGKEY" ] && { echo "usage: bash run_lap_accum.sh <before|after> <bag_key>"; exit 1; }

WS=/home/a/f1tenth
BAG=$(ls -d /home/a/Downloads/0730/rosbag2_2026_07_30-${BAGKEY} 2>/dev/null)
[ -z "$BAG" ] && { echo "bag not found: $BAGKEY"; exit 1; }
case "$BAGKEY" in
    22_48_25|22_52_38|22_55_09) LANE=map1_path ;;
    *)                          LANE=map7_path ;;
esac

YAML=$WS/src/perception/config/perception/f1tenth.yaml
RVIZ=$WS/src/perception/config/perception/f1test.rviz
BACKUP=$(mktemp)
cp "$YAML" "$BACKUP"
sed -i "s|^\( *csv_path: \).*|\1\"$LANE\"|" "$YAML"
if [ "$VER" = "before" ]; then
    sed -i "s|^\( *min_age: \).*|\10.0             # [영상용 임시] 0.3->0.0, 콜드스타트 게이트 비활성|" "$YAML"
    echo "=== BEFORE (min_age=0.0, pre-fix와 데이터상 동일) -- 랩마다 원반이 쌓이는지 확인 ==="
else
    echo "=== AFTER (min_age=0.3, 현재 기본값) -- 평평하게 유지되는지 확인 ==="
fi

cleanup() {
    echo; echo "[cleanup] restoring yaml and killing nodes"
    cp "$BACKUP" "$YAML"; rm -f "$BACKUP"
    kill $LAUNCH_PID $RVIZ_PID 2>/dev/null
    pkill -9 -f "[p]erception_node" 2>/dev/null
    pkill -9 -f "[r]viz2" 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export ROS_DOMAIN_ID=63

echo "=== bag: $BAGKEY  lane: $LANE ==="
ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false &
LAUNCH_PID=$!
sleep 4

rviz2 -d "$RVIZ" --ros-args -p use_sim_time:=true &
RVIZ_PID=$!
sleep 3

echo "=== playing bag once through -- watch /static_obstacle_markers (orange disks) ==="
echo "=== bags are ~4-6 laps; --clock 200 = playback is unthrottled by wall clock ==="
ros2 bag play "$BAG" --clock 200

echo "=== bag finished; leaving rviz up. Ctrl-C to quit. ==="
wait $RVIZ_PID
