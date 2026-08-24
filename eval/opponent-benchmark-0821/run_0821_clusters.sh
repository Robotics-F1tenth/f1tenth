#!/bin/bash
# Replay 0821 once with debug outputs on, capturing the per-frame clustered
# points (/obj_cluster, cluster id in intensity) plus map<-laser TF, so centroid
# estimators can be compared offline instead of by one 7-minute replay each.
# Neither `set -u` nor `set -e`: ROS setup.bash dereferences unset variables, and
# the pkill/kill calls here return non-zero whenever nothing matches, which is the
# normal case on a clean machine. Both flags silently aborted this script before
# it ever started recording.
M="$(cd "$(dirname "$0")" && pwd)"
BAG=/home/a/Downloads/f1tenth_bag/0821/rosbag2_2026_08_21-03_13_15
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/a/f1tenth/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=88
pkill -9 -f "[p]erception_node" 2>/dev/null || true
pkill -9 -f "[r]os2 bag play" 2>/dev/null || true
sleep 1
ros2 launch f1_perception f1_perception.launch.py use_sim_time:=true use_rviz:=false \
    publish_debug_outputs:=true > "$M/clusters_perception.log" 2>&1 &
sleep 5
ros2 bag record -o "$M/clusters_out" \
    /obj_cluster /tf /tf_static /f1/perception/object/obstacles/arr \
    /mocap/car3/pose /newmcl_pose > /dev/null 2>&1 &
REC=$!
sleep 2
# The bags were recorded with perception running, so they carry
# /f1/perception/{object/obstacles/arr,static_obstacles} themselves. Replayed
# as-is they drown out the live node under test and the capture measures the
# recording instead -- that silently invalidated a boundary A/B until the node's
# own funnel counters contradicted the captured topic. Humble's bag play has no
# --exclude, so shunt them to dead names.
ros2 bag play "$BAG" --clock 200 \
    --remap /f1/perception/object/obstacles/arr:=/bagged/obstacles/arr /f1/perception/static_obstacles:=/bagged/static_obstacles > /dev/null 2>&1
sleep 3
kill -INT $REC 2>/dev/null || true
sleep 5
pkill -9 -f "[p]erception_node" 2>/dev/null || true
echo "captured -> $M/clusters_out"
