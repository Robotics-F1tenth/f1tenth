#!/usr/bin/env bash

# Run mocap calibration for rigid body 6 (/mocap_pose1).
cali1() (
    cd /home/a/f1tenth_ws || exit 1
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    exec python3 src/localization/mocap_localization/calibration_v2.py \
        --ros-args -p obj:=1
)

# Run mocap calibration for rigid body 7 (/mocap_pose2).
cali2() (
    cd /home/a/f1tenth_ws || exit 1
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    exec python3 src/localization/mocap_localization/calibration_v2.py \
        --ros-args -p obj:=2
)

# Run only mocap localization for rigid body 6 using params.yaml.
mocap_launch1() (
    cd /home/a/f1tenth_ws || exit 1
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    exec ros2 launch mocap_localization mocap_localization.launch.py with_map:=false
)

# Run only mocap localization for rigid body 7 using params2.yaml.
mocap_launch2() (
    cd /home/a/f1tenth_ws || exit 1
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    exec ros2 launch mocap_localization mocap_localization2.launch.py
)
