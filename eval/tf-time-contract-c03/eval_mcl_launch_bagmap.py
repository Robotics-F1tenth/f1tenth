import os
from launch import LaunchDescription
from launch_ros.actions import Node

# No map_server here on purpose: the repo's map7.yaml/map1.yaml are stale/different
# crops of the actual maps used on 2026-07-30 (confirmed: map7.yaml=175x269 vs the
# bag's own baked /map=133x221, different origin). als_ros mcl subscribes to /map
# directly (map_name param), so we just replay the bag's own /map topic instead --
# guarantees the exact occupancy grid + origin used during that day's recording.
PARAMS = os.environ["MCL_PARAMS"]


def generate_launch_description():
    # base_link->laser 하드코딩 제거 — bag 의 /tf_static 을 재생해서 녹화 당시 실제
    # 장착값을 쓴다 (run_bag.sh 의 --topics 에 /tf_static 포함).
    # 이유: bag 마다 이 값이 다르다. record5=0.11/0/0, 0730=0.27/0/0.11.
    # 하드코딩하면 bag 에 따라 최대 16cm 종방향 편향이 생긴다.
    # 상세: docs/localization-version-comparison.md
    return LaunchDescription([
        Node(package="als_ros", executable="mcl", name="mcl",
             parameters=[PARAMS], output="screen"),
    ])
