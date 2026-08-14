import os
from launch import LaunchDescription
from launch_ros.actions import Node

P = os.path.dirname(os.path.realpath(__file__))
PARAMS = os.environ["MCL_PARAMS"]

def generate_launch_description():
    # base_link->laser 를 여기서 하드코딩하지 않는다. 대신 bag 의 /tf_static 을 그대로
    # 재생해서(run_one.sh 의 --topics 에 포함) 그 bag 이 녹화될 당시의 실제 장착값을 쓴다.
    #
    # 왜: 예전에는 0.27/0/0.11 을 하드코딩했는데, record5(2026-03-13)는 0.11/0/0 으로
    # 녹화된 bag이라 16cm 가 어긋났다. MCL 은 base_link 를 추정하면서 이 오프셋만큼
    # 스캔 위치를 보정하므로, 오프셋이 16cm 크면 추정 base_link 가 그만큼 뒤로 밀린다
    # -- 오래 미제였던 "replay 만 -13~-17cm 후방 편향" 의 정체가 이것이었다.
    # (0730 bag 은 0.27/0/0.11 로 녹화돼 우연히 일치했고, 그래서 편향이 안 나왔다.)
    #
    # /tf_static 에는 base_link->laser, base_link->imu_link 만 있고 map->base_link 는
    # 동적 /tf 쪽에 있으므로, /tf_static 만 재생하는 것은 이중 발행 위험이 없다.
    return LaunchDescription([
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[P + "/map.yaml"], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization",
             parameters=[P + "/lifecycle.yaml"], output="screen"),
        Node(package="als_ros", executable="mcl", name="mcl",
             parameters=[PARAMS], output="screen"),
    ])
