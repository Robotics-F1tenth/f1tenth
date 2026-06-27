import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    slam_params = os.path.join(
        get_package_share_directory('als_ros'), 'params', 'slam_params.yaml'
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
    )

    # 서비스 제공 노드 (필수)
    map_saver_server = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )


    # 5초마다 자동 저장 (당신이 만든 파이썬 노드)
    autosaver = Node(
        package="als_ros",        
        executable="map_autosaver.py",
        name="map_autosaver",
        output="screen",
        parameters=[
            {"save_dir": os.path.expanduser("~/capstone_ws/src/localization/f1_localization/als_ros/maps/slam_map/")},
            {"map_topic": "map"},
            {"period_sec": 5.0},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_saver",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_saver"]},
        ],
    )

    return LaunchDescription([
        slam,
        map_saver_server,
        autosaver,
        lifecycle_manager
    ])
