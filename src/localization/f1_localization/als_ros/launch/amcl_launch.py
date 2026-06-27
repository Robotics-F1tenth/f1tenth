from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    cfg = os.getenv("LOC_CFG_DIR", "/home/a/f1tenth_ws/src/localization/f1_localization/navigation2/nav2_bringup/params")

    return LaunchDescription([
        # EKF (/odom 생성)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(cfg, 'ekf.yaml')]
        ),
        # map_server (Lifecycle)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[os.path.join(cfg, 'map.yaml')]
        ),
        # AMCL (Lifecycle)
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(cfg, 'amcl.yaml')]
        ),
        # Lifecycle Manager (map_server, amcl 활성화)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[os.path.join(cfg, 'lifecycle_manager_localization.yaml')]
        ),
        # (예시) 정적 TF: base_link->laser_frame (실제 장착값으로 교체)
        # Node(package='tf2_ros', executable='static_transform_publisher',
        #      arguments=['0','0','0.20','0','0','0','base_link','laser_frame']),
    ])
