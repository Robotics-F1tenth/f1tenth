"""
시뮬레이터 런치 파일
- EKF 노드 없음 (시뮬레이터가 완벽한 ground truth 제공)
- SMPPI 는 시뮬레이터 odom 토픽을 직접 수신
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    smppi_pkg = FindPackageShare("smppi_cuda_controller")

    sim_odom_topic_arg = DeclareLaunchArgument(
        "sim_odom_topic",
        default_value="/odom0",
        description="시뮬레이터 odom 토픽 (예: /odom0, /ego_racecar/odom 등)",
    )

    # ── 1. Path Publisher ─────────────────────────────────────────
    path_pub_node = Node(
        package="smppi_cuda_controller",
        executable="path_publisher",
        name="path_publisher",
        output="screen",
        parameters=[
            PathJoinSubstitution([smppi_pkg, "config", "path_publisher.yaml"]),
        ],
    )

    # ── 2. SMPPI Controller (시뮬레이터) ──────────────────────────
    smppi_node = Node(
        package="smppi_cuda_controller",
        executable="smppi_node",
        name="smppi_controller",
        output="screen",
        parameters=[
            PathJoinSubstitution([smppi_pkg, "config", "smppi_params.yaml"]),
            {
                # 시뮬레이터 odom 토픽으로 오버라이드
                "odom_topic": LaunchConfiguration("sim_odom_topic"),
            },
        ],
    )

    return LaunchDescription([
        sim_odom_topic_arg,
        path_pub_node,
        smppi_node,
    ])
