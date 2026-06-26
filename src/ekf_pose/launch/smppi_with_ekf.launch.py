"""
실차(Real Robot) 런치 파일
- EKF 노드 실행 (IMU + odom + mcl_pose 융합)
- SMPPI 는 /ekf_odom 단일 토픽으로 상태 수신
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ekf_pkg   = FindPackageShare("ekf_pose")
    smppi_pkg = FindPackageShare("smppi_cuda_controller")

    # ── 1. EKF Pose 노드 ──────────────────────────────────────────
    ekf_node = Node(
        package="ekf_pose",
        executable="ekf_pose_node",
        name="ekf_pose_estimator",
        output="screen",
        parameters=[
            PathJoinSubstitution([ekf_pkg, "config", "ekf_pose.yaml"]),
        ],
    )

    # ── 2. Path Publisher ─────────────────────────────────────────
    path_pub_node = Node(
        package="smppi_cuda_controller",
        executable="path_publisher",
        name="path_publisher",
        output="screen",
        parameters=[
            PathJoinSubstitution([smppi_pkg, "config", "path_publisher.yaml"]),
        ],
    )

    # ── 3. SMPPI Controller (실차) ────────────────────────────────
    smppi_node = Node(
        package="smppi_cuda_controller",
        executable="smppi_node",
        name="smppi_controller",
        output="screen",
        parameters=[
            PathJoinSubstitution([smppi_pkg, "config", "smppi_params.yaml"]),
            {
                # EKF 출력 단일 토픽 (pose + twist 일체)
                "odom_topic": "/ekf_odom",
            },
        ],
    )

    return LaunchDescription([
        ekf_node,
        path_pub_node,
        smppi_node,
    ])
