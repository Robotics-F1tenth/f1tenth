"""
실차(Real Robot) 런치 파일
- EKF 노드 실행 (IMU + odom + mcl_pose 융합)
- SMPPI 는 /ekf_odom 단일 토픽으로 상태 수신

2026-08-03 수정 — 이 파일은 그동안 실행 불가능한 상태였다:
  1) 존재하지 않는 `smppi_params.yaml`, `path_publisher.yaml` 을 참조 (실제는 `params.yaml`)
  2) 실행파일 이름이 `ekf_pose_node` 로 되어 있었으나 실제 타깃은 `ekf_pose_estimator`
  3) `use_mcl_pose:false` 를 설정하지 않아 smppi_node 가 기본값 true 로 뜨고,
     정성껏 지정한 `odom_topic:/ekf_odom` 을 **무시**한 채 /mcl_pose 를 구독했다
따라서 "EKF 를 거쳐 상태를 받는다"는 경로는 실제로 한 번도 동작한 적이 없다.
실차 기본 경로는 smppi_cuda_controller/launch/cuda_mppi.launch.py (MCL 직접 구독).
자세한 배경: ekf_pose/docs/ekf-pose-analysis-2026-08.md
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ekf_share = get_package_share_directory("ekf_pose")
    smppi_share = get_package_share_directory("smppi_cuda_controller")

    map_name = "map1"
    centerline_csv = os.path.join(
        smppi_share, "data", map_name, f"{map_name}_centerline.csv")

    ekf_params = os.path.join(ekf_share, "config", "ekf_pose.yaml")
    smppi_params = os.path.join(smppi_share, "config", "params.yaml")

    # ── 1. EKF Pose 노드 ──────────────────────────────────────────
    ekf_node = Node(
        package="ekf_pose",
        executable="ekf_pose_estimator",
        name="ekf_pose_node",
        output="screen",
        parameters=[ekf_params],
    )

    # ── 2. Path Publisher ─────────────────────────────────────────
    path_pub_node = Node(
        package="smppi_cuda_controller",
        executable="path_publisher",
        name="path_publisher",
        output="screen",
        parameters=[smppi_params, {
            "csv_file_path": centerline_csv,
            "frame_id": "map",
            "publish_rate": 1.0,
        }],
    )

    # ── 3. SMPPI Controller (실차) ────────────────────────────────
    #  use_mcl_pose:false 여야 odom_topic 단일 구독 경로를 탄다.
    #  (true 면 pose_topic/velocity_topic 을 따로 구독하고 odom_topic 은 무시된다)
    smppi_node = Node(
        package="smppi_cuda_controller",
        executable="smppi_node",
        name="smppi_controller",
        output="screen",
        parameters=[smppi_params, {
            "use_mcl_pose": False,
            "odom_topic": "/ekf_odom",      # EKF 출력 단일 토픽 (pose + twist 일체)
            "drive_topic": "/drive",
            "num_samples": 10000,
            "visualize_candidates": False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value=map_name),
        ekf_node,
        path_pub_node,
        smppi_node,
    ])
