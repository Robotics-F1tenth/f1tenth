"""
ekf_evaluate.launch.py
======================
ros2 bag 재생 + EKF 노드 + 평가 노드를 한 번에 실행합니다.

사용법:
  ros2 launch ekf_pose ekf_evaluate.launch.py \
      bag_path:=/home/user/bags/rosbag2_2026_03_13-21_52_51 \
      gt_topic:=/mocap_pose1

인자:
  bag_path    bag 디렉터리 경로 (필수)
  gt_topic    GT mocap 토픽 (default: /mocap_pose1)
  output_dir  평가 결과 저장 경로 (default: /tmp/ekf_eval)
  sync_slop   시간 동기화 허용 오차 [s] (default: 0.05)
  rate        bag 재생 속도 배율 (default: 1.0)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_pkg = FindPackageShare("ekf_pose")

    # ── 인자 선언 ──────────────────────────────────────────────────
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        description="ros2 bag 디렉터리 경로 (필수)",
    )
    gt_topic_arg = DeclareLaunchArgument(
        "gt_topic", default_value="/mocap_pose1",
        description="GT mocap 토픽 이름",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir", default_value="/tmp/ekf_eval",
        description="평가 결과(CSV/PNG) 저장 경로",
    )
    sync_slop_arg = DeclareLaunchArgument(
        "sync_slop", default_value="0.05",
        description="ApproximateTimeSynchronizer slop [s]",
    )
    rate_arg = DeclareLaunchArgument(
        "rate", default_value="1.0",
        description="bag 재생 속도 배율",
    )

    # ── EKF 노드 (use_sim_time=true) ───────────────────────────────
    ekf_node = Node(
        package="ekf_pose",
        executable="ekf_pose_estimator",
        name="ekf_pose_estimator",
        output="screen",
        parameters=[
            PathJoinSubstitution([ekf_pkg, "config", "ekf_pose.yaml"]),
            {"use_sim_time": True},
        ],
    )

    # ── 평가 노드 ──────────────────────────────────────────────────
    evaluator_node = Node(
        package="ekf_pose",
        executable="ekf_evaluator",
        name="ekf_evaluator",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "ekf_topic":   "/ekf_pose",
            "gt_topic":    LaunchConfiguration("gt_topic"),
            "output_dir":  LaunchConfiguration("output_dir"),
            "sync_slop":   LaunchConfiguration("sync_slop"),
        }],
    )

    # ── bag 재생 (노드 기동 후 2초 지연) ──────────────────────────
    bag_play = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "play",
                    LaunchConfiguration("bag_path"),
                    "--clock",
                    "--rate", LaunchConfiguration("rate"),
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        bag_path_arg,
        gt_topic_arg,
        output_dir_arg,
        sync_slop_arg,
        rate_arg,
        ekf_node,
        evaluator_node,
        bag_play,
    ])
