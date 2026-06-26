from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ekf_pkg   = FindPackageShare("ekf_pose")
    smppi_pkg = FindPackageShare("smppi_cuda_controller")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false")

    # ── 1. EKF Pose 노드 ──────────────────────────────────────────
    ekf_node = Node(
        package="ekf_pose",
        executable="ekf_pose_node",
        name="ekf_pose_estimator",
        output="screen",
        parameters=[
            PathJoinSubstitution([ekf_pkg, "config", "ekf_pose.yaml"]),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
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

    # ── 3. SMPPI Controller ───────────────────────────────────────
    # odom_topic 하나만 /ekf_odom 으로 지정 — pose+velocity 전부 EKF에서 옴
    smppi_node = Node(
        package="smppi_cuda_controller",
        executable="smppi_node",
        name="smppi_controller",
        output="screen",
        parameters=[
            PathJoinSubstitution([smppi_pkg, "config", "smppi_params.yaml"]),
            {
                "odom_topic":   "/ekf_odom",   # 단일 토픽: pose + twist 일체
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
        path_pub_node,
        smppi_node,
    ])