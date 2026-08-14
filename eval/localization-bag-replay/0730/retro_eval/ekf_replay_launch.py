import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ekf_share = get_package_share_directory("ekf_pose")
    base_params = os.path.join(ekf_share, "config", "ekf_pose.yaml")
    retro = os.environ.get("RETRO", "true") == "true"

    ekf_node = Node(
        package="ekf_pose", executable="ekf_pose_estimator", name="ekf_pose_node",
        output="screen",
        parameters=[base_params, {
            "use_sim_time": True,
            "retrodiction_enabled": retro,
            "publish_tf": False,
        }],
    )
    return LaunchDescription([ekf_node])
