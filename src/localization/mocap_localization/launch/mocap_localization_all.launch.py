from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mocap_localization")

    params1_file = LaunchConfiguration("params1_file")
    params2_file = LaunchConfiguration("params2_file")
    body1_name = LaunchConfiguration("body1_name")
    body2_name = LaunchConfiguration("body2_name")
    pose1_topic = LaunchConfiguration("pose1_topic")
    pose2_topic = LaunchConfiguration("pose2_topic")
    diagnostic_logging = LaunchConfiguration("diagnostic_logging")

    common_arguments = [
        DeclareLaunchArgument(
            "params1_file",
            default_value=os.path.join(pkg_share, "params", "params.yaml"),
            description="Vehicle-1 calibration parameter file.",
        ),
        DeclareLaunchArgument(
            "params2_file",
            default_value=os.path.join(pkg_share, "params", "params2.yaml"),
            description="Vehicle-2 calibration parameter file.",
        ),
        DeclareLaunchArgument(
            "body1_name",
            default_value="6",
            description="Motive rigid-body name for vehicle 1.",
        ),
        DeclareLaunchArgument(
            "body2_name",
            default_value="7",
            description="Motive rigid-body name for vehicle 2.",
        ),
        DeclareLaunchArgument("pose1_topic", default_value="/mocap_pose1"),
        DeclareLaunchArgument("pose2_topic", default_value="/mocap_pose2"),
        DeclareLaunchArgument(
            "diagnostic_logging",
            default_value="false",
            description="Enable one throttled yaw/measurement-age INFO line per vehicle.",
        ),
    ]

    localization1 = Node(
        package="mocap_localization",
        executable="mocap_localization",
        namespace="mocap1",
        name="mocap_localization_1",
        output="screen",
        parameters=[
            params1_file,
            {
                "rigid_body_name": ParameterValue(body1_name, value_type=str),
                "output_topic": ParameterValue(pose1_topic, value_type=str),
                "publish_tf": False,
                "diagnostic_logging": ParameterValue(
                    diagnostic_logging, value_type=bool),
            },
        ],
    )

    localization2 = Node(
        package="mocap_localization",
        executable="mocap_localization",
        namespace="mocap2",
        name="mocap_localization_2",
        output="screen",
        parameters=[
            params2_file,
            {
                "rigid_body_name": ParameterValue(body2_name, value_type=str),
                "output_topic": ParameterValue(pose2_topic, value_type=str),
                "publish_tf": False,
                "diagnostic_logging": ParameterValue(
                    diagnostic_logging, value_type=bool),
            },
        ],
    )

    return LaunchDescription(common_arguments + [localization1, localization2])
