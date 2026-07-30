from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mocap_localization')
    params_file = os.path.join(pkg_share, 'params', 'params2.yaml')

    declare_with_map = DeclareLaunchArgument(
        'with_map',
        default_value='true',
        description='Publish /map from this launch.',
    )

    localization2 = Node(
            package='mocap_localization',
            executable='mocap_localization',
            name='mocap_localization2',
            output='screen',
            parameters=[params_file],
        )

    return LaunchDescription([declare_with_map, localization2])
