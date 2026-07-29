from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mocap_localization')
    print(pkg_share, flush=True)
    params_file = os.path.join(pkg_share, 'params', 'params.yaml')

    # This launch is self-contained by default (map_publisher included) so RViz has a map
    # when mocap runs alone. mcl.launch.py brings its own map via nav2 map_server, so pass
    # `with_map:=false` when running both -- otherwise /map gets two latched publishers.
    with_map = LaunchConfiguration("with_map")
    declare_with_map = DeclareLaunchArgument(
        "with_map",
        default_value="true",
        description="Publish /map from this launch. Set false when mcl.launch.py already provides it.",
    )

    mcl_params_file = os.path.join(get_package_share_directory('als_ros'), "params", "mcl_params.yaml")
    print(mcl_params_file, flush=True)
    map_publisher = Node(
        package="als_ros",
        executable="map_publisher",
        name="map_publisher",
        output="screen",
        parameters=[mcl_params_file],
        condition=IfCondition(with_map),
    )
    # map_server_params_file = os.path.join(get_package_share_directory('als_ros'), "params", "map.yaml")
    # map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[map_server_params_file]
    # )
    
    localization = Node(
        package='mocap_localization',
        executable='mocap_localization',
        name='mocap_localization',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/mocap_pose', '/mocap_pose1')
        ]   
    )
    ld = LaunchDescription()
    ld.add_action(declare_with_map)
    ld.add_action(map_publisher)
    # ld.add_action(map_server)
    ld.add_action(localization)
    return ld