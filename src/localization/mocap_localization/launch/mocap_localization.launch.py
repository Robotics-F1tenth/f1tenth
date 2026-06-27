from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mocap_localization')
    print(pkg_share, flush=True)
    params_file = os.path.join(pkg_share, 'params', 'params.yaml')

    mcl_params_file = os.path.join(get_package_share_directory('als_ros'), "params", "mcl_params.yaml")
    print(mcl_params_file, flush=True)
    map_publisher = Node(
        package="als_ros",
        executable="map_publisher",
        name="map_publisher",
        output="screen",
        parameters=[mcl_params_file],
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
            parameters=[params_file]
        )
    
    ld = LaunchDescription()
    ld.add_action(map_publisher)
    # ld.add_action(map_server)
    ld.add_action(localization)
    return ld