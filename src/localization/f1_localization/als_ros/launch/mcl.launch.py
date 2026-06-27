from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

from launch_testing_ros import WaitForTopics


def generate_launch_description():

    ld = LaunchDescription()

    # Get the share directory for the racecar_simulator package
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # Define the path to the rviz configuration file
    mcl_params_file = os.path.join(pkg_dir, "params", "mcl_params.yaml")
    map_server_params_file = os.path.join(pkg_dir, "params", "map.yaml")
    lifecycle_params_file = os.path.join(pkg_dir, "params", "lifecycle.yaml")
    print(mcl_params_file)

    mcl_node = Node(
        package="als_ros",
        executable="mcl",
        name="mcl",
        output="screen",
        parameters=[mcl_params_file],
    )

    # map_publisher = Node(
    #     package="als_ros",
    #     executable="map_publisher",
    #     name="map_publisher",
    #     output="screen",
    #     parameters=[mcl_params_file],
    # )

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[map_server_params_file]
        )
    
    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[lifecycle_params_file]
        )


    ld.add_action(map_server)
    ld.add_action(lifecycle_manager)
    ld.add_action(mcl_node)

    return ld
