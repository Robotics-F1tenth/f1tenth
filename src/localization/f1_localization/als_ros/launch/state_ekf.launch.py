from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():

     ld = LaunchDescription()

     # Get the share directory for the racecar_simulator package
     pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

     # Define the path to the rviz configuration file
     mcl_params_file = os.path.join(pkg_dir, "params", "mcl_params.yaml")
     print(mcl_params_file)
     

     
     state_ekf = Node(
        package="als_ros",
        executable="state_ekf",
        name="state_ekf",
        output="screen",
        parameters=[mcl_params_file],
     )

     

     ld.add_action(state_ekf)
    
     return ld