# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
# Original: launch/sys_id.launch @ 7b2b712 (ROS1 XML) -> ROS 2 Humble Python launch
#
# 원본 5개 인자(odom_topic, ackermann_cmd_topic, save_LUT_name, plot_model, racecar_version)를
# 모두 그대로 두고, 우리 워크스페이스용 인자(state_source, imu_topic, generate_lut)를 더했다.
# 기본값은 params/sysid_node.yaml 에서 오고, 여기서 넘긴 인자가 그 위에 덮인다.
#
# smppi_cuda_controller/launch/cuda_mppi.launch.py 와 달리 모드 전환을 파이썬 리터럴이 아니라
# 진짜 launch 인자로 노출한다 (재빌드 없이 바꿀 수 있어야 하므로).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory('pacejka_sysid'), 'params', 'sysid_node.yaml')

    args = [
        DeclareLaunchArgument('param_file', default_value=default_param_file,
                              description='sysid 노드 파라미터 YAML 경로'),
        DeclareLaunchArgument('racecar_version', default_value='F1TENTH',
                              description='models/<이름>/<이름>_pacejka.txt 의 이름 (밑줄 금지)'),
        DeclareLaunchArgument('save_LUT_name', default_value='F1TENTH_on_track_pacejka',
                              description='generate_lut=true 일 때 저장될 LUT 이름'),
        DeclareLaunchArgument('plot_model', default_value='true',
                              description='이터레이션마다 식별 결과 그림 표시'),
        DeclareLaunchArgument('generate_lut', default_value='false',
                              description='조향 LUT 생성 (ForzaETH MAP 컨트롤러 전용)'),
        DeclareLaunchArgument('state_source', default_value='imu_odom',
                              description="'imu_odom' (실차) 또는 'odom' (시뮬/ekf_odom 검증)"),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='v_x 출처. state_source=odom 이면 v_y/omega 도 여기서'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data',
                              description='omega, a_y 출처 (state_source=imu_odom 일 때)'),
        DeclareLaunchArgument('ackermann_topic', default_value='/ackermann_cmd',
                              description='조향각 delta 출처'),
    ]

    overrides = {
        'racecar_version': LaunchConfiguration('racecar_version'),
        'save_LUT_name': LaunchConfiguration('save_LUT_name'),
        'plot_model': ParameterValue(LaunchConfiguration('plot_model'), value_type=bool),
        'generate_lut': ParameterValue(LaunchConfiguration('generate_lut'), value_type=bool),
        'state_source': LaunchConfiguration('state_source'),
        'odom_topic': LaunchConfiguration('odom_topic'),
        'imu_topic': LaunchConfiguration('imu_topic'),
        'ackermann_topic': LaunchConfiguration('ackermann_topic'),
    }

    sysid_node = Node(
        package='pacejka_sysid',
        executable='pacejka_sysid_node',
        name='pacejka_sysid',
        output='screen',
        emulate_tty=True,          # tqdm 진행바가 보이도록
        parameters=[LaunchConfiguration('param_file'), overrides],
    )

    return LaunchDescription(args + [sysid_node])
