#!/usr/bin/env python3
# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
# Original: src/on_track_sys_id.py @ 7b2b712 (ROS1 / catkin)
#
# ROS 2 Humble (rclpy) 재작성. 바뀐 것은 ROS 인터페이스뿐이고,
# 데이터 수집 규약(50 Hz, v_x > 1 게이트, (timesteps,4) 롤링 버퍼, [v_x, v_y, omega, delta])과
# 학습 트리거 방식은 원본 그대로다.
#
# 원본 대비 ROS 계층 변경점:
#   rospy.init_node          -> rclpy.init + Node('pacejka_sysid')
#   rospy.get_param('~x')    -> declare_parameter (원본은 save_LUT_name / plot_model 에
#                               기본값이 없어 런치 인자를 빠뜨리면 크래시했다. 전부 기본값을 준다)
#   rospy.Subscriber x2      -> create_subscription x2~3 (state_source 에 따라)
#   rospy.Rate + 백그라운드   -> executor 를 별도 스레드에서 spin + 메인 스레드 50 Hz 샘플링
#                               (원본과 같은 스레딩 모델. loop() 주석 참고)
#   rospkg.get_path          -> paths.py (share 읽기 / output_dir 쓰기 분리)
#   rospy.signal_shutdown    -> 루프 탈출 후 rclpy.shutdown
#   input() 무조건 프롬프트   -> export_csv / prompt_before_export 파라미터
#
# v_x, v_y, omega 를 어디서 얻는지는 state_assembler.py 상단 주석 참고.

import csv
import os
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tqdm import tqdm

from . import paths
from . import ros_log
from .state_assembler import StateAssembler

# 원본과 동일하게 50 Hz 로 고정한다. helpers/generate_predictions.py 와
# helpers/train_model.py 가 dt = 0.02 를 상수로 갖고 있어 이 값과 반드시 짝이 맞아야 한다.
COLLECTION_RATE_HZ = 50


class PacejkaSysId(Node):
    def __init__(self):
        super().__init__('pacejka_sysid')
        ros_log.set_logger(self.get_logger())

        self._declare_parameters()

        self.racecar_version = self.get_parameter('racecar_version').value
        self.save_LUT_name = self.get_parameter('save_LUT_name').value
        self.plot_model = self.get_parameter('plot_model').value
        self.save_plots = self.get_parameter('save_plots').value
        self.generate_lut = self.get_parameter('generate_lut').value
        self.export_csv = self.get_parameter('export_csv').value
        self.prompt_before_export = self.get_parameter('prompt_before_export').value
        self.state_source = self.get_parameter('state_source').value
        self.min_collect_speed = self.get_parameter('min_collect_speed').value

        paths.set_output_root(self.get_parameter('output_dir').value)

        self.get_logger().info(f"Racecar_version: {self.racecar_version}")
        self._check_model_template()

        self.rate = COLLECTION_RATE_HZ
        self._last_bias_warn = 0.0
        self.load_parameters()
        self.setup_data_storage()

        self._setup_io()

    # ------------------------------------------------------------------ #
    # 파라미터
    # ------------------------------------------------------------------ #
    def _declare_parameters(self):
        # 원본 sys_id.launch 의 5개 인자에 대응
        self.declare_parameter('racecar_version', 'F1TENTH')
        self.declare_parameter('save_LUT_name', 'F1TENTH_on_track_pacejka')
        self.declare_parameter('plot_model', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ackermann_topic', '/ackermann_cmd')

        # 우리 워크스페이스용 추가 인터페이스
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('state_source', 'imu_odom')   # 'imu_odom' | 'odom'
        self.declare_parameter('output_dir', '~/.ros/pacejka_sysid')
        self.declare_parameter('generate_lut', False)
        self.declare_parameter('save_plots', True)
        self.declare_parameter('export_csv', True)
        self.declare_parameter('prompt_before_export', False)
        self.declare_parameter('min_collect_speed', 1.0)     # 원본 하드코딩 'v_x > 1'

        # IMU 기반 v_y 조립기 (state_source='imu_odom' 일 때만 사용)
        # 이 워크스페이스의 MicroStrain 센서는 축은 FLU 로 내보내지만 가속도 부호가
        # REP-145 표준(정지 시 a_z=+9.81)과 반대로 나온다 (실측: a_z=-9.81).
        # als_ros/mcl.launch.py:48 도 같은 /imu/data 에 대해 이미 180 을 기본값으로 쓴다.
        # 다른 하드웨어라면 다를 수 있으니 state_assembler.py 상단의 검증 절차로 확인할 것.
        self.declare_parameter('imu_rotation_roll', 180.0)   # deg
        self.declare_parameter('imu_rotation_pitch', 0.0)
        self.declare_parameter('imu_rotation_yaw', 0.0)
        self.declare_parameter('vy_washout_tau', 2.0)        # s
        self.declare_parameter('imu_to_cog_x', 0.0)          # m, 미실측이면 0
        self.declare_parameter('bias_calib_samples', 200)
        self.declare_parameter('standstill_speed', 0.05)     # m/s

    def _check_model_template(self):
        model_path = paths.model_file(self.racecar_version,
                                      self.racecar_version + '_pacejka')
        if not os.path.isfile(model_path):
            available = paths.available_models()
            raise RuntimeError(
                f"차량 파라미터 파일이 없습니다: {model_path}\n"
                f"  share 에 준비된 racecar_version: {available}\n"
                f"  models/<racecar_version>/<racecar_version>_pacejka.txt 를 만들고 "
                f"colcon build 후 다시 실행하세요.{paths._SYMLINK_HINT}")
        self.get_logger().info(f"차량 파라미터: {model_path}")

    # ------------------------------------------------------------------ #
    # 원본과 동일한 저장소/파라미터 로딩
    # ------------------------------------------------------------------ #
    def setup_data_storage(self):
        """
        Set up storage for collected data.

        Initializes data storage and related variables based on parameters loaded from 'nn_params'.
        """
        self.data_duration = self.nn_params['data_collection_duration']
        self.timesteps = self.data_duration * self.rate
        self.data = np.zeros((self.timesteps, 4))
        self.counter = 0
        self.current_state = np.zeros(4)

    def load_parameters(self):
        """
        This function loads parameters neural network parameters from 'params/nn_params.yaml' and stores them in self.nn_params.
        """
        yaml_file = paths.params_file('nn_params.yaml')
        with open(yaml_file, 'r') as file:
            self.nn_params = yaml.safe_load(file)

    # ------------------------------------------------------------------ #
    # 구독 설정
    # ------------------------------------------------------------------ #
    def _setup_io(self):
        odom_topic = self.get_parameter('odom_topic').value
        ackermann_topic = self.get_parameter('ackermann_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        self.create_subscription(AckermannDriveStamped, ackermann_topic,
                                 self.ackermann_cb, 10)

        if self.state_source == 'odom':
            # 원본과 동일한 경로: 단일 Odometry 에서 v_x, v_y, omega 를 모두 받는다.
            # 시뮬레이터(/odom0) 나 /ekf_odom 로 포팅 검증할 때 사용.
            self.assembler = None
            self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
            self.get_logger().info(
                f"state_source=odom  |  {odom_topic} (v_x,v_y,omega) + {ackermann_topic} (delta)")
        elif self.state_source == 'imu_odom':
            model = self._load_vehicle_params()
            self.assembler = StateAssembler(
                l_r=model['l_r'],
                imu_rotation_rpy=(
                    np.deg2rad(self.get_parameter('imu_rotation_roll').value),
                    np.deg2rad(self.get_parameter('imu_rotation_pitch').value),
                    np.deg2rad(self.get_parameter('imu_rotation_yaw').value)),
                washout_tau=self.get_parameter('vy_washout_tau').value,
                imu_to_cog_x=self.get_parameter('imu_to_cog_x').value,
                bias_calib_samples=self.get_parameter('bias_calib_samples').value,
                standstill_speed=self.get_parameter('standstill_speed').value,
                min_speed=self.min_collect_speed,
                logger=self.get_logger())
            self.create_subscription(Odometry, odom_topic,
                                     self.assembler_odom_cb, 10)
            self.create_subscription(Imu, imu_topic,
                                     self.assembler_imu_cb, qos_profile_sensor_data)
            self.get_logger().info(
                f"state_source=imu_odom  |  {odom_topic} (v_x) + {imu_topic} (omega,a_y) "
                f"+ {ackermann_topic} (delta)")
            self.get_logger().warn(
                "차를 정지시킨 채로 두면 IMU 바이어스 보정이 먼저 끝납니다. "
                "보정 완료 로그가 뜬 뒤에 주행을 시작하세요.")
        else:
            raise RuntimeError(
                f"state_source 는 'imu_odom' 또는 'odom' 이어야 합니다 (받은 값: {self.state_source})")

    def _load_vehicle_params(self):
        with open(paths.model_file(self.racecar_version,
                                   self.racecar_version + '_pacejka'), 'r') as f:
            return yaml.safe_load(f)

    # ------------------------------------------------------------------ #
    # 콜백
    # ------------------------------------------------------------------ #
    def odom_cb(self, msg):
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        self.current_state[2] = msg.twist.twist.angular.z

    def assembler_odom_cb(self, msg):
        self.assembler.update_odom(msg)

    def assembler_imu_cb(self, msg):
        self.assembler.update_imu(msg)

    def ackermann_cb(self, msg):
        self.current_state[3] = msg.drive.steering_angle

    # ------------------------------------------------------------------ #
    # 수집 루프 (원본 collect_data 와 동일)
    # ------------------------------------------------------------------ #
    def collect_data(self):
        """
        Collects data during simulation.

        Adds the current state to the data array and updates the counter.
        Closes the progress bar and logs a message if data collection is complete.
        """
        if self.assembler is not None:
            if not self.assembler.bias_ready:
                # 정지 구간을 놓치면 여기서 영원히 기다리게 되므로 주기적으로 상태를 알린다.
                self._warn_waiting_for_bias()
                return
            v_x, v_y, omega = self.assembler.get_state()
            self.current_state[0] = v_x
            self.current_state[1] = v_y
            self.current_state[2] = omega

        if self.current_state[0] > self.min_collect_speed:  # Only collect data when the car is moving
            self.data = np.roll(self.data, -1, axis=0)
            self.data[-1] = self.current_state
            self.counter += 1
            self.pbar.update(1)
        if self.counter == self.timesteps + 1:
            self.pbar.close()
            self.get_logger().info("Data collection completed.")

    def _warn_waiting_for_bias(self, period_s=5.0):
        now = time.monotonic()
        if now - self._last_bias_warn < period_s:
            return
        self._last_bias_warn = now
        got, need = self.assembler.calibration_progress()
        if self.assembler.imu_msg_count == 0:
            self.get_logger().warn(
                f"IMU 메시지가 아직 안 옵니다. "
                f"'{self.get_parameter('imu_topic').value}' 가 발행 중인지 확인하세요.")
        else:
            self.get_logger().warn(
                f"IMU 바이어스 보정 대기 중 ({got}/{need} 샘플). "
                f"차를 완전히 정지시키세요 — 현재 v_x={self.assembler.v_x:.2f} m/s "
                f"(정지 판정 기준 {self.assembler.standstill_speed} m/s). "
                f"IMU 수신 {self.assembler.imu_msg_count} 개.")

    def _report_data_quality(self):
        """수집된 데이터가 식별에 쓸 만한지 진단한다 (원본에는 없는 추가 점검).

        v_y 가 사실상 0 이면 NN 이 배울 잔차가 없어 타이어가 아니라
        운동학 자전거 모델을 재식별하게 된다 — 결과가 무의미해진다.
        """
        v_x, v_y, omega, delta = (self.data[:, i] for i in range(4))
        self.get_logger().info(
            f"수집 요약  v_x[{v_x.min():.2f}, {v_x.max():.2f}] m/s | "
            f"|v_y|max={np.abs(v_y).max():.3f} m/s (std {v_y.std():.3f}) | "
            f"|omega|max={np.abs(omega).max():.2f} rad/s | "
            f"|delta|max={np.abs(delta).max():.3f} rad")
        if np.abs(v_y).max() < 1e-6:
            self.get_logger().error(
                "v_y 가 항상 0 입니다. 식별 결과는 신뢰할 수 없습니다. "
                "state_source / imu_topic 배선을 확인하세요.")
        if np.abs(delta).max() < 0.05:
            self.get_logger().warn(
                "조향각 범위가 너무 좁습니다. 슬립각 커버리지가 부족해 "
                "피팅이 잘 되지 않습니다. 더 공격적으로 주행하세요.")

    def export_data_as_csv(self):
        """
        Export collected data as a CSV file.

        원본은 무조건 stdin 프롬프트를 띄웠다. 비대화형 실행이 가능하도록
        export_csv / prompt_before_export 파라미터로 감쌌다.
        """
        if not self.export_csv:
            return
        if self.prompt_before_export:
            user_input = input("\033[33m[WARN] Press 'Y' and then ENTER to export data as CSV, or press ENTER to continue without dumping: \033[0m")
            if user_input.lower() != 'y':
                return

        data_dir = paths.data_dir(self.racecar_version)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = os.path.join(
            data_dir, f'{self.racecar_version}_sys_id_data_{timestamp}.csv')

        # Write data to CSV file
        with open(csv_file, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['v_x', 'v_y', 'omega', 'delta'])
            for row in self.data:
                writer.writerow(row)  # Each row contains v_x, v_y, omega, delta
        self.get_logger().info(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def run_nn_train(self):
        """
        Initiates training of the neural network using collected data.
        """
        # matplotlib 백엔드는 pyplot 을 import 하기 전에 정해야 하므로 지연 임포트한다.
        import matplotlib
        if not self.plot_model:
            matplotlib.use('Agg')
        elif not os.environ.get('DISPLAY'):
            self.get_logger().warn(
                "plot_model=True 인데 DISPLAY 가 없습니다. Agg 로 전환합니다 "
                "(save_plots=True 면 파일로 저장됩니다).")
            matplotlib.use('Agg')
        from .helpers.train_model import nn_train

        self.get_logger().info("Training neural network...")
        C_Pf, C_Pr = nn_train(self.data, self.racecar_version, self.save_LUT_name,
                              self.plot_model, generate_lut=self.generate_lut,
                              save_plots=self.save_plots)
        self.get_logger().info(f"식별 완료  C_Pf(B,C,D,E) = {C_Pf}")
        self.get_logger().info(f"식별 완료  C_Pr(B,C,D,E) = {C_Pr}")

    # ------------------------------------------------------------------ #
    def loop(self):
        """
        Main loop for data collection, training, and exporting.

        원본의 스레딩 모델을 그대로 재현한다: 콜백은 백그라운드에서 돌고,
        메인 스레드는 rospy.Rate 처럼 50 Hz 로 현재 상태를 샘플링한다.

        주의 — rclpy.spin_once() 는 호출당 **콜백 하나만** 처리한다.
        메인 루프에서 spin_once 를 부르는 방식으로는 50 Hz 루프가 100 Hz IMU 를
        따라잡지 못해 메시지를 흘리고, v_y 적분이 망가진다(실측: 4초에 400개 중 124개만 처리).
        그래서 executor 를 별도 스레드에서 돌린다.
        """
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        try:
            self.pbar = tqdm(total=self.timesteps, desc='Collecting data', ascii=True)
            period = 1.0 / self.rate
            next_t = time.monotonic() + period

            while rclpy.ok() and self.counter != self.timesteps + 1:
                now = time.monotonic()
                if now < next_t:
                    time.sleep(next_t - now)
                self.collect_data()
                next_t += period
                if next_t < time.monotonic():   # 처리가 밀렸으면 재동기화
                    next_t = time.monotonic() + period
        finally:
            # 학습 중에는 콜백이 상태를 건드리지 않도록 먼저 멈춘다.
            executor.shutdown()
            spin_thread.join(timeout=2.0)

        if self.counter == self.timesteps + 1:
            self._report_data_quality()
            self.run_nn_train()
            self.export_data_as_csv()
            self.get_logger().info("Training completed. Shutting down...")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PacejkaSysId()
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
