# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
#
# 신규 파일 — 원본에는 대응물이 없다.
#
# 원본 노드는 `/car_state/odom` 하나에서 (v_x, v_y, omega) 를 전부 받는다.
# 우리 f1tenth_ws 에는 그런 토픽이 없다:
#   - vesc_to_odom.cpp:157  /odom 의 twist.linear.y = 0.0 하드코딩 (횡속도 센서 없음)
#   - vesc_to_odom.cpp:110  /odom 의 angular.z 는 측정이 아니라 v*tan(delta)/L 운동학 재구성
#   - ekf_pose_estimator.cpp:295,306  /ekf_odom 은 위의 "항상 0인 vy" 를 측정값으로 써서
#                                     추정 vy 를 0 으로 끌어내린다
# 따라서 v_y 를 IMU 로부터 유도한다 (사용자 결정).
#
#   v_x  <- /odom      twist.twist.linear.x
#   w    <- /imu/data  angular_velocity      (IMU 회전 보정 후 z)
#   a_y  <- /imu/data  linear_acceleration   (IMU 회전 보정 후 y)
#
#   바디 프레임 횡방향 운동방정식:  v_y_dot = a_y - v_x * w
#   드리프트를 막기 위해 워시아웃(고역통과) 항을 더한다: -v_y / tau
#
# 목표 좌표계는 이 패키지(원본 ForzaETH 코드)의 슬립각 공식이 가정하는
# ISO 8855 / FLU 차량축이다: x=전방, y=좌측, z=상방, +omega=좌회전(위에서 반시계),
# +delta=좌측 조향, +v_y=좌측 이동, 정지 시 a_z=+9.81 (REP-145 표준 규약 —
# 가속도계는 중력이 아니라 중력에 저항하는 반력을 잰다). (helpers/generate_predictions.py,
# helpers/solve_pacejka.py 의 alpha_f = -atan((v_y+omega*l_f)/v_x)+delta 참고)
#
# imu_rotation_roll/pitch/yaw 는 실제 IMU 출력을 이 목표 좌표계로 맞추는 보정값이다.
# "축이 FLU 방향을 향한다"는 것과 "가속도 부호가 REP-145 표준을 따른다"는 것은
# 별개다 — 드라이버가 축은 FLU 로 내보내면서 가속도 부호는 중력 벡터 방향 그대로
# (반력이 아니라) 낼 수 있다. 이 워크스페이스의 MicroStrain 센서가 그런 경우다:
# 정지 시 a_z 실측값이 -9.81 이었다 (als_ros/MCL.h:509-514 의 같은 토픽에 대한
# 관찰과 일치, mcl.launch.py:48 도 이미 180 을 기본값으로 씀). 그래서 기본값을
# roll=180 으로 둔다 — (x,y,z) -> (x,-y,-z) 로 뒤집어야 a_z 가 +9.81 로,
# omega/a_y 부호가 목표 좌표계와 맞게 나온다.
#
# ⚠ 그래도 다른 하드웨어/장착 상태에서는 다를 수 있으니 맹신하지 말고 확인할 것
# (README "문제 해결" 참고):
#   1. 차를 세워 둔 상태에서 `ros2 topic echo /imu/data/linear_acceleration --field z`
#      → +9.81 근처면 roll=0, -9.81 근처면 roll=180 이 맞다.
#   2. 좌회전(delta>0) 중 조립된 omega 가 양수로 나오는지.

import math

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """tf2 의 setRPY 와 동일한 ZYX 합성: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


class StateAssembler:
    """/odom 의 v_x 와 /imu/data 의 (w, a_y) 로 (v_x, v_y, w) 를 조립한다.

    출력 v_y 는 **후륜축 기준**이다. 원본 train_model.filter_data() 가
    non-SIM 인 경우 `v_y += l_r * omega` 로 CoG 기준으로 되돌리는 것을 전제로
    작성되어 있고(ForzaETH 의 odom vy 가 후륜축 기준이라는 하드웨어 전제),
    그 줄을 수정하지 않기 위해 여기서 규약을 맞춰 준다.
    """

    def __init__(self, l_r, imu_rotation_rpy=(math.pi, 0.0, 0.0),
                 washout_tau=2.0, imu_to_cog_x=0.0, bias_calib_samples=200,
                 standstill_speed=0.05, min_speed=1.0, logger=None):
        self.l_r = float(l_r)
        self.rot = rpy_to_matrix(*imu_rotation_rpy)
        self.washout_tau = float(washout_tau)
        self.imu_to_cog_x = float(imu_to_cog_x)
        self.bias_calib_samples = int(bias_calib_samples)
        self.standstill_speed = float(standstill_speed)
        self.min_speed = float(min_speed)
        self.logger = logger

        # 관측값
        self.v_x = 0.0
        self.omega = 0.0
        self.a_y = 0.0

        # 적분 상태
        self.v_y_cog = 0.0
        self._last_imu_t = None
        self._last_omega = 0.0

        # 바이어스 추정 (정지 상태에서 평균)
        self._bias_samples_ay = []
        self._bias_samples_wz = []
        self.bias_ay = 0.0
        self.bias_wz = 0.0
        self.bias_ready = False

        # 진단용
        self.max_abs_v_y = 0.0
        self.imu_msg_count = 0

    # ------------------------------------------------------------------ #
    # 콜백에서 호출
    # ------------------------------------------------------------------ #
    def update_odom(self, msg):
        self.v_x = msg.twist.twist.linear.x

    def update_imu(self, msg):
        acc = self.rot @ np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])
        gyro = self.rot @ np.array([msg.angular_velocity.x,
                                    msg.angular_velocity.y,
                                    msg.angular_velocity.z])
        self.a_y = float(acc[1])
        self.omega = float(gyro[2])
        self.imu_msg_count += 1

        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        if self._last_imu_t is None:
            self._last_imu_t = t
            self._last_omega = self.omega
            return

        dt = t - self._last_imu_t
        self._last_imu_t = t
        if dt <= 0.0 or dt > 0.5:      # 시계 역행 / 큰 공백은 버린다
            self._last_omega = self.omega
            return

        # ── 정지 상태 바이어스 추정 ──────────────────────────────────
        if not self.bias_ready:
            if abs(self.v_x) < self.standstill_speed:
                self._bias_samples_ay.append(self.a_y)
                self._bias_samples_wz.append(self.omega)
                if len(self._bias_samples_ay) >= self.bias_calib_samples:
                    self.bias_ay = float(np.mean(self._bias_samples_ay))
                    self.bias_wz = float(np.mean(self._bias_samples_wz))
                    self.bias_ready = True
                    self._log(f"IMU 바이어스 보정 완료: a_y={self.bias_ay:+.4f} m/s^2, "
                              f"w_z={self.bias_wz:+.5f} rad/s "
                              f"({len(self._bias_samples_ay)} 샘플)")
            self._last_omega = self.omega
            return

        a_y = self.a_y - self.bias_ay
        omega = self.omega - self.bias_wz

        # ── IMU 장착 위치 -> CoG 보정 (기본 0.0 이면 무시) ────────────
        if self.imu_to_cog_x != 0.0:
            omega_dot = (omega - self._last_omega) / dt
            a_y = a_y - omega_dot * self.imu_to_cog_x
        self._last_omega = omega

        # ── v_y 적분 (워시아웃 포함) ──────────────────────────────────
        if abs(self.v_x) < self.min_speed:
            # 원본이 v_x > 1 에서만 데이터를 모으므로, 그 아래에서는 리셋해
            # 정지/저속 구간의 적분 드리프트가 다음 주행 구간으로 새지 않게 한다.
            self.v_y_cog = 0.0
        else:
            v_y_dot = a_y - self.v_x * omega - self.v_y_cog / self.washout_tau
            self.v_y_cog += v_y_dot * dt
            self.max_abs_v_y = max(self.max_abs_v_y, abs(self.v_y_cog))

    # ------------------------------------------------------------------ #
    def calibration_progress(self):
        """(모은 샘플 수, 목표 샘플 수) — 노드가 대기 상태를 안내하는 데 쓴다."""
        return len(self._bias_samples_ay), self.bias_calib_samples

    def get_state(self):
        """원본 파이프라인이 기대하는 (v_x, v_y_rear_axle, omega) 를 돌려준다."""
        omega = self.omega - self.bias_wz if self.bias_ready else self.omega
        v_y_rear = self.v_y_cog - self.l_r * omega
        return self.v_x, v_y_rear, omega

    def _log(self, msg):
        if self.logger is not None:
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")
