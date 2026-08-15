# IMU 축 보정 — 3호차 정상화 완료 / 4호차 미변경 (2026-08-15)

3호차의 IMU 축을 펌웨어 identity + ROS 드라이버 보정으로 정상화한 작업 기록이다.
**4호차는 이번 작업에서 실행·설정 변경을 전혀 하지 않았다.**

관련 문서: `ekf-pose-analysis-2026-08.md`, `handoff-control-2026-08-04.md`.

## 0. 현재 상태

| 차량 | 펌웨어 `sensor2vehicle` | ROS 드라이버 보정 | 검증 상태 |
|---|---|---|---|
| 3호차 | Euler `[0, 0, 0]` identity, NVM 저장 | `imu_data_roll180: true` | 2026-08-15 실차 `imu_frame_test` 통과 |
| 4호차 | identity (기존 조사 기록) | `pacejka_sysid`의 `imu_rotation_roll=180` (기존 상태) | **이번 작업에서 미변경** |

3호차의 현재 `/imu/data`는 차체 FLU 기준이다. 따라서 앞으로 3호차에서
`ekf_pose`를 실행할 때 `imu_axis_yaw180`은 `false`여야 한다.

```text
정지 a_z: +9.78 m/s²
자이로 z: 기울기 +0.85, 상관 +0.90
횡가속 a_y: 기울기 +1.04, 상관 +0.96
종가속 a_x: 기울기 +0.70, 상관 +0.70
```

위 값은 `/odom` 기준 40초 8자 주행에서 얻었다. 세 축 모두 부호가 정상이다.

## 1. 3호차에서 수행한 변경

### 1.1 펌웨어 설정을 identity로 통일

기존 3호차는 `filter_sensor2vehicle_frame_transformation_euler=[π,0,π]`가
설정된 상태였다. 이를 다음 차량 전용 YAML로 `[0,0,0]`으로 설정하고,
`save_settings: true`로 장치 NVM에 저장했다.

- `src/driver/new_imu/microstrain_inertial_driver/config/f1tenth_car3_imu_xy_corrected.yml`

```yaml
filter_sensor2vehicle_frame_selector: 1
filter_sensor2vehicle_frame_transformation_euler: [0.0, 0.0, 0.0]
save_settings: true
```

실행 중에는 다음으로 확인한다.

```bash
ros2 param get /microstrain_inertial_driver \
  filter_sensor2vehicle_frame_transformation_euler
# Double values are: array('d', [0.0, 0.0, 0.0])
```

### 1.2 ROS publish 경계에서 roll 180° 보정

Microstrain의 `/imu/data`는 센서 DeltaTheta/DeltaVelocity를 드라이버가 직접
publish하므로, `filter_sensor2vehicle`만 바꿔서는 원하는 `/imu/data` 축 보정을
보장하지 못한다. 따라서 `new_imu` 드라이버에 publisher-side 보정 파라미터를 추가했다.

3호차 YAML은 다음만 활성화한다.

```yaml
imu_data_roll180: true
```

이는 일반 ENU 변환 뒤에 적용되는 `Rx(π)` 보정이다. x는 유지하고 y/z를 반전하며,
가속도·자이로·자력계와 `/imu/data` orientation에 같은 보정을 적용한다.

- 구현: `microstrain_inertial_driver_common/src/publishers.cpp`
- 파라미터 읽기: `microstrain_inertial_driver_common/src/config.cpp`
- 기본값: `microstrain_inertial_driver_common/config/params.yml`

처음에는 `Rz(π)` (`imu_data_yaw180`)를 시도했지만, identity 펌웨어 상태에서
`imu_frame_test`가 `a_x`, `gyro_z` 반전 / `a_y` 정상으로 판정했다. 이를 근거로
최종 보정은 `Rx(π)`로 확정했다. `imu_data_yaw180`은 기본값 `false`이므로 3호차
YAML에 명시하지 않는다.

### 1.3 EKF의 과거 보정 해제

과거 3호차 bag은 `a_x/a_y`가 반전돼 있어 EKF 내부에서 yaw180 보정을 사용했다.
현재 드라이버 출력은 이미 정상 FLU이므로 다음을 `false`로 변경했다.

- `src/ekf_pose/config/ekf_pose.yaml`

```yaml
imu_axis_yaw180: false
```

이 값은 EKF 내부에서만 `a_x/a_y` 부호를 바꾸며, `ekf_pose`를 실행하지 않을 때
`/imu/data`에는 영향을 주지 않는다.

## 2. 3호차 실행 방법

기존 `imu` alias는 인자 없이 실행하면 기본
`f1tenth_upside_down.yml`을 사용한다. 그 파일에는 예전 `[π,0,π]` 값이 있으므로,
**3호차에서는 반드시 아래 차량 전용 YAML을 명시한다.**

```bash
imu params_file:=/home/a/f1tenth_ws/src/driver/new_imu/microstrain_inertial_driver/config/f1tenth_car3_imu_xy_corrected.yml
```

`params_file:=...` 앞에 같은 줄의 `\`를 넣지 않는다. 줄을 나눌 때만 백슬래시를
줄 끝에 둔다.

```bash
ros2 launch microstrain_inertial_driver microstrain_launch.py \
  params_file:=/home/a/f1tenth_ws/src/driver/new_imu/microstrain_inertial_driver/config/f1tenth_car3_imu_xy_corrected.yml
```

정상 시작 로그에는 다음이 포함돼야 한다.

```text
Setting sensor to vehicle transformation euler to [0.000000, 0.000000, 0.000000]
Saving the launch file configuration settings to the device
```

## 3. 검증 절차

펌웨어·드라이버·장착 상태를 바꾼 뒤에는 정지 기울임만으로 판정하지 않는다.
반드시 실제 주행 데이터로 확인한다.

```bash
ros2 run ekf_pose imu_frame_test
```

8자 또는 좌/우 코너를 섞어 40초 이상 주행한다. 출발 가속과 제동도 여러 번 넣어야
종가속 `a_x` 부호를 판단할 수 있다. 통과 조건은 다음이다.

```text
정지 a_z ≈ +9.8
자이로 z: 정상
횡가속 a_y: 정상
종가속 a_x: 정상
```

## 4. downstream 소비자 주의

현재 3호차 `/imu/data`는 이미 정상 FLU이다. 따라서 downstream에서 이전 부호 보정을
다시 적용하면 이중 반전된다.

- `ekf_pose`: `imu_axis_yaw180: false` 적용 완료.
- `smppi_cuda_controller`: MPPI 수신 경계의 `imu_wz_sign`, `imu_ax_sign`,
  `imu_ay_sign`을 모두 `+1`로 변경했다. 따라서 MPPI 상태/KF/학습 모델은
  3호차 `/imu/data`와 같은 FLU를 사용한다.
- `als_ros` MCL: 가속도는 MCL 모션 모델에서 사용하지 않고, 수신 IMU의 차량
  yaw-rate만 적분한다. `imu_rotation_*`으로 장착 회전을 적용한 뒤
  `imu_wz_sign`을 적용하도록 만들었으며, 3호차 라이브 기본값은 `+1`이다.
  `mcl.launch.py`의 `imu_rotation_roll` 기본값도 `0.0`이므로 이중 반전이 없다.

## 5. 4호차 상태와 금지 사항

4호차는 이번 정상화 작업에서 다음을 포함해 어떠한 변경도 하지 않았다.

- IMU firmware `sensor2vehicle`
- `new_imu` launch/YAML
- `pacejka_sysid` 학습·추론 설정

4호차의 기존 조사 결과는 firmware identity + 소프트웨어 roll180 보정이었다. 4호차에
3호차 YAML을 실행하면 `save_settings: true`가 장치 NVM에 저장되므로 **절대 실행하지
않는다.**

4호차를 다시 다룰 때는 먼저 별도 실차 `imu_frame_test`를 수행하고, 기존 학습 당시
`imu_rotation_roll=180`이 유지됐는지도 git 로그로 재확인한다.

```bash
git log -p --follow -- src/control/pacejka_sysid/params/sysid_node.yaml | \
  grep -A2 imu_rotation_roll
```

## 6. 남은 작업

- [ ] 3호차 SMPPI/MCL IMU 부호 변경 후 저속 실차에서 MPPI yaw 추종 및 MCL pose를 검증.
- [ ] 3호차 IMU 실행 alias 또는 bringup launch에 차량별 `params_file`을 고정해,
      인자 없는 `imu` 실행으로 예전 기본 YAML이 적용되는 실수를 방지.
- [ ] 4호차는 별도 검증 전까지 현재 firmware/소프트웨어 설정을 변경하지 않음.
