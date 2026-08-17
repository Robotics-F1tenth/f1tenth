# IMU 축 보정 — 3·4호차 FLU 검증 완료 (2026-08-17)

3호차의 IMU 축을 펌웨어 identity + ROS 드라이버 보정으로 정상화한 작업 기록이다.
2026-08-17에 4호차에서도 아래 alias로 `f1tenth_imu_xy_corrected.yml`을 지정해
드라이버를 실행했다. 이후 15:28 KST에 4호차에서 `imu_frame_test`를 수행해
`/imu/data`의 세 축 부호가 정상임을 확인했다. 따라서 이전의 **“4호차 미변경”**
기록은 더 이상 현재 상태를 나타내지 않는다.

```bash
alias imu='sudo chmod 777 /dev/tty* && cd ~/f1tenth_ws/ && source install/setup.bash && ros2 launch microstrain_inertial_driver microstrain_launch.py params_file:=/home/a/f1tenth_ws/src/driver/new_imu/microstrain_inertial_driver/config/f1tenth_imu_xy_corrected.yml'
```

관련 문서: `ekf-pose-analysis-2026-08.md`, `handoff-control-2026-08-04.md`.

## 0. 현재 상태

| 차량 | 펌웨어 `sensor2vehicle` | ROS 드라이버 보정 | 검증 상태 |
|---|---|---|---|
| 3호차 | Euler `[0, 0, 0]` identity, NVM 저장 | `imu_data_roll180: true` | 2026-08-15 실차 `imu_frame_test` 통과 |
| 4호차 | identity `[0,0,0]`을 다시 쓰도록 요청됨. `save_settings: true`이므로 launch가 정상 완료됐다면 NVM에도 저장됨 | `imu_data_roll180: true` 적용. `pacejka_sysid` runtime parameter도 `imu_rotation_roll=0.0` 확인 | **2026-08-17 실차 `imu_frame_test` 및 좌/우회전 부호 검사 통과**. NVM save 성공 여부만 시작 로그로 추후 확인 |

3호차의 현재 `/imu/data`는 차체 FLU 기준이다. 따라서 앞으로 3호차에서
`ekf_pose`를 실행할 때 `imu_axis_yaw180`은 `false`여야 한다. 4호차도 위
driver가 실행 중인 동안에는 같은 publisher-side roll180이 적용되므로, 출력의
`a_y`, `a_z`, `gyro_z`는 보정 전 값과 부호가 반대가 된다. 4호차도 아래 실차
검사로 정상 FLU임을 확인했다.

```text
정지 a_z: +9.78 m/s²
자이로 z: 기울기 +0.85, 상관 +0.90
횡가속 a_y: 기울기 +1.04, 상관 +0.96
종가속 a_x: 기울기 +0.70, 상관 +0.70
```

위 값은 `/odom` 기준 40초 8자 주행에서 얻었다. 세 축 모두 부호가 정상이다.

### 0.1 4호차 실차 검사 결과 (2026-08-17 15:28 KST)

`python3 src/ekf_pose/scripts/imu_frame_test.py`로 `/imu/data`와 `/odom`을 40초간
수집했다. IMU 4,001개, odom 1,692개 중 주행 표본 3,043개가 사용됐다.

```text
정지 a_z: +9.79 m/s²
자이로 z: 기울기 +0.41, 상관 +0.74 — 정상(부호는 맞음, 크기는 어긋남)
횡가속 a_y: 기울기 +0.77, 상관 +0.81 — 정상
종가속 a_x: 기울기 +0.45, 상관 +0.58 — 정상(부호는 맞음, 크기는 어긋남)
판정: 반전된 축 없음
```

따라서 4호차 live 경로에서도 `ekf_pose.imu_axis_yaw180: false`와 차체 FLU를
전제로 한 Coriolis 부호(`vx += ax + vy·yaw_rate`, `vy += ay - vx·yaw_rate`)를
그대로 사용한다. 기울기는 `/odom` 기반 크기 비교이므로, 이 결과는 축 **부호**
판정이지 IMU/odom gain 보정 근거는 아니다.

### 0.2 SysID runtime 및 좌/우회전 최종 확인 (2026-08-17)

재빌드·재시작한 `pacejka_sysid`에서 다음을 확인했다.

```text
ros2 param get /pacejka_sysid imu_rotation_roll
Double value is: 0.0
```

그 상태로 직접 `/imu/data`를 확인한 결과, 전진 좌회전에서는
`angular_velocity.z > 0` 및 `linear_acceleration.y > 0`, 전진 우회전에서는 두 값이
모두 `< 0`였다. 따라서 SysID가 사용하는 `ω`, `a_y` 입력은 FLU 부호와 일치하며,
추가 roll180 보정은 적용하지 않는다.

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

### 2.1 `f1tenth_imu_xy_corrected.yml`은 범용 설정이 아니다

2026-08-17에 4호차에서 명시한
`f1tenth_imu_xy_corrected.yml`은 이름과 달리 3호차 파일
`f1tenth_car3_imu_xy_corrected.yml`과 사실상 같은 설정이다. 두 파일의 차이는
`port`뿐이다.

| 항목 | `f1tenth_imu_xy_corrected.yml` | `f1tenth_car3_imu_xy_corrected.yml` |
|---|---|---|
| 포트 | `/dev/microstrain_imu` | `/dev/ttyACM1` |
| `sensor2vehicle` | Euler `[0,0,0]` | Euler `[0,0,0]` |
| NVM 저장 | `save_settings: true` | `save_settings: true` |
| ROS 출력 보정 | `imu_data_roll180: true` | `imu_data_roll180: true` |

즉 4호차에서 symlink가 실제 IMU를 가리킨 상태로 이 파일을 실행하면 다음 두 효과가
발생한다.

1. 실행 중 `/imu/data`, `/imu/data_raw`, `/imu/mag`의 벡터와 orientation에 `Rx(π)`가
   적용된다. `(x,y,z) → (x,-y,-z)`이므로 `a_y`, `a_z`, `gyro_z`가 이전 raw 출력과
   반전된다.
2. 드라이버는 sensor2vehicle을 identity로 설정한 뒤, `save_settings: true`이면
   `configureBase`/`configure3DM`/`configureGNSS`/`configureFilter` 뒤의 장치 설정을
   NVM에 저장한다. 설정 파일의 일부 키만 저장하는 동작이 아니다.

`params_file`은 절대 경로로 전달되므로, `source install/setup.bash` 후에도 이 source
트리의 YAML 내용이 그대로 사용된다. 단, 현재 `install/.../config`에는 이 범용 파일이
없으므로 해당 절대 경로를 생략하거나 다른 alias로 실행하면 동일한 결과를 기대할 수 없다.

또한 alias의 `sudo chmod 777 /dev/tty*`는 모든 tty 장치의 권한을 넓히며 축 보정과는
무관하다. 이 저장소의 제공 udev rule은 MicroStrain 장치에 `MODE="0666"`과
`/dev/microstrain_main` 등의 symlink를 부여한다. 4호차의 `/dev/microstrain_imu`가
별도 rule이라면 그 rule을 확인·수정해 필요한 IMU 장치에만 권한을 주고, 광범위한
`chmod`는 alias에서 제거한다.

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
- `pacejka_sysid`: driver가 보정한 live `/imu/data`를 다시 뒤집지 않도록
  `imu_rotation_roll`의 노드 기본값과 `sysid_node.yaml`을 모두 `0.0`으로 변경했다
  (commit `1b68ebb`). 이미 실행 중인 SysID node는 재시작해야 새 값이 적용된다.
- `smppi_cuda_controller`: MPPI 수신 경계의 `imu_wz_sign`, `imu_ax_sign`,
  `imu_ay_sign`을 모두 `+1`로 변경했다. 따라서 MPPI 상태/KF/학습 모델은
  3호차 `/imu/data`와 같은 FLU를 사용한다.
- `als_ros` MCL: 가속도는 MCL 모션 모델에서 사용하지 않고, 수신 IMU의 차량
  yaw-rate만 적분한다. `imu_rotation_*`으로 장착 회전을 적용한 뒤
  `imu_wz_sign`을 적용하도록 만들었으며, 3호차 라이브 기본값은 `+1`이다.
  `mcl.launch.py`의 `imu_rotation_roll` 기본값도 `0.0`이므로 이중 반전이 없다.

## 5. 4호차 실행 기록, 영향 및 즉시 확인 사항

4호차에서 사용한 alias는 3호차와 동등한 보정 YAML을 명시했으므로, 더는 “4호차를
변경하지 않았다”고 볼 수 없다. 2026-08-17 `imu_frame_test`는 세 축 부호가 정상인
FLU 출력임을 확인했다. 다만 driver 시작 로그를 확보하지 못했으므로 NVM 저장 성공만은
확정할 수 없다.

가장 큰 호환성 위험이었던 `pacejka_sysid`의 이중 보정은 commit `1b68ebb`에서
해제했다. node 기본값과 YAML 모두 `imu_rotation_roll: 0.0`이다. 새 driver가
`imu_data_roll180: true`로 이미 `(x,-y,-z)` 보정한 `/imu/data`에 roll180을 다시
적용하면 `a_y`, `a_z`, `gyro_z`가 driver 보정 전 부호로 되돌아가므로, 현재 live
3·4호차에서는 `0.0`을 유지한다. raw/z-down IMU 또는 과거 bag을 쓸 때만 실차 축
검증 후 `180.0`을 명시적으로 override한다.

MCL live launch의 `imu_rotation_roll` 기본값은 이미 `0.0`이다. 새 driver 출력이
정상 FLU라면 이 값은 맞지만, 4호차의 실제 축이 3호차와 다르면 역시 검증이 필요하다.

다음 순서로 확인한다.

```bash
# 1) driver 시작 직후: NVM save 및 현재 ROS 파라미터 확인
ros2 param get /microstrain_inertial_driver \
  filter_sensor2vehicle_frame_transformation_euler
ros2 param get /microstrain_inertial_driver imu_data_roll180

# 2) 실행 로그에 아래 두 줄이 있었는지 확인
# Setting sensor to vehicle transformation euler to [0.000000, 0.000000, 0.000000]
# Saving the launch file configuration settings to the device

# 3) 4호차 실차에서 축 부호 확인
ros2 run ekf_pose imu_frame_test
```

4호차 검사는 위 조건을 만족해 통과했으며, SysID 재시작 뒤
`imu_rotation_roll=0.0`과 좌/우회전 `ω`·`a_y` 부호까지 확인했다.

```bash
git log -p --follow -- src/control/pacejka_sysid/params/sysid_node.yaml | \
  grep -A2 imu_rotation_roll
```

## 6. 남은 작업

- [ ] 3호차 SMPPI/MCL IMU 부호 변경 후 저속 실차에서 MPPI yaw 추종 및 MCL pose를 검증.
- [ ] 3호차 IMU 실행 alias 또는 bringup launch에 차량별 `params_file`을 고정해,
      인자 없는 `imu` 실행으로 예전 기본 YAML이 적용되는 실수를 방지.
- [ ] 4호차에서 2026-08-17 launch의 시작 로그를 확보해 NVM 저장 성공 여부를 확인.
- [x] 4호차 `imu_frame_test`를 새 YAML 활성 상태에서 수행 — 2026-08-17 세 축 부호 통과.
- [x] 3·4호차 live SysID 이중 roll 보정 해제 — node 기본값/YAML `0.0`, commit `1b68ebb`.
- [x] SysID node 재시작 후 live parameter `imu_rotation_roll=0.0` 및 좌/우회전
      `ω`·`a_y` 부호 확인 — 좌회전 둘 다 양수, 우회전 둘 다 음수.
- [ ] 차량별 파일명을 명확히 분리하고, `f1tenth_imu_xy_corrected.yml`처럼 범용으로
      보이는 3호차 설정은 제거 또는 안전한 `save_settings: false` 기본 파일로 교체.
