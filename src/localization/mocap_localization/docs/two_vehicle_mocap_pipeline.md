# 3호차·4호차 동시 mocap GT 파이프라인

## 1. 목표와 현재 결론

목표 데이터는 다음 세 묶음이다.

1. ego 명령과 상태: 제어 의도, 실제 mux 출력, `/newmcl_pose`
2. ego GT: ego 차량의 Motive rigid body에서 만든 pose
3. opponent 추정과 GT: perception track과 상대 차량 rigid body pose

현재 확인된 핵심 결론은 다음과 같다.

- 한 `/rigid_bodies` 메시지 안에 여러 rigid body가 함께 들어오므로 OptiTrack
  driver는 하나만 실행하면 된다. 두 localization node가 차량 이름으로 골라 각각 pose를
  발행한다.
- `T_map_mocap`은 방과 map 사이의 변환이므로 두 차량이 **완전히 같은 값**을 써야 한다.
  `T_tracked_point_base`만 차량별 값이다.
- 현재 `params.yaml`과 `params2.yaml`의 공통 변환이 yaw 1.431도, translation 약
  9.28 cm 다르다. 7/30 주행영역에서는 두 GT 프레임 사이 p50 약 13 cm, 최대 약
  20 cm의 계통오차를 만들었다.
- 8/6 bag의 `/rigid_bodies` header는 약 70 Hz로 일정해 기록 구간의 upstream gap이 없다는
  강한 증거가 있다. 그러나 `frame_number`는 Motive `iFrame`이 아니라 driver-local counter라
  native camera-frame 무손실을 혼자 증명하지는 못한다. ROS 기록 시각은 100~140 ms씩
  멈췄다가 몰렸고, 두 derived pose가 약 31%씩 빠졌다.
- 따라서 capture rate를 낮추는 것만으로는 해결되지 않는다. driver QoS, domain bridge,
  executor/로그, recorder 부하를 단계적으로 분리해야 한다.
- `RELIABLE` publisher와 `BEST_EFFORT` subscriber 자체는 호환된다. 문제 후보는 QoS
  incompatibility가 아니라 upstream의 `RELIABLE + KeepLast(1000)` writer와 reliable reader가
  NatNet callback/DDS 전달을 밀리게 하는 back-pressure다.
- 현 상태의 `/mocap_pose2` quaternion yaw는 직진 course와 10~12도 차이가 관측되어
  body axis/pivot을 다시 확인하기 전까지 상대차 yaw GT로 쓰지 않는다.

숫자 파라미터는 새 측량·bag 없이 임의로 확정하지 않는다. 아래 절차를 통과한 candidate만
운영 파일에 반영한다.

## 2. 권장 시스템 구조

```text
OptiTrack cameras
  -> Motive Windows desktop
     -> NatNet UDP (70 Hz 권장 시작점)
        -> Victus Ubuntu bridge laptop, ROS domain 11
           mocap4r2_optitrack_driver_node
             -> /rigid_bodies
                -> car3 rigid-body consumer -> /mocap/car3/pose
                -> car4 rigid-body consumer -> /mocap/car4/pose
                -> timing probe / central rosbag
           domain_bridge 11 -> car3 domain 12 (두 physical pose)
           domain_bridge 11 -> car4 domain 13 (두 physical pose)
           optional domain_bridge 12/13 -> 11
             <- namespaced MCL, perception, control topics for central recording
```

권장 recorder 위치는 Victus다. raw `/rigid_bodies`를 domain bridge 전 단계에서 기록할 수
있고, 각 차량의 필요한 출력만 Victus 쪽으로 선택적으로 bridge하면 한 bag에 시간축을
모을 수 있다.

두 차량이 같은 전역 토픽 이름(`/newmcl_pose`, `/scan`, `/ackermann_cmd`)을 쓰므로 두
차량 domain 전체를 한 domain으로 합치면 안 된다. 물리 차량 namespace로 remap하거나,
한 실험에서 ego 차량의 토픽만 central domain으로 가져온다.

`pose1=ego`, `pose2=opponent`라는 의미를 영구적으로 부여하지 않는다. 역할을 바꾸면 의미가
뒤집힌다. 가능하면 물리 이름을 canonical topic으로 사용한다.

```text
/mocap/car3/pose
/mocap/car4/pose
```

기존 evaluator가 `/mocap_pose1`, `/mocap_pose2`를 요구할 때만 실험 launch에서 ego와
opponent 역할에 맞춰 remap한다.

## 3. Windows Motive 설정

### 3.1 네트워크

Windows Motive PC에는 보통 NIC가 두 개 필요하다.

- OptiTrack camera/switch 전용 NIC
- Victus·차량·공유기와 통신하는 실험망 NIC

Motive의 `Local Interface`는 **Windows PC에 실제로 붙은 실험망 NIC의 Windows IP**다.
Ubuntu/Victus IP를 입력하는 칸이 아니다. 이 IP와 Victus의 `local_address`가 같은 subnet에
있어야 한다.

예시:

```text
Motive Windows experimental NIC: 192.168.51.44
Victus Ubuntu NIC:               192.168.51.17
netmask:                         255.255.255.0
```

Motive에서:

1. `Edit -> Settings -> Streaming`을 연다.
2. NatNet streaming/frame data를 켠다.
3. `Local Interface`를 Windows의 `192.168.51.44` 같은 실험망 NIC로 고른다.
4. 먼저 `Unicast`로 구성한다. driver 설정도 같은 mode여야 한다.
5. capture/stream rate는 70 Hz부터 검증한다. rate 변경은 실험 manifest에 기록한다.

Windows firewall에서 NatNet UDP command/data port 1510/1511을 허용한다. `ping` 성공은
ICMP 연결만 확인하며 UDP streaming 품질을 보장하지는 않는다.

공식 [OptiTrack MOCAP4ROS2 설정 문서](https://docs.optitrack.com/v3.1/robotics/mocap4ros2-setup)도
Motive의 Local Interface를 Ubuntu companion과 같은 network의 Windows IP로 선택하도록
안내한다.

### 3.2 rigid body 생성

3호차와 4호차의 marker layout은 서로 구분 가능하고 가림에 강하게 만든다. 최소 marker 수만
맞추는 것보다 비대칭 5~6 marker 배치를 권장한다.

각 차량에 대해:

1. 해당 차량 marker만 선택해 rigid body를 만든다.
2. Motive asset name을 물리 차량명과 명확히 연결한다. 예: `car3`, `car4`.
3. pivot을 ROS `base_link` 원점, 현재 차량 정의가 후륜축 중심이면 그 지점에 둔다.
4. local +X를 차량 전방, +Y를 위쪽으로 두고, 현재 코드의 `y_up -> z_up` 변환 후 ROS
   +X forward, +Y left, +Z up이 되는지 실제 회전 시험으로 확인한다.
5. Motive project 파일과 marker layout 사진을 실험 artifact로 보존한다.

기존 body 이름 `6`, `7`이 3호차·4호차 중 무엇인지는 저장소만으로 알 수 없다. 한 대만
움직여 `/rigid_bodies`에서 변하는 body를 확인하고 manifest에 고정한다.

### 3.3 ground plane 인수인계값

인수인계에는 Motive `Change Ground Plane` 원점으로 `x=-5.5112`, `y=-0.0029`,
`z=0.1467`이 남아 있다. 이 숫자만으로 당시 ground-plane orientation, 적용 방향, Motive
project 상태를 복원할 수는 없다. 운영값으로 다시 입력하기 전에 원본 Motive project를
보존하고, map 원점 기준점과 +X 방향 기준점 두 개를 물리적으로 확인한다.

Motive ground plane을 map과 직접 일치시키면 shared `T_map_mocap`을 identity에 가깝게 둘 수
있다. 그렇지 않으면 ground plane은 고정한 뒤 8절의 측량점 fit으로 shared transform을 한 번만
구한다. 차량마다 ground plane 또는 shared transform을 다르게 맞추지 않는다.

## 4. Victus bridge workspace 구성

현재 Victus 출력은 ROS 2 Jazzy와 `/home/a/Downloads/mocap_ws/install` 실행본을 사용했다.
Humble 명령을 섞지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/Downloads/mocap_ws/install/setup.bash
```

새 workspace를 만들 때는 동일 package를 제공하는 `mocap_msgs` 저장소를 두 벌 두고 하나를
임의 삭제하지 않는다. 먼저 package 중복을 확인한다.

```bash
cd /home/a/Downloads/mocap_ws
colcon list | sort
find src -name package.xml -print
```

현재 upstream README의 간단한 설치 흐름은 다음과 같다.

```bash
mkdir -p /home/a/Downloads/mocap_ws/src
cd /home/a/Downloads/mocap_ws/src
git clone -b rolling https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack.git
vcs import . < mocap4ros2_optitrack/dependency_repos.repos
cd /home/a/Downloads/mocap_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
```

사용 중인 checkout이 과거 인수인계의 `mocap`, `mocap_msgs` 조합이라면 무조건 다시 clone하지
말고 먼저 SHA와 package provider를 보존한다. 현재
[MOCAP4ROS2 공식 build 문서](https://mocap4ros2-project.github.io/build_instructions/index.html)도
같이 확인한다.

## 5. OptiTrack driver 설정

driver source의 network params를 수정하고 source workspace를 다시 build한다. `install/`
사본만 직접 수정하지 않는다. 먼저 실제 Victus checkout을 확인한다.

```bash
git -C /home/a/Downloads/mocap_ws/src/mocap4ros2_optitrack rev-parse HEAD

rg -n 'qos_|create_publisher|frame_number_|iFrame|CameraMidExposureTimestamp' \
  /home/a/Downloads/mocap_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver
```

network 설정의 권장 시작값:

```yaml
mocap4r2_optitrack_driver_node:
  ros__parameters:
    connection_type: "Unicast"
    server_address: "192.168.51.44"  # Motive Windows experimental NIC
    local_address: "192.168.51.17"   # Victus Ubuntu NIC
    multicast_address: "239.255.42.99"
    server_command_port: 1510
    server_data_port: 1511
```

### 5.1 QoS YAML 함정과 실제 수정

8/6 실행 상태에서 driver publisher는 `RELIABLE`이었다. 공식 rolling commit
`7b92128`도 YAML의 `qos_history_policy`, `qos_reliability_policy`, `qos_depth`를 선언하거나
읽지 않고, [publisher를 C++에서 `rclcpp::QoS(1000)`으로 생성한다](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/blob/7b92128fe05ef6e5126d6727e5bf82bd60e02445/mocap4r2_optitrack_driver/src/mocap4r2_optitrack_driver/mocap4r2_optitrack_driver.cpp#L196-L204).
즉 이 계열은 `RELIABLE + KeepLast(1000)`이며 **YAML만 바꿔도 QoS는 바뀌지 않는다**.

이 저장소에는 해당 upstream commit용 patch 두 개가 있다.

- `mocap4r2_optitrack_driver_timing_diagnostics.patch`: `data->iFrame`, callback 간격,
  rigid publish 시간, 전체 callback 시간을 1초 집계
- `mocap4r2_optitrack_driver_low_latency.patch`: publisher를 bounded best-effort로 변경

먼저 diagnostics만 적용한 reliable baseline을 기록한 뒤 low-latency patch를 추가해 같은
조건으로 비교한다.

```bash
DRIVER_REPO=/home/a/Downloads/mocap_ws/src/mocap4ros2_optitrack
PATCH_DIR=/path/to/f1tenth/src/localization/mocap_localization/patches
DIAG_PATCH="$PATCH_DIR/mocap4r2_optitrack_driver_timing_diagnostics.patch"
LOW_LATENCY_PATCH="$PATCH_DIR/mocap4r2_optitrack_driver_low_latency.patch"

git -C "$DRIVER_REPO" apply --check "$DIAG_PATCH"
git -C "$DRIVER_REPO" apply "$DIAG_PATCH"

cd /home/a/Downloads/mocap_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mocap4r2_optitrack_driver
source install/setup.bash

# reliable baseline 측정이 끝난 다음
git -C "$DRIVER_REPO" apply --check "$LOW_LATENCY_PATCH"
git -C "$DRIVER_REPO" apply "$LOW_LATENCY_PATCH"
colcon build --symlink-install --packages-select mocap4r2_optitrack_driver
```

patch는 publisher를 `BEST_EFFORT + KeepLast(10) + VOLATILE`로 바꾼다. 배포 SHA가 다르거나
`git apply --check`가 실패하면 억지로 적용하지 말고 같은 두 `create_publisher` 줄을 직접
검토한다. 장기적으로는 YAML QoS를 실제 구현하고 NatNet callback은 latest-frame buffer에만
복사한 뒤 ROS executor가 publish하도록 분리하는 편이 안전하다.

현재 localization consumer는 이미 `BestEffort + KeepLast(1)`라 최신값만 사용한다. 이것은
live control에는 맞지만 burst 중 모든 중간 frame을 보존하지 않는다. archive GT는 반드시
Victus의 raw `/rigid_bodies`를 기록하고 offline으로 다시 변환한다.

driver의 reliable write 또는 그 이전 NatNet SDK가 batching의 근본 원인이라는 것은 아직
가설이다. [upstream은 NatNet callback 안에서 바로 DDS publish한다](https://github.com/MOCAP4ROS2-Project/mocap4ros2_optitrack/blob/7b92128fe05ef6e5126d6727e5bf82bd60e02445/mocap4r2_optitrack_driver/src/mocap4r2_optitrack_driver/mocap4r2_optitrack_driver.cpp#L92-L186).
아래 A/B와 UDP packet capture로 둘을 분리한다.

driver 실행:

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/Downloads/mocap_ws/install/setup.bash
ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
ros2 lifecycle get /mocap4r2_optitrack_driver_node
```

검증:

```bash
ros2 topic info /rigid_bodies --verbose
ros2 topic echo /rigid_bodies --once
```

publisher QoS가 실제로 `BEST_EFFORT`, `VOLATILE`인지 확인한다. 계속 `RELIABLE`이면 이전
install을 source했거나 patch되지 않은 executable을 실행한 것이다.

low-latency patch는 `/markers`도 best-effort로 바꾼다. marker visualization subscriber가
reliable이면 매칭되지 않으므로 `/markers --verbose`도 확인한다. timing A/B 동안에는 marker
visualization을 끄고, 필요하면 viz subscriber를 best-effort로 맞춘 뒤 별도 검증한다.

## 6. 두 차량 pose 동시 발행

이 저장소의 통합 launch를 사용한다.

```bash
cd /home/a/f1tenth
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mocap_localization
source install/setup.bash

ros2 launch mocap_localization mocap_localization_all.launch.py \
  body1_name:=car3 body2_name:=car4 \
  pose1_topic:=/mocap/car3/pose \
  pose2_topic:=/mocap/car4/pose
```

실제 Motive 이름이 `6`, `7`이면 `car3`, `car4` 대신 확인한 mapping을 넣는다. launch는
`params.yaml`과 `params2.yaml`의 차량별 offset을 각각 사용한다.

통합 launch의 기본값은 기존 호환용 body `6/7`, topic `/mocap_pose1/2`다. 물리 mapping을
확정한 뒤에는 위 명령처럼 body와 physical topic을 항상 명시한다.

평소에는 `diagnostic_logging:=false`를 유지한다. yaw와 `now-header.stamp`를 차량별 초당
한 줄만 확인할 때는 `diagnostic_logging:=true`를 추가한다. frame마다 140줄을 쓰던 기존
INFO 경로는 제거됐다.

주의:

- 두 params의 `rotation`과 `translation`은 아직 서로 다르다. 새 calibration을 통과하기
  전에는 cross-vehicle 정밀 GT로 승인하지 않는다.
- 이 저장소 변경은 Victus의 `/home/a/Downloads/mocap_ws/install`에 자동 반영되지 않는다.
- `publish_tf`는 기본 false다. 켤 경우 `child_frame_id`를 차량별로 다르게 지정한다.
- 두 node는 독립적인 depth-1 subscriber라 burst에서 서로 다른 raw frame을 선택할 수 있다.
  상대 pose를 index로 짝짓지 말고 보존된 header timestamp로 매칭하며, 정밀 archive 평가는
  한 raw `/rigid_bodies` 메시지 안의 car3/car4를 함께 offline 변환한다.

## 7. ROS domain bridge

domain_bridge는 다른 `ROS_DOMAIN_ID` 사이의 topic을 선택적으로 전달한다. 공식
[domain_bridge 설정](https://github.com/ros2/domain_bridge/blob/75cdfae6d920a6b62284c22413344b42d06f14f2/doc/design.md#yaml-config-file)는
`from_domain`, `to_domain`, topic별 QoS와 destination remap을 지원한다.

현재 관측된 `/mocap_11_to_13_11`은 domain 11에서 raw `/rigid_bodies`를 `RELIABLE`로
구독했지만 두 pose를 전달하는 endpoint는 아니었다. raw는 Victus domain 11에서 직접
기록하고, 차량에는 bandwidth가 작은 두 physical pose만 보낸다.

아래 `11/12/13`은 Victus/car3/car4용 **template 값**이다. 현재 확인된 것은 기존 이름에서
11→13 경로뿐이며, 12가 실제 car3 domain인지는 아직 현장 확인되지 않았다. 각 PC에서
`echo "$ROS_DOMAIN_ID"`와 실행 launch를 확인한 뒤 config와 manifest를 함께 수정한다.

저장소에는 네 개의 config가 있다.

```text
domain_bridge_mocap_to_car3.yaml  # 11 -> 12, car3/car4 pose 둘 다
domain_bridge_mocap_to_car4.yaml  # 11 -> 13, car3/car4 pose 둘 다
domain_bridge_car3_to_mocap.yaml  # 12 -> 11, /car3/...로 telemetry remap
domain_bridge_car4_to_mocap.yaml  # 13 -> 11, /car4/...로 telemetry remap
```

두 차량에 GT를 보내는 bridge는 각각 별도 terminal에서 실행한다.

```bash
BRIDGE_CFG=$(ros2 pkg prefix mocap_localization)/share/mocap_localization/config

ros2 run domain_bridge domain_bridge "$BRIDGE_CFG/domain_bridge_mocap_to_car3.yaml"
ros2 run domain_bridge domain_bridge "$BRIDGE_CFG/domain_bridge_mocap_to_car4.yaml"
```

Victus 한 곳에서 MCL/perception/control까지 기록할 때만 car-to-mocap config를 추가한다.
처음 timing A/B에서는 이 고대역폭 reverse bridge를 켜지 않는다. 한 실험에서 ego 한 대만
필요하면 해당 차량 config 하나만 실행한다. reverse config에는 `f1_msgs` type support가
필요하므로 Victus에서도 f1tenth overlay를 source하거나 동일 interface package를 설치한다.

```bash
source /path/to/f1tenth/install/setup.bash
ros2 interface show f1_msgs/msg/F1stateArr
ros2 interface show vesc_msgs/msg/VescStateStamped
ros2 interface show smppi_cuda_controller/msg/MppiTrajectory

ros2 run domain_bridge domain_bridge "$BRIDGE_CFG/domain_bridge_car3_to_mocap.yaml"
# 역할이 car4 ego이면 위 파일 대신 domain_bridge_car4_to_mocap.yaml
```

bridge 재시작 후 각 domain에서 physical topic과 QoS를 확인한다.

```bash
ROS_DOMAIN_ID=11 ros2 topic info /mocap/car3/pose --verbose
ROS_DOMAIN_ID=12 ros2 topic info /mocap/car3/pose --verbose
ROS_DOMAIN_ID=12 ros2 topic info /mocap/car4/pose --verbose
ROS_DOMAIN_ID=13 ros2 topic info /mocap/car3/pose --verbose
ROS_DOMAIN_ID=13 ros2 topic info /mocap/car4/pose --verbose
ROS_DOMAIN_ID=12 ros2 topic echo --qos-reliability best_effort \
  /mocap/car3/pose --once

# reverse car3 bridge를 켠 경우
ROS_DOMAIN_ID=11 ros2 topic info /car3/newmcl_pose --verbose
```

두 generic 차량 토픽을 같은 이름으로 domain 11에 합치지 않는다. `/tf`도 topic 이름만
remap하면 message 내부 frame ID 충돌이 남으므로, frame namespace 설계 전에는 두 차량 TF를
central domain으로 무작정 합치지 않는다.

mocap-to-car bridge publisher는 `BEST_EFFORT`다. 차량 쪽 live consumer도 best-effort로
subscribe해야 한다. 기존 `eval_logger.py`는 reliable subscription이므로 domain 11의 원본
pose 또는 central bag replay에서 실행하고, best-effort bridge 출력에 그대로 붙이지 않는다.

## 8. 캘리브레이션 모델

차량 `i`의 올바른 변환 체인은 다음과 같다.

```text
T_map_base_i =
  T_map_mocap              # 방/맵 공통
  * T_mocap_tracked_i      # Motive 측정 + y_up/z_up 축변환
  * T_tracked_base_i       # 차량별 pivot/lever arm
```

### 8.1 차량별 먼저: rigid-body frame과 base_link

가장 좋은 해결은 Motive에서 pivot과 local axes를 실제 `base_link`와 일치시키는 것이다.
그러면 차량별 `tracked_point_to_base_*`가 identity에 가까워진다.

Motive에서 정확히 맞출 수 없으면:

1. marker pivot에서 후륜축 중심까지 x/y를 직접 실측한다.
2. 두 차량을 물리적으로 평행하게 놓고 raw yaw 차이를 측정한다.
3. 각 차량을 같은 위치에서 0/90/180/270도로 정지시켜 기록한다.
4. 보정된 base 위치가 제자리 회전 중 원을 그리면 lever arm이 틀린 것이다.
5. body yaw가 직진 course와 계속 다르면 local axis가 틀린 것이다.

body 7에서 과거 10~12도 문제가 있었으므로 quaternion yaw는 이 시험을 통과하기 전까지
평가에서 제외한다.

### 8.2 공통 한 번: T_map_mocap

독립 GT가 목적이면 MCL을 기준으로 맞추지 않는다. 권장 절차:

1. map 좌표를 알고 있는 기준점을 트랙 전체에 넓게 8~12개 정한다.
2. calibrated target/base_link를 각 점에 정지시키고 raw pose를 2~5초 평균한다.
3. 일부 점은 fit에서 제외해 holdout으로 남긴다.
4. 2D rigid Procrustes/SVD로 단 하나의 yaw+xy를 적합한다.
5. 정확히 같은 `rotation`, `translation`을 두 차량 params에 넣는다.
6. holdout 위치오차, 두 차량 상대거리 보존, 반복 실험 편차를 검증한다.

Motive ground plane의 원점·축을 map 원점·축에 물리적으로 맞출 수 있다면 이 변환을
identity에 가깝게 만드는 것이 더 재현 가능하다.

측량이 불가능해 MCL을 기준으로 쓸 때는 두 차량이 각각 ego였던 별도 calibration session을
합쳐 다음 변수를 한 번에 적합한다.

```text
[shared yaw, tx, ty,
 car3 yaw_offset, dx, dy,
 car4 yaw_offset, dx, dy]
```

이 결과는 독립적인 MCL GT가 아니라 `MCL-aligned reference`다. calibration lap과 evaluation
lap을 분리해야 한다.

### 8.3 shared-map candidate 생성 도구

`config/shared_map_survey_example.csv`를 복사해 synthetic 행을 실제 측량값으로 모두 바꾼다.
CSV의 `mocap_x/y`는 **shared map transform을 적용하기 전의 base-corrected planar point**다.

`collect_shared_map_survey`는 이 CSV를 현장에서 만든다. Z-up/pivot 검증용 identity
pose publisher를 `/mocap_cal/car3/base`, `/mocap_cal/car4/base`로 실행한 뒤, 매 기준점의
ID와 독립 측량한 `map_x map_y`를 입력한다. 도구가 car3를 평균한 뒤 car4를 평균해 두 행을
원자적으로 저장한다.

```bash
ros2 run mocap_localization collect_shared_map_survey \
  --output /data/car34_shared_map_survey.csv \
  --duration-sec 5 \
  --min-samples 30
```

```text
survey> P01 fit -1.250 0.700
Place car3 rear-axle center on the point, keep it still, then press ENTER ...
Place car4 rear-axle center on the point, keep it still, then press ENTER ...
survey> P09 holdout 2.100 -3.400
```

`map_x/y`는 MCL output이 아니라 물리 측량 또는 독립 map landmark에서 얻는다. 중단해도
완료된 point pair는 남으며, 같은 `P01_car3`/`P01_car4`를 다시 기록하려면 새 CSV 경로를
사용한다.

### 8.4 측량 불가 시: MCL-aligned shared map

충돌 뒤 map을 다시 만들거나 물리 기준점의 map 좌표를 재현할 수 없으면, MCL을 기준으로
공통 변환을 맞춘다. 이 결과는 **MCL-aligned reference**이며, 같은 map에서 MCL 정확도를
평가하는 GT로 사용할 수 없다. 반면 동일 map에서 두 차량 mocap을 하나의 frame으로 맞추고
perception 상대차 위치를 평가하는 운영 기준으로는 사용할 수 있다.

`calibration_v2.py`는 tracked-point offset까지 동시에 다시 적합하고 단일 최신 pose를 SPACE로
저장하므로, pivot/axis를 물리적으로 확정한 현재 절차에는 사용하지 않는다. 대신
`collect_mcl_aligned_survey`가 timestamp-matched stationary 평균만 CSV에 저장한다.

car3의 MCL을 central domain으로 `/car3/newmcl_pose`에 bridge한 예시는 다음과 같다.

```bash
ros2 run mocap_localization collect_mcl_aligned_survey \
  --output /data/car34_mcl_aligned_survey.csv \
  --vehicle car3 \
  --mocap-topic /mocap_cal/car3/base \
  --mcl-topic /car3/newmcl_pose \
  --duration-sec 3 --min-pairs 20
```

각 정지 자세에서 `C01 fit` 또는 `C09 holdout`을 입력한다. car4 MCL도 **같은 map**을 쓰고
`/car4/newmcl_pose`로 bridge되어 있다면 같은 CSV에 `--vehicle car4`로 추가한다. 두 차량의
map이 서로 다르면 car3처럼 기준으로 정한 한 차량 데이터만 fit에 쓰고, 생성된 공통 변환을
두 candidate YAML에 동일하게 적용한다. 새 map을 쓰는 실험마다 다시 수행하고 calibration
lap은 MCL 평가 lap에서 제외한다.

수집이 끝나면 현재의 pivot/axis가 identity인 `params3.yaml`/`params4.yaml`을 source로 하여
candidate만 만든다. 아래 명령은 원본을 수정하지 않는다.

```bash
ros2 run mocap_localization calibrate_shared_map \
  --csv /data/car34_mcl_aligned_survey.csv \
  --params1 ~/f1tenth_ws/src/localization/mocap_localization/params/params3.yaml \
  --params2 ~/f1tenth_ws/src/localization/mocap_localization/params/params4.yaml \
  --output-dir /data/car34_mcl_aligned_candidate
```

fit 8개 이상과 holdout 2개 이상을 권장한다. 입력이 car3뿐이어도 output 두 YAML의
`rotation`/`translation`은 하나의 값으로 같아야 하며,
`tracked_point_to_base_*`는 identity/zero로 유지되어야 한다. holdout residual이 충분히 작은지
확인한 뒤 별도 topic으로 shadow 비교하고 운영 YAML에 반영한다.

```text
p_mocap_base = R_axis * p_raw + (q_axis * q_raw) * t_tracked_base
```

Motive pivot/axis를 이미 `base_link`에 맞췄다면 두 번째 항은 거의 0이다. 각 기준점에서
2~5초의 valid raw frame을 평균해 한 행을 만들고, 트랙 전체에 퍼진 점을 `fit`, 사용하지 않은
점을 `holdout`으로 지정한다. 최소 3개 non-collinear fit과 1개 holdout을 강제하지만 실제로는
fit 8~10개, holdout 2~4개를 권장한다.

```bash
ros2 run mocap_localization calibrate_shared_map \
  --csv /data/car34_shared_map_survey.csv \
  --params1 /data/deployed_params_before_cal/params.yaml \
  --params2 /data/deployed_params_before_cal/params2.yaml \
  --output-dir /data/car34_shared_map_candidate
```

도구는 scale 없는 2D SVD fit과 conditioning 검사를 수행하고 fit/holdout p50·p95·RMSE를
출력한다. 원본 params는 절대 쓰지 않으며, 기존 candidate도 `--overwrite` 없이는 덮어쓰지
않는다. 출력 두 YAML의 `rotation`, `translation`은 정확히 같은 값이고 기존 차량별
`tracked_point_to_base_*`는 보존된다.

```bash
sha256sum /data/car34_shared_map_candidate/params*.yaml
diff -u \
  <(rg '^    (rotation|translation):' /data/car34_shared_map_candidate/params.yaml) \
  <(rg '^    (rotation|translation):' /data/car34_shared_map_candidate/params2.yaml)
```

candidate는 별도 output topic으로 shadow replay하고 holdout·상대거리 gate를 통과한 뒤에만
두 파일을 한 묶음으로 승인한다. 도구 자체의 합성 검증은 다음으로 재실행할 수 있다.

```bash
ros2 run mocap_localization calibrate_shared_map --self-test
```

### 8.4 기존 calibration_v2.py 상태

기존 코드는 각 차마다 공통 변환까지 독립적으로 풀어 13 cm 불일치를 만들 수 있다. 또한
MCL을 reference로 쓰고 latest message만 pairing하므로 최종 캘리브레이터로 쓰지 않는다.

이번 수정으로:

- source params를 기본적으로 덮어쓰지 않는다.
- candidate 파일을 복사해 identity 준비 후 그 파일에만 쓴다.
- 기존 candidate도 명시적인 `overwrite_candidate:=true` 없이는 덮어쓰지 않는다.
- 기본 timestamp tolerance를 0.05초로 낮췄다.
- underconstrained fit 경고를 출력한다.

하지만 알고리즘 자체의 shared fit/측량 문제는 남아 있다. 새 측량 데이터 없이 현재
`-91도` 같은 값을 두 운영 파일에 바로 복사하지 않는다.

## 9. 프레임 batching 진단

### 9.1 8/6 bag에서 확인된 사실

```text
/rigid_bodies measurement header / driver sequence:
  driver-local frame_number 153529 -> 155993, sequence gap 0
  reconstructed camera-mid-exposure header p50 14.286 ms, p99 14.403 ms
  body 0, 6, 7 presence 2465/2465 each

rosbag receive timing:
  p95 118.35 ms, p99 130.72 ms, max 223.30 ms
  >60 ms gap 160회

derived poses:
  pose1 약 30.5% 손실
  pose2 약 31.2% 손실
```

upstream driver는 `CameraMidExposureTimestamp`로 계산한 system latency를 Victus의 `now()`에서
빼 header를 만들며, `frame_number_++`로 자체 번호를 만든다. 따라서 위 결과는 기록된 callback
구간의 header timing이 일정했다는 강한 증거지만 Motive native `data->iFrame` 연속성을 직접
증명하지는 않는다. native loss 확정에는 driver에 `data->iFrame` 통계를 추가해야 한다.

확실한 사실은 measurement header가 14.3 ms로 일정한 동안 bag record timing은 100 ms 이상
멈췄다가 몰렸다는 점이다. `BestEffort + KeepLast(1)` consumer는 이 backlog에서 최신 하나만
남기므로 derived pose가 줄어드는 것은 현재 정책의 예상 동작이다.

두 localization node의 frame별 yaw INFO 140줄/초는 callback 지연과 pose drop을 악화시킬 수
있지만, localization보다 앞선 raw record batching까지 혼자 설명하지는 못한다. 이번 변경은
그 증폭요인을 제거한 것이며 root cause 확정은 driver/UDP A/B로 한다.

기존 derived pose는 callback `now()`로 다시 stamp되어 bag에서 age가 약 0 ms처럼 보였다.
수정판은 raw measurement header를 보존하므로 이후 pose의 age와 시간 매칭이 실제 지연을
반영한다.

### 9.2 live probe

Victus source domain에서:

```bash
ros2 run mocap_localization mocap_timing_probe --ros-args \
  -p topic:=/rigid_bodies \
  -p report_period_sec:=1.0 \
  -p gap_threshold_ms:=60.0
```

target/vehicle domain에서도 같은 probe를 실행한다. 출력은 measurement-header/receive Hz,
receive gap, driver sequence gap, measurement age의 p50/p95/max를 초당 한 줄로 보여준다.

header는 Victus ROS clock을 기준으로 재구성된다. target PC clock이 chrony/NTP로 동기화되지
않으면 target에서 measurement age 절대값은 믿지 않는다. 같은 PC의 age 변화, header/receive
inter-arrival, driver counter는 여전히 유효하다.

### 9.3 단계별 A/B 시험

각 단계 30~60초씩 수행한다.

1. driver + Victus probe만
2. localization 두 개 추가
3. mocap-to-car domain bridge만 추가
4. target vehicle subscriber를 추가
5. Victus minimal raw bag을 추가
6. reverse telemetry bridge와 full evaluation bag을 추가

각 단계 사이에 node를 완전히 종료하고 잔여 process를 확인한다.

```bash
ps -ef | rg 'mocap|domain_bridge|ros2 bag'
```

판정:

- UDP packet부터 burst: Motive streaming/Windows NIC/switch/Victus NIC 경로
- UDP는 일정하지만 1단계부터 burst: NatNet SDK callback 또는 driver DDS publish
- 1은 정상, 2부터 burst: localization callback/log/executor
- 2는 정상, 3부터 burst: domain bridge/QoS/downstream
- 3은 정상, 4부터 burst: target reader/DDS/NIC 경로. 실제 QoS가 reliable이면
  back-pressure를 우선 의심한다.
- 4는 정상, 5 또는 6부터 burst: rosbag/SQLite/disk/CPU/telemetry 부하

domain bridge OFF/ON뿐 아니라 driver `reliable` 대 `best_effort`를 같은 rate에서 비교한다.
한 번에 하나의 변수만 바꾼다.

동시에 NatNet NIC의 UDP 1511 arrival을 capture하면 driver 앞/뒤를 분리할 수 있다.

```bash
ip -br addr
sudo tcpdump -i <Motive와_연결된_Victus_NIC> -n -tt -B 4096 \
  'udp port 1511' -w /tmp/natnet_70hz.pcap
```

제공된 diagnostics patch는 초당 한 번 `data->iFrame` 누락·비단조, callback steady-clock
interval max, rigid publish max, callback work max를 남긴다. `callback_dt_max`는 큰데
`callback_work_max`와 `rigid_publish_max`가 작으면 callback 이전 SDK/UDP 쪽을 본다.
`rigid_publish_max`가 함께 100 ms 이상 튀면 reliable DDS write가 직접 원인이다.

### 9.4 offline bag report

```bash
ros2 run mocap_localization mocap_bag_timing_report \
  /path/to/rosbag_directory --gap-ms 60
```

acceptance gate 권장 시작값:

- driver callback-sequence gap = 0
- measurement header p99가 설정 주기의 1.5배 이내
- 계측한 Motive native `iFrame` gap = 0
- receive gap >60 ms = 0
- raw 대비 각 visible body pose count 손실 <1%
- record-header age는 절대 30 ms로 자르지 않고 local stationary baseline을 먼저 정한다.
  provisional jitter gate는 p99-p50 <30 ms이며 sawtooth 증가가 없어야 한다.

이 gate를 통과하지 못한 run은 정밀 GT 평가에 사용하지 않는다.

## 10. 실험과 bag 기록

### 10.1 실험 전 artifact

`config/experiment_manifest_example.yaml`을 복사해 채운다.

- Windows/Victus/차량 IP와 ROS domain
- Motive/NatNet version, capture rate, project 파일
- `car3/car4 <-> Motive rigid-body name` mapping
- ego/opponent 역할
- 각 PC git SHA, `ros2 pkg prefix`, RMW implementation
- Victus와 ego Jetson의 chrony/PTP source, 측정 offset, `chronyc tracking/sources` 출력
- driver/localization/perception/controller param dump
- marker/obstacle 배치 사진

한 central bag에 모았다고 message header clock이 자동으로 같아지는 것은 아니다. mocap
header는 Victus clock, 차량 sensor/MCL header는 Jetson clock이다. 실험 직전에 두 PC에서
`chronyc tracking`과 `chronyc sources -v`를 저장하고 offset이 허용범위인지 확인한다.

특히 현재 controller checkout에는 simulation flag가 hardcode된 정황이 있으므로 실제 주행 전
live parameter와 input topic을 확인하고 `/newmcl_pose`를 정말 사용하는지 증명한다.

또한 현재 source launch는 mux 출력을 `/ackermann_drive`로 remap하지만
`ackermann_to_vesc_node` source는 `/ackermann_cmd`를 구독한다. 배포본이 별도로 수정됐을 수
있으므로 어느 이름이 post-mux라고 가정하지 말고 주행 전에 실제 연결을 확인한다.

```bash
ros2 node info /ackermann_mux
ros2 node info /ackermann_to_vesc_node
ros2 topic info /ackermann_drive --verbose
ros2 topic info /ackermann_cmd --verbose
```

mux publisher와 `ackermann_to_vesc_node` subscriber가 같은 topic에서 매칭되지 않으면 실험을
시작하지 않는다.

### 10.2 먼저 기록할 calibration/QC bag

1. 두 차량 정지 30초
2. 각 차량 0/90/180/270도 정지
3. 같은 지점 제자리 회전
4. 두 차량 평행 배치
5. 측량 기준점 8~12개

각 run 시작과 끝에는 5~10초 정지 구간을 둔다.

### 10.3 dynamic evaluation matrix

- car3 ego / car4 opponent
- car4 ego / car3 opponent
- 장애물 있음 / 없음 대조군
- 각 조건 최소 3회 반복
- calibration에 사용하지 않은 lap/run으로 평가

### 10.4 필수 topic

공통/GT:

```text
/rigid_bodies
/mocap/car3/pose
/mocap/car4/pose
/map
/tf
/tf_static
/parameter_events
```

ego localization/perception:

```text
/scan
/odom
/imu/data
/ekf/imu/data
/newmcl_pose            # central domain에서는 /car3/... 또는 /car4/... 권장
/f1/perception/object/obstacles/arr
/f1/perception/static_obstacles
```

제어 의도와 실제 출력:

```text
/drive
/teleop
/ackermann_drive
/ackermann_cmd
/commands/motor/speed
/commands/servo/position
/sensors/servo_position_command
/sensors/core
/fsm/state
/mppi_optimal_trajectory
/mppi_target_path
/mppi_left_boundary
/mppi_right_boundary
/center_path
/left_boundary
/right_boundary
```

먼저 Victus에서 GT-only bag으로 timing gate를 통과시킨다. QoS override의 canonical topic은
통합 launch와 동일한 `/mocap/car3/pose`, `/mocap/car4/pose`다. live consumer의 depth 1과
archive recorder queue를 혼동하지 않는다. override는 raw depth 256, pose depth 64라 이번
223 ms stall보다 긴 burst도 local recorder reader에서 흡수할 수 있다.

```bash
QOS_FILE=$(ros2 pkg prefix mocap_localization)/share/mocap_localization/config/rosbag_qos_overrides.yaml

ROS_DOMAIN_ID=11 ros2 bag record -o /data/mocap_qc_run \
  --qos-profile-overrides-path "$QOS_FILE" \
  /rigid_bodies /mocap/car3/pose /mocap/car4/pose
```

그 gate가 통과한 뒤 car3가 ego인 central-recording 예시는 다음과 같다. reverse bridge가
`/map`, `/tf`, `/tf_static`은 원래 이름으로, 나머지는 `/car3/...`로 가져온다는 전제다.

```bash
ROS_DOMAIN_ID=11 ros2 bag record -o /data/car3_ego_run \
  --qos-profile-overrides-path "$QOS_FILE" \
  /rigid_bodies /mocap/car3/pose /mocap/car4/pose \
  /map /tf /tf_static /parameter_events /car3/parameter_events \
  /car3/scan /car3/odom /car3/imu/data /car3/ekf/imu/data \
  /car3/newmcl_pose \
  /car3/f1/perception/object/obstacles/arr \
  /car3/f1/perception/static_obstacles \
  /car3/drive /car3/teleop /car3/ackermann_drive /car3/ackermann_cmd \
  /car3/commands/motor/speed /car3/commands/servo/position \
  /car3/sensors/servo_position_command /car3/sensors/core \
  /car3/fsm/state /car3/mppi_optimal_trajectory \
  /car3/mppi_target_path /car3/mppi_left_boundary /car3/mppi_right_boundary \
  /car3/center_path /car3/left_boundary /car3/right_boundary
```

car4 ego일 때는 `/car3/...`를 `/car4/...`로 바꾼다. 실행 전에 각 custom interface의
`ros2 topic type`과 reverse bridge YAML type이 같은지 확인한다.
두 reverse bridge를 동시에 켜면 generic `/map`/TF가 충돌하므로 이 예시는 ego 한 대만
central로 가져오는 운영을 전제로 한다.

`/sensors/servo_position_command`는 실제 조향각 센서 GT가 아니라 VESC가 되돌려주는 command
값이다. Float64 actuator topic에는 source header가 없어 central bag timestamp에 reverse
bridge 지연이 포함된다. 정밀 command-to-actuator latency가 필요하면 ego 차량에서도 같은
actuator topic만 담은 작은 local bag을 동시에 기록하거나 source-stamped relay를 추가한다.

## 11. 평가 순서와 금지사항

평가는 다음 gate 순서를 지킨다.

1. timing/frame QC
2. rigid-body ID/marker visibility/axis QC
3. shared calibration holdout QC
4. ego `/newmcl_pose` 대 ego GT
5. perception track 대 opponent GT
6. control 선택과 명령 반응

### 11.1 ego MCL

- 동일 measurement timestamp 또는 nearest-time tolerance로 매칭한다.
- index 정렬을 쓰지 않는다.
- 긴 mocap gap을 가로질러 interpolation하지 않는다.
- MCL로 calibration한 run을 다시 MCL 평가에 사용하지 않는다.

### 11.2 opponent perception

- 공통 `T_map_mocap` 검증 전의 absolute position error는 정식 결과로 인용하지 않는다.
- 기존 0.5 m association gate 때문에 큰 match-rate 차이는 남을 수 있지만 경계 sample과
  position error에는 13 cm 오염이 직접 들어간다.
- 기존 문서의 “opponent였던 stationary track 4~16%, control 0%” 요약은 현재 표와 맞지
  않는다. 표의 random-control p95는 4%, 5%, 8%이고 3개 bag 중 2개만 control보다 컸다.
  shared calibration과 재현 가능한 matching script로 다시 계산하기 전에는 예비 관측이다.
- body quaternion yaw가 axis test를 통과하기 전에는 위치 차분 course를 사용하되 gap 인접
  sample을 제외한다.
- role swap 때 evaluator topic을 반드시 바꾼다.
- 현재 perception `F1stateArr.header`가 발행 시 채워지지 않아 evaluator가 callback `now()`를
  대신 쓰는 경로가 있다. 이를 고치기 전에는 perception absolute latency를 결과로 인용하지
  않고, position association에도 bridge/record 지연이 섞였다고 본다.
- mocap gap을 가로질러 보간하지 말고 raw header/counter로 GT-validity mask를 먼저 만든다.
- controller가 실제 선택한 opponent ID를 별도 출력하지 않으면 offline nearest-GT track이
  다른 track을 채점할 수 있음을 명시한다.

기존 evaluator의 absolute `/mocap_pose1=ego`, `/mocap_pose2=opponent` 계약을 당장 유지해야
한다면 domain 11 또는 central bag replay에서 실행 시 role별 remap을 명시한다.
아래는 central bag에 들어 있는 **baked namespaced MCL/track**을 평가하는 명령이다. MCL과
perception을 fresh replay할 때는 새 노드가 generic output을 내므로 마지막 두 remap을 빼고
bag에서도 baked MCL/track topic을 재생하지 않는다.

```bash
# car3 ego / car4 opponent
python3 /home/a/f1tenth/eval/eval_logger.py --out /tmp/car3_ego --ros-args \
  -r /mocap_pose1:=/mocap/car3/pose \
  -r /mocap_pose2:=/mocap/car4/pose \
  -r /newmcl_pose:=/car3/newmcl_pose \
  -r /f1/perception/object/obstacles/arr:=/car3/f1/perception/object/obstacles/arr

# car4 ego / car3 opponent
python3 /home/a/f1tenth/eval/eval_logger.py --out /tmp/car4_ego --ros-args \
  -r /mocap_pose1:=/mocap/car4/pose \
  -r /mocap_pose2:=/mocap/car3/pose \
  -r /newmcl_pose:=/car4/newmcl_pose \
  -r /f1/perception/object/obstacles/arr:=/car4/f1/perception/object/obstacles/arr
```

### 11.3 replay

- isolated ROS domain을 사용한다.
- bag의 `/map`, TF, `--clock`, `use_sim_time:=true`를 쓴다.
- perception/MCL을 fresh replay할 때 bag의 baked output topic은 재생하지 않는다.
- 특히 과거 full-bag replay처럼 기존 track output과 새 perception output을 같은 topic에
  동시에 발행하지 않는다.
- 모든 metric에 sample count를 붙이고 n<50 구간은 headline 결과로 쓰지 않는다.

## 12. 진행 순서

다음 순서가 가장 짧은 critical path다.

1. Victus 실제 source SHA/config/params를 이 저장소와 대조하고 기존 reliable raw bridge를
   A/B 동안 중지한다.
2. Motive body name과 3·4호차 mapping을 확정한다.
3. UDP capture를 켠 채 driver patch 전/후와 domain bridge OFF/ON을 각각 비교한다.
4. 검증된 경우에만 driver를 `keep_last + best_effort`, pose bridge를 best effort로 운영한다.
5. 이 저장소의 timestamp/log 수정판과 통합 launch를 Victus candidate workspace에서 실행한다.
6. timing acceptance gate를 통과한다.
7. Motive pivot/axis를 두 차량에서 수정하고 static rotation bag으로 검증한다.
8. 측량점 기반 공통 `T_map_mocap` candidate를 만든다.
9. 두 차량 holdout/상대거리 검증 후 두 params를 동시에 승인한다.
10. 역할 교대 dynamic bag을 기록한다.
11. GT-validity mask를 적용해 MCL/perception/control 평가를 재실행한다.

현재 즉시 완료할 수 없는 것은 7~9의 숫자 산출이다. 실제 body mapping, 물리 pivot 위치,
측량점과 새 bag이 필요하기 때문이다. 그 전까지 기존 params를 운영에서 임의 변경하지 않는다.
