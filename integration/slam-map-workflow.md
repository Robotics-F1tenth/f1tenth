# F1TENTH 새 SLAM 지도 생성·저장·적용 절차

이 문서는 처음 작업하는 사람이 실차에서 새 점유지도를 만들고, 저장한 지도를
localization·control·perception·mocap에 적용하는 전체 절차를 설명한다.

검증 기준 환경은 이 저장소의 ROS 2 Humble 구성이다. 차량의 워크스페이스가
`/home/a/f1tenth_ws`처럼 다르면 아래 첫 변수만 실제 경로로 바꾼다. 소스 경로를
명령마다 직접 하드코딩하지 않는다.

```bash
export F1TENTH_WS=/home/a/f1tenth
source /opt/ros/humble/setup.bash
source "${F1TENTH_WS}/install/setup.bash"
```

## 1. 먼저 알아야 하는 것

SLAM에 필요한 TF 연결은 다음과 같다.

```text
map -> odom -> base_link -> laser
 ^        ^                   ^
 SLAM     vesc_to_odom        차량 bringup의 static TF
```

- `/odom` 토픽과 `odom -> base_link` TF는 서로 다르다.
- `/odom`이 출력돼도 TF가 없으면 slam_toolbox는 `/scan`을 odom 좌표로 변환하지
  못한다.
- mapping 중에는 slam_toolbox만 `map -> odom`을 발행해야 한다. MCL, AMCL,
  mocap localization처럼 map TF를 발행하는 노드는 함께 실행하지 않는다.
- 운영 기본값은 `publish_odom_tf:=false`다. mapping할 때만 launch 인자로
  `true`를 전달한다. `vesc_to_odom.cpp`의 기본값이나 차량별 `vesc.yaml`을 수정할
  필요가 없다.
- 새 `.pgm/.yaml`을 저장하는 것과 새 주행 centerline/boundary를 만드는 것은
  별도 작업이다.

### 1.1 기존 인수인계 방식도 가능한가?

가능하다. 기존 인수인계의 핵심은 다음과 같다.

1. `vesc.yaml`의 `publish_tf`를 `true`로 바꾼다.
2. VESC 노드를 재시작해서 `odom -> base_link`를 만든다.
3. slam_toolbox 기본 설정이 요구하는 `base_footprint`를 static TF로 보충한다.
4. 주행 후 `map_saver_cli`로 `.yaml/.pgm`을 저장한다.

다만 현재 저장소에서 그대로 반복하기에는 다음 문제가 있었다.

- mapping 전후로 차량별 `vesc.yaml`을 직접 수정하고 되돌려야 한다.
- `vesc_to_odom.cpp`의 기본값까지 `true`로 바꾸는 것은 YAML이 값을 덮어쓰므로
  중복이며, 소스 재빌드만 늘어난다.
- Humble의 launch 파일명은 `online_async.py`가 아니라
  `online_async_launch.py`다.
- 첨부한 SLAM YAML은 자동으로 로드되지 않으므로 `slam_params_file:=...`을 반드시
  전달해야 한다.
- 인수인계에서 수정하라고 한 `mcl_params.yaml`의 legacy `map_publisher`는 현재
  `mcl.launch.py`에서 실행되지 않는다. 현재 MCL의 지도는 `nav2_map_server`가
  로드한다.

따라서 지도 생성 알고리즘을 고친 것이 아니라, 반복 수동 작업과 현재 launch 구조의
불일치를 다음과 같이 정리했다.

| 변경 파일 | 변경 이유 | mapping에 반드시 필요한가? |
|---|---|---|
| `bringup_launch.py` | `publish_odom_tf:=true`로 mapping 때만 odom TF 활성화 | 수동으로 `vesc.yaml`을 바꾸는 방식을 쓰면 없어도 됨 |
| `sick_bringup_launch.py` | SICK 라이다 차량에도 같은 인자 제공 | SICK 구성을 사용할 때만 해당 |
| `f1tenth_online_async.yaml` | SLAM 기준 프레임을 차량 이동 기준인 `base_link`로 통일 | 기존 `base_frame: laser`도 TF가 연결되면 동작하므로 필수 수정은 아님 |
| `mcl.launch.py` | 실제 map_server의 지도와 MCL 초기 자세를 launch 인자로 선택 | 지도 생성에는 무관하지만 저장한 지도를 현재 MCL에 적용할 때 필요 |

launch 파일에서 제거된 `throttle_interpolator_node` 변수는 원래도
`ld.add_action(...)`이 주석이라 실행되지 않던 코드다. 사용하지 않는 import와 함께
lint 정리한 것이며 차량 동작을 제거한 것이 아니다.

결과적으로 mapping 때 운영 파일이나 C++를 수정하지 않고 다음 한 줄로 필요한 TF를
켤 수 있다.

```bash
ros2 launch f1tenth_stack bringup_launch.py publish_odom_tf:=true
```

저장한 지도도 YAML 파일을 매번 고치지 않고 다음처럼 적용한다.

```bash
ros2 launch als_ros mcl.launch.py \
  map_yaml:=/absolute/path/to/new_map.yaml \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

## 2. 최초 1회 빌드

이 절차에 사용되는 launch 인자를 설치 공간에 반영한다.

```bash
cd "${F1TENTH_WS}"
colcon build --symlink-install --packages-select f1tenth_stack als_ros
source "${F1TENTH_WS}/install/setup.bash"
```

다음 인자가 보여야 한다.

```bash
ros2 launch f1tenth_stack bringup_launch.py --show-args
ros2 launch als_ros mcl.launch.py --show-args
```

- 차량 bringup: `publish_odom_tf`
- MCL: `map_yaml`, `initial_pose_x`, `initial_pose_y`, `initial_pose_yaw`

## 3. Mapping 전 안전 준비

1. 차량을 넓고 평평한 장소에 둔다.
2. 조종기와 비상정지 방법을 먼저 확인한다.
3. 속도 제한을 낮추고 수동 주행만 사용한다.
4. 기존 MCL/AMCL/mocap localization/기존 SLAM을 모두 종료한다.
5. 라이다 시야를 가리는 사람이나 움직이는 물체를 가능한 한 줄인다.

다음 명령에서 기존 localization 노드가 남아 있지 않은지 확인한다.

```bash
ros2 node list
```

## 4. 차량 센서와 odom TF 실행

일반 라이다 bringup은 다음과 같이 실행한다.

```bash
ros2 launch f1tenth_stack bringup_launch.py publish_odom_tf:=true
```

SICK 라이다 구성이라면 대신 다음을 사용한다.

```bash
ros2 launch f1tenth_stack sick_bringup_launch.py publish_odom_tf:=true
```

중요: 실행 중인 노드에 나중에 `ros2 param set`만 하지 않는다. TF broadcaster는
노드 시작 시 만들어지므로 반드시 위 인자를 준 상태로 노드를 시작해야 한다.

다른 터미널에서 실제 적용값을 확인한다.

```bash
source "${F1TENTH_WS}/install/setup.bash"
ros2 param get /vesc_to_odom_node publish_tf
```

결과가 `Boolean value is: True`여야 한다.

## 5. SLAM 실행 전 사전점검

차량을 앞뒤로 아주 조금 움직여 VESC와 servo 메시지가 들어오게 한 다음 확인한다.

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo odom laser
```

각 `tf2_echo`에서 translation/rotation이 반복 출력되면 `Ctrl+C`로 종료한다.
마지막 `odom -> laser`가 실패하면 SLAM을 시작하지 않는다.

정상 상태는 다음과 같다.

- `/scan`과 `/odom`이 한 번 이상 출력된다.
- `base_link -> laser`가 조회된다.
- 차량을 움직이면 `odom -> base_link` 값이 변한다.
- 두 TF가 연결되어 `odom -> laser`가 조회된다.

## 6. slam_toolbox 실행

새 터미널에서 다음 명령을 실행한다.

```bash
source "${F1TENTH_WS}/install/setup.bash"
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="${F1TENTH_WS}/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml" \
  use_sim_time:=false
```

주의 사항:

- Humble의 파일명은 `online_async_launch.py`다.
- 파라미터 인자는 `slam_params_file`이다.
- 실차에는 `/clock`이 없으므로 `use_sim_time:=false`를 명시한다.
- 이 저장소의 SLAM YAML은 `base_frame: base_link`를 사용한다. 별도의
  `base_link -> base_footprint` identity TF는 필요 없다.

## 7. RViz 확인과 주행

```bash
rviz2
```

RViz에서 다음을 설정한다.

- Fixed Frame: `map`
- Map topic: `/map`
- LaserScan topic: `/scan`
- TF 표시 활성화

새 지도를 처음 만드는 mapping에서는 `p`로 초기 자세를 지정할 필요가 없다.
`2D Pose Estimate`는 저장한 지도에서 localization할 때 사용한다.

저속으로 다음처럼 주행한다.

1. 출발 지점 주변을 천천히 한 바퀴 돈다.
2. 같은 구간을 3~4바퀴 반복하되 가능한 한 같은 벽을 여러 각도에서 본다.
3. 급가속·급조향·휠스핀을 피한다.
4. 출발 지점으로 돌아와 loop closure 후 지도가 안정되는지 확인한다.
5. 마지막 정지 후 `map_update_interval`보다 긴 5초 이상 기다린다.

다음 두 항목이 조회돼야 한다.

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map odom
```

벽이 겹쳐 두껍게 보이거나 트랙이 두 갈래로 어긋나면 저장 전에 다시 주행하거나
mapping을 재시작한다.

## 8. 지도 저장

지도 이름에는 장소와 날짜를 포함하는 것을 권장한다.

```bash
export MAP_NAME=lab_20260808
export MAP_DIR="${F1TENTH_WS}/maps/${MAP_NAME}"
mkdir -p "${MAP_DIR}"
```

MCL/map_server용 점유지도를 저장한다.

```bash
ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f "${MAP_DIR}/${MAP_NAME}" \
  --fmt pgm \
  --occ 0.65 \
  --free 0.196
```

SLAM을 나중에 이어서 하거나 slam_toolbox localization을 사용할 가능성이 있으면
pose graph도 저장한다.

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${MAP_DIR}/${MAP_NAME}'}"
```

SLAM을 종료하기 전에 파일을 확인한다.

```bash
ls -lh "${MAP_DIR}"
sed -n '1,12p' "${MAP_DIR}/${MAP_NAME}.yaml"
```

최소한 다음 파일이 있어야 한다.

- `${MAP_NAME}.yaml`과 `${MAP_NAME}.pgm`: MCL/map_server용 점유지도
- `${MAP_NAME}.posegraph`와 `${MAP_NAME}.data`: 이어서 mapping할 때 사용, 선택사항

YAML의 `image:`는 보통 상대경로이므로 `.yaml`과 `.pgm`은 항상 같은 디렉터리로
함께 이동·복사한다.

## 9. 새 지도를 MCL에 적용

파일을 저장소 안으로 옮기지 않아도 절대경로로 바로 시험할 수 있다. 차량의 대략적인
시작 자세를 새 map 좌표계에서 측정해 함께 전달한다. yaw 단위는 **degree**다.

```bash
ros2 launch als_ros mcl.launch.py \
  map_yaml:="${MAP_DIR}/${MAP_NAME}.yaml" \
  initial_pose_x:=0.0 \
  initial_pose_y:=0.0 \
  initial_pose_yaw:=0.0 \
  use_sim_time:=false
```

위의 세 초기값은 예시일 뿐이다. 실제 차량 위치로 바꾸거나, 실행 후 RViz의
`2D Pose Estimate`(`p`)로 `/initialpose`를 보낸다.

정식 지도 자산으로 버전 관리하려면 두 파일을 함께 복사한다.

```bash
cp "${MAP_DIR}/${MAP_NAME}.yaml" \
  "${F1TENTH_WS}/src/f1_localization/als_ros/maps/"
cp "${MAP_DIR}/${MAP_NAME}.pgm" \
  "${F1TENTH_WS}/src/f1_localization/als_ros/maps/"
```

`mcl.launch.py`는 `map_yaml` 인자를 map_server에 직접 전달하므로 더 이상
`mcl_params.yaml`의 legacy `map_publisher` 경로를 수정할 필요가 없다. 단,
`mocap_localization.launch.py with_map:=true`처럼 legacy `map_publisher`를 직접
실행하는 구성은 해당 경로도 별도로 맞춰야 한다.

## 10. Control·Perception·Mocap까지 적용

점유지도만 교체하면 전체 자율주행 스택 전환은 끝나지 않는다.

### 10.1 Control/SMPPI

SMPPI는 `/map`이나 PGM을 직접 읽지 않고 map 좌표계의 centerline CSV를 읽는다.
새 맵에는 다음 형식의 파일이 필요하다.

```text
src/control/smppi_cuda_controller/data/<MAP_NAME>/<MAP_NAME>_centerline.csv
```

현재 CSV는 centerline 좌표뿐 아니라 좌·우 폭과 경계 좌표를 포함한다.

```text
x_m,y_m,w_tr_left_m,w_tr_right_m,w_total_m,left_x_m,left_y_m,right_x_m,right_y_m
```

PGM에서 이 CSV를 자동 생성하는 완성된 파이프라인은 현재 저장소에 없다. 트랙의
center/left/right polyline을 새 map 좌표계에서 준비한 뒤 폭을 계산해야 한다.
그 후 `smppi_cuda_controller/launch/cuda_mppi.launch.py`의 `map_name`을 새 이름으로
맞춘다. controller 파라미터는 기존 map1 전용 튜닝일 수 있으므로 저속 재검증한다.

### 10.2 Perception

Perception은 `/map`의 occupied cell을 벽 필터에 사용하면서 별도의 lane CSV도
읽는다. 다음 파일을 새 map 좌표계로 만든다.

```text
src/perception/data/lane/<MAP_NAME>_path.csv
```

실행할 때 basename을 전달한다.

```bash
ros2 launch f1_perception f1_perception.launch.py \
  csv_path:="${MAP_NAME}_path"
```

control의 `/mppi_target_path`와 좌·우 boundary topic도 동일한 새 map 좌표계인지
확인한다.

### 10.3 Mocap localization

새 SLAM 지도는 기존 지도와 원점·yaw가 다르므로 기존 `T_map_mocap`을 그대로 쓰면
mocap pose가 체계적으로 이동하거나 회전한다. 다음 순서로 다시 정합한다.

1. 새 지도에서 확인 가능한 survey 대응점을 준비한다.
2. `mocap_localization/scripts/calibrate_shared_map.py`로 shared
   `T_map_mocap` candidate를 만든다.
3. 차량별 `T_tracked_base`와 shared `T_map_mocap`을 분리해서 유지한다.
4. production YAML을 바로 덮어쓰지 말고 candidate/shadow 비교 후 적용한다.

상세 절차는
`src/localization/mocap_localization/docs/two_vehicle_mocap_pipeline.md`를 따른다.

## 11. 최종 검증 체크리스트

### Localization

- [ ] map_server가 새 YAML/PGM을 오류 없이 로드한다.
- [ ] RViz 지도와 실제 벽/라이다가 겹친다.
- [ ] 초기 자세 지정 후 MCL pose가 반대편 트랙으로 점프하지 않는다.
- [ ] localization 중 `map -> base_link` TF 발행자는 하나뿐이다.

### Control

- [ ] centerline과 좌·우 경계가 새 map 위에 정확히 표시된다.
- [ ] CSV의 좌·우 폭 부호와 단위가 맞다.
- [ ] 저속 수동/섀도 검증 후에만 autonomous control을 켠다.

### Perception

- [ ] `/map`을 transient-local QoS로 수신한다.
- [ ] 새 lane CSV와 boundary topic이 새 map 좌표계에 있다.
- [ ] 벽이 장애물로 대량 오검출되지 않는다.

### Mocap

- [ ] 두 차량이 동일한 shared `T_map_mocap`을 사용한다.
- [ ] 정지 위치와 yaw를 여러 지점에서 map/MCL과 비교한다.
- [ ] 보정 candidate를 검증하기 전 기존 production 값을 덮어쓰지 않는다.

## 12. 자주 발생하는 문제

| 증상 | 원인 | 확인/해결 |
|---|---|---|
| `/odom`은 있지만 SLAM이 scan transform 실패 | `odom -> base_link` TF 없음 | bringup을 `publish_odom_tf:=true`로 재시작하고 `tf2_echo odom laser` 확인 |
| `base_footprint` frame이 없다고 나옴 | 첨부 YAML이 아닌 slam_toolbox 기본 YAML 사용 | `slam_params_file:=.../f1tenth_online_async.yaml` 확인 |
| 지도나 차량 pose가 흔들림 | SLAM과 MCL/AMCL이 동시에 map TF 발행 | mapping 중 다른 localization 노드 종료 |
| SLAM이 시간 대기 상태 | 실차에서 `use_sim_time:=true` | `use_sim_time:=false`로 재시작 |
| 저장했는데 control 경로가 옛 트랙 | SMPPI는 PGM이 아닌 centerline CSV 사용 | 새 centerline/boundary CSV 생성 및 `map_name` 변경 |
| perception 벽 필터가 이상함 | `/map`과 lane CSV 좌표계 불일치 | 둘 다 새 map 좌표계로 교체 |
| mocap pose가 일정하게 회전/이동 | 기존 지도용 `T_map_mocap` 사용 | shared map alignment 재보정 |

## 13. 이전 지도로 되돌리기

원래 YAML과 초기 자세를 launch 인자로 다시 지정하면 된다.

```bash
ros2 launch als_ros mcl.launch.py \
  map_yaml:="${F1TENTH_WS}/src/f1_localization/als_ros/maps/map1.yaml" \
  initial_pose_x:=-0.30 \
  initial_pose_y:=-9.16 \
  initial_pose_yaw:=174.0 \
  use_sim_time:=false
```

control/perception/mocap 자산도 반드시 같은 이전 지도 세트로 함께 되돌린다.
