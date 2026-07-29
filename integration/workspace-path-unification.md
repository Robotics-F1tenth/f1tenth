# 워크스페이스 통합 · 저장소 정리 계획

최초: 2026-07-23 · 갱신: 2026-07-29 (driver 이식 로컬 커밋 완료, push/서브모듈 등록 대기)
워크스페이스: `~/f1tenth_ws` (4호차)

> **범위 조정**: driver 이식 검증 중 발견한 "deadman 떼도 계속 움직이는" 문제는
> 코드 통합과 무관한 별도 이슈라 [vesc-deadman-command-timeout.md](vesc-deadman-command-timeout.md)로
> 분리했다. 이 문서는 저장소 통합·이관에만 집중한다.

---

## 0. 현재 상태

**rename 완료.** 4호차가 `~/f1tenth` → `~/f1tenth_ws`로 바뀌어 3호차와 이름이 통일됐다.
센서/IMU/MCL/perception 정상, 제어도 저속 정상.

### rename으로 부수적으로 해결된 것

이전 조사에서 "3세대 잔재"로 분류했던 `/home/a/f1tenth_ws/...` 참조 9곳이
**이름이 바뀌면서 다시 유효해졌다.** 레이아웃 부분(`src/localization/f1_localization`
→ `src/f1_localization`)도 이미 갱신돼 있어, 실제로 깨진 경로는 3개뿐이고 전부 inert:

| 경로 | 왜 inert |
|---|---|
| `als_ros/logs/pose_log.txt` | `write_pose: false` |
| `als_ros/maps/slam_map/` | SLAM 전용 |
| `navigation2/nav2_bringup/params` | `amcl_launch.py` 폴백, 미사용 |

→ **절대경로 제거(Phase D)의 긴급도는 내려갔다.** 다음 rename 때 또 깨지므로 중기 과제로는 유효.
지금은 저장소 정리가 훨씬 급하다.

### 정리할 잔여물

`/home/a/f1tenth/` 가 남아있다. 내용은 `mocap_localization.cpp` 1개뿐이고
워크스페이스 버전이 그것의 상위집합(calibration_v2 관련 코드 추가됨)임을 확인.
→ **삭제 안전.**

---

## 1. 저장소 연결 분석

트리 내 git 저장소 **8개**(최상위 포함). 최상위는 `Robotics-F1tenth/f1tenth`.

### 연결됨 — 서브모듈 5개

| 경로 | 원격 | 브랜치 | 상태 |
|---|---|---|---|
| `src/f1_localization` | `heojy/f1_localization` | main | **dirty 8** (MCL 작업 미커밋) |
| `src/perception` | `heojy/f1_perception` | `feature/acc-yaw-estimation` | clean ✓ |
| `src/racecar_simulator` | `JangJunhyeok21/racecar_simulator` | detached HEAD | clean |
| `src/control/mppi_controller` | `JangJunhyeok21/mppi-controller` | main | **작업트리 비어있음 (`.git`만)** |
| `src/control/smppi_cuda_controller` | `JangJunhyeok21/smooth-mppi-cuda` | main | dirty 5, **브랜치 불일치** |

문제 2건:

- **`mppi_controller` 내용물이 없다.** 파일 7개가 전부 삭제 스테이징 상태.
  `submodule update --init`이 안 됐거나 내용이 지워진 것.
  쓰는 패키지면 복구, 안 쓰면 서브모듈 해제.
  **방치하면 clone하는 사람마다 빈 패키지를 받는다.**
- **`smppi_cuda_controller` 브랜치 불일치.** `.gitmodules`는 `feature/obstacle`인데 실제는 `main`.
  둘 중 뭐가 맞는지 정해 일치시켜야 한다.

### 연결 안 됨 — 3개

| 경로 | 원격 | 상태 | 위험 |
|---|---|---|---|
| `src/f1tenth_system` | `Robotics-F1tenth/f1tenth_system` | **미푸시 커밋 2 + dirty 2** | **최고** |
| `src/vesc` | `f1tenth/vesc` (상류) | clean, 로컬커밋 0 | 낮음 |
| `src/ackermann_mux` | `f1tenth/ackermann_mux` (상류) | clean, 로컬커밋 0 | 낮음 |

**`src/f1tenth_system`이 가장 위험하다:**

```
30946d3 fix(odom): cancel vesc_to_odom's hardcoded speed negation, retune vesc params
73c9f0d config(teleop): cap manual speed at 1.5 and disable the unused default profile
+ 미커밋: joy_teleop.yaml (speed 1.5→4.5, steering 0.34→0.4788), vesc.yaml (주석)
```

푸시도 안 됐고 서브모듈 등록도 안 돼 있다.
→ **`git clone --recursive`로 최상위를 받아도 주행에 필수인 드라이버 설정이 하나도 안 따라온다.
이 작업은 현재 4호차 디스크에만 존재한다.**

`vesc`/`ackermann_mux`는 상류 그대로라 급하진 않지만 **핀 고정이 안 돼 있다.**
재clone 시 상류가 바뀌면 `vesc_to_odom.cpp`의 하드코딩 부호 반전이 달라져 odom 부호 문제가 재발할 수 있다.

### 중복 패키지 판정 — 뭐가 실제로 쓰이나 (확정)

`vesc`/`ackermann_mux`가 두 군데 있는데, **최상위 `src/` 쪽만 쓰인다.**

| 경로 | 실체 | 빌드됨? |
|---|---|---|
| `src/vesc/*` | vesc, vesc_driver, vesc_msgs, vesc_ackermann | **O — 실사용** |
| `src/ackermann_mux` | ackermann_mux | **O — 실사용** |
| `src/f1tenth_system/vesc` | **파일 0개** | X |
| `src/f1tenth_system/ackermann_mux` | **파일 0개** | X |
| `src/f1tenth_system/teleop_tools` | **파일 0개** | X (joy_teleop은 시스템 1.7.0 사용) |

근거: `build/*/CMakeCache.txt`의 소스 경로가 전부 `src/vesc/…`, `src/ackermann_mux`를 가리킴.

f1tenth_system 안의 3개는 **f1tenth_system 자체 `.gitmodules`에 선언된 서브모듈인데 init이 안 된 빈 디렉터리**다.
→ **커밋 대상 아님.** 신경 쓸 것은 `src/f1tenth_system/f1tenth_stack` 뿐이다.

> ⚠️ **서브모듈 등록 시 함정**: `f1tenth_system`을 최상위 서브모듈로 등록한 뒤 누군가
> `clone --recursive` 하면 저 3개가 **채워지면서** `src/vesc`·`src/ackermann_mux`와
> 패키지 이름이 충돌해 `colcon build`가 "Multiple packages found with the same package name"으로 실패한다.
> → **fork인 `Robotics-F1tenth/f1tenth_system`의 `.gitmodules`에서 이 3개 선언을 제거**하는 것이 정답.
> (이 워크스페이스는 최상위 `src/vesc`·`src/ackermann_mux`를 쓰므로 중첩본은 애초에 불필요)

### 최상위 미추적

| 대상 | 내용 |
|---|---|
| `config/` | `imu_params.yml` (IMU 시리얼 포트 고정) |
| `integration/` | 이 문서 |
| `src/control/pacejka_sysid/` | 신규 패키지 |
| `src/localization/mocap_localization/calibration_v2.py` | mocap 캘리브 |

### 최상위 스테이징 (미커밋)

`src/localization/f1_localization` 구 복사본 삭제 73파일 + mocap 수정.
`src/localization/mocap_localization`은 유지된다. → 별도 커밋으로 확정 필요.

---

## 2. 드라이버 세팅 전략 (핵심 설계 결정)

### 문제

3호차 수동조작이 4호차보다 낫다 → 가져오고 싶다.
그런데 `f1tenth_stack/config/vesc.yaml`에는 **성격이 다른 두 종류**가 섞여 있다:

| 종류 | 예시 | 공유 가능? |
|---|---|---|
| **차량별 물리 캘리브레이션** | `steering_angle_to_servo_offset: 0.44105`<br>`steering_angle_to_servo_gain_left/right`<br>`speed_to_erpm_gain: 3806.35` | **불가** — 서보 중립·모터·기어비가 개체마다 다름 |
| **구조/로직 수정** | `vesc_to_odom_node`의 부호 반전 취소<br>`port: /dev/sensors/vesc` | 가능 — 코드 버그 회피, udev 규약 |
| **운전 취향/한계** | joy_teleop `scale`(속도 4.5, 조향 0.4788)<br>`throttle_interpolator` 유무 | 대체로 가능 |

**`vesc.yaml`을 통째로 복사하면 서보 중립까지 넘어와 4호차가 비뚤게 달린다.**

### 원인 규명 완료 — 코드가 yaml을 무시하고 있었다

`~/Downloads/driver/message.txt`(3호차 driver 세트 분석, 2026-07-28)의 진단을 **직접 검증했고 맞다.**

`vesc.yaml`에는 이 차의 실측 좌/우 조향 게인이 들어 있는데:

```yaml
steering_angle_to_servo_gain_left:  -1.05417
steering_angle_to_servo_gain_right: -1.34250
steering_angle_to_servo_gain:       -1.0
```

현재 `src/vesc`는 f1tenth 업스트림 pristine이라 **`_left`/`_right`를 declare하지 않는다.**
검증: `ackermann_to_vesc.cpp:54`에 `declare_parameter<double>("steering_angle_to_servo_gain")` 하나뿐.
ROS 2는 미선언 파라미터를 **에러도 경고도 없이 무시**하므로 실제 적용값은 `-1.0` 하나다.

3호차 driver 버전에는 좌/우 분기가 있다 (검증: `driver/motor/vesc_ackermann/src/ackermann_to_vesc.cpp:53-56, 79-83`).

**증상**: 좌우 조향 비대칭. 한쪽은 스틱 90%에서 이미 풀락, 반대쪽은 끝까지 밀어도 최대 타각에 못 감
(계산: `+0.4788+0.44105 = 0.920` < `servo_max 0.99`).

→ **"코드는 3호차에서, 캘리브레이션은 현재 값 유지"가 정확한 처방이다.**

### ⛔ 함께 처리해야 하는 것 — odom 부호

driver 버전은 `vesc_to_odom.cpp`의 하드코딩 음수화를 **제거**했다 (검증 완료):

| | `vesc_to_odom.cpp` | yaml gain | 결과 |
|---|---|---|---|
| 현재 ws | `(-state.speed - offset)/gain` 음수화 **있음** | `-3806.35` | 정상 (상쇄) |
| 이식 후 yaml 그대로 | 음수화 **없음** | `-3806.35` | ⛔ **부호 반전 → MCL 붕괴** |
| 이식 후 yaml 정정 | 음수화 없음 | **`+3806.35`** | 정상 |

**이식과 부호 정정은 반드시 동시에.** 크기 `3806.35`는 그대로이므로 캘리브 변경이 아니라 코드 변경에 딸린 정정이다.

### 이식 범위 (message.txt §5 검증 결과 동의)

**가져올 것 — `driver/motor/` → `src/vesc/` 소스 4개뿐:**
- `vesc_ackermann/src/ackermann_to_vesc.cpp` + `include/.../ackermann_to_vesc.hpp` (좌/우 게인, 핵심)
- `vesc_ackermann/src/vesc_to_odom.cpp` (음수화 제거 + QoS)
- `vesc_driver/src/vesc_driver.cpp` (커맨드 QoS 10→1)

**가져오면 안 되는 것 (특히 위험한 2개):**
- `mux.yaml` — driver는 `input_mux/*`를 구독하는데 ws의 joy_teleop은 `teleop`으로 발행
  → **수동조작이 완전히 죽는다.** 게다가 nav·joy 둘 다 `priority:100` 동점이라 deadman 우선권 소멸(안전 회귀).
  현재 ws는 nav 10 / joy 100 (검증함) — **유지할 것.**
- `sensors.yaml` — driver는 `/dev/ttyLIDAR` 시리얼, 이 차는 이더넷 `192.168.0.10:10940`
- `joy_teleop.yaml`·`vesc.yaml` — 캘리브레이션 (3호차 값으로 덮임)
- `teleop_tools` — driver는 1.2.1, 현재 시스템 1.7.0 사용 중 → **다운그레이드**
- `f1tenth_system/ackermann_mux`, `motor/vesc`, `motor/vesc_msgs` — 내용 동일, 이름만 중복

**보류**: `lidar`(urg_node2) / `imu`(stella_ahrs) / `new_imu`(microstrain)
— 앞 둘은 현재 불필요, microstrain은 이미 시스템에 설치돼 있어 src에 넣으면 가린다.

> message.txt가 배제한 항목도 확인함: `throttle_interpolator`는 **양쪽 bringup 모두 주석 처리**라
> 조작감 차이의 원인이 아니다. (내가 앞서 유력 후보로 지목했던 가설은 틀렸다.)

### 드라이버를 git에 올릴 것인가 → **올린다. 단 분리해서.**

"개발 코드만 올리고 드라이버는 빼자"는 선택지는 **이번 사건이 정확히 그 결과다.**
`src/vesc`가 버전관리 밖(embedded repo)이라 좌/우 게인 수정이 3호차에만 있었고,
4호차는 1년 가까이 조향이 비대칭인 채로 굴러갔는데 **아무도 몰랐다.** 에러가 안 나니까.

올리지 않으면 이번 이식도 손으로 복사한 게 되고, 5호차에서 또 반복된다.

**단 성격에 따라 두는 곳을 나눈다:**

```
src/vesc/  (fork → 서브모듈)          ← 코드. 좌/우 게인 분기, odom 음수화 제거, QoS
src/f1tenth_system/  (fork → 서브모듈)
  f1tenth_stack/config/vesc.yaml       ← 구조·로직 (port, mux 우선순위, 공용 기본값)
  f1tenth_stack/config/joy_teleop.yaml ← 조작 프로파일

config/  (최상위, 차량별)
  imu_params.yml                        ← 이미 있음
  car4.yaml                             ← 서보 오프셋, 좌/우 게인, erpm gain, imu_rotation
```

`car4.yaml`에 들어갈 것 = **개체마다 실측해야 하는 값**:
`steering_angle_to_servo_offset`, `steering_angle_to_servo_gain_left/right`,
`speed_to_erpm_gain`, IMU 축 보정.

launch에서 `parameters=[vesc.yaml, car4.yaml]` 순으로 넘기면 뒤가 앞을 덮는다.
이번에 `mcl.launch.py`에서 쓴 **"yaml 뒤에 오버라이드를 덧붙이는"** 패턴과 동일.
차량 선택은 launch 인자(`car:=car4`) 또는 환경변수.

> 이 구조를 갖추기 전까지는 **개별 값만 손으로 옮기고 파일 통째 복사는 금지.**

`src/vesc`는 현재 f1tenth 업스트림 pristine이다. 이식하면 상류와 갈라지므로
**`Robotics-F1tenth/vesc` fork를 만들어 서브모듈로 등록**해야 한다.
(그래야 3호차도 `git pull`로 같은 코드를 받는다)

---

## 3. 실행 계획

### Phase A — 유실 방지 (최우선, 지금)

**중요한 원칙**: `f1_localization`/`perception`은 **이미 등록된 서브모듈**이라 "서브모듈 등록" 단계가 없다.
`vesc`(오늘 새로 이식)처럼 미등록인 것만 등록이 필요하다. 그리고 **등록/포인터 갱신은 반드시 push 이후**다 —
최상위의 서브모듈 포인터는 커밋 SHA를 가리키는데, 그 SHA가 원격에 없으면 3호차가 `git pull`해도 못 받는다.
순서: **저장소 안에서 commit → push → (신규면) 최상위에 등록 → 최상위에서 포인터 커밋.**

진행 상황 (2026-07-29):

| 저장소 | 로컬 커밋 | 푸시 | 최상위 등록 |
|---|---|---|---|
| `src/f1_localization` | ✅ `9776585` | ⬜ 대기 (사용자 확인 후) | 이미 등록됨, 포인터 갱신만 남음 |
| `src/perception` | ✅ `57cfe1f` (이전 세션에 커밋됨, fetch로 상류 안 뒤처짐 확인) | ⬜ 대기 | 이미 등록됨, 포인터 갱신만 남음 |
| `src/vesc` | ✅ `d53d999` | ⬜ 대기 (remote를 포크로 변경 필요) | ⬜ 미등록 |
| `src/f1tenth_system` | ✅ `e288c86` (누적 3개) | ⬜ 대기 | ⬜ 미등록 |

남은 작업:

1. **`src/f1_localization` push** (커밋 완료, 브랜치 `main`)
2. **`src/perception` push** (커밋 이미 있었음, 뒤처지지 않음 확인 완료 — 바로 push 가능)
3. **`src/vesc` remote를 포크로 변경 후 push**
   ```
   git -C src/vesc remote set-url origin git@github.com:Robotics-F1tenth/vesc.git
   git -C src/vesc push origin ros2
   ```
4. **`src/f1tenth_system` push** (origin 이미 포크 주소, `.pyc`/build 정리도 커밋에 포함됨)
5. **`src/control/smppi_cuda_controller`** dirty 5 → 커밋 여부 판단 (별도)
6. 1~4 푸시 후, 최상위에서:
   - `vesc`·`f1tenth_system` 서브모듈 등록 (Phase B-1, B-2)
   - `f1_localization`·`perception` 포인터 갱신 커밋
   - 재구성 확정(스테이징된 73파일 삭제) + 신규 추가(`config/`, `integration/`, `pacejka_sysid`, `calibration_v2.py`)는
     위와 **분리된 별도 커밋**으로 (성격이 다름)

### Phase A' — 3호차 driver 코드 이식 (수동조작 수정)

Phase A와 독립. 차를 **들어올린 상태**에서 검증할 것.

1. **이식 전 증거 확보** (§2 진단의 직접 증명):
   ```
   ros2 param get /ackermann_to_vesc_node steering_angle_to_servo_gain_left
   ```
   → 지금은 "parameter not declared"로 **실패해야 정상**
2. `src/vesc` 백업 (embedded repo라 ws의 `git checkout`으로 못 되돌림)
3. `driver/motor/` → `src/vesc/` 소스 4개 덮어쓰기 (`.git`, `__pycache__` 제외)
4. **`vesc.yaml`의 `vesc_to_odom_node` 블록 `-3806.35` → `+3806.35`** (3과 동시에)
   주석도 "driver의 음수화를 제거했으므로 부호를 되돌림"으로 갱신
   > ⚠️ **`steering_angle_to_servo_gain`(단일)은 지우지 말 것.** 이식 후 `ackermann_to_vesc`는
   > 안 쓰지만 **`vesc_to_odom.cpp:68`이 여전히 declare한다**(서보 명령을 역산해 각속도 추정).
   > 지우면 `UninitializedStaticallyTypedParameterException`으로 `vesc_to_odom_node`가 즉시 죽는다.
   > (실제로 밟았던 함정)
5. `colcon build --packages-select vesc_ackermann vesc_driver --symlink-install`
6. 검증 **(2026-07-29 실행 완료 — 아래는 실측값)**:
   - 이식 전 `param get …_left` → `Parameter not set`, 단일 gain만 `-1.0` ✔ 진단 확정
   - 이식 후 → `left=-1.05417`, `right=-1.3425`, odom gain `+3806.35` ✔
   - 속도 0으로 조향만 명령해 실측한 `/commands/servo/position`:

     | 스틱 | 이식 전 | 이식 후 | 판정 |
     |---|---|---|---|
     | 좌 최대 (+0.4788) | −0.038 → clip 0.01 | **−0.0637 → clip 0.01** | 풀레인지 |
     | 우 최대 (−0.4788) | **0.920 (풀락 미달)** | **1.0838 → clip 0.99** | 풀레인지 |

     → **좌우 대칭 회복.** 사용자 관측(좌회전만 클리핑)과 정확히 일치했다.
   - **clipping 경고는 사라지지 않는다** — message.txt §7.3의 "경고 소멸" 서술은 자기 계산과 모순.
     게인이 풀스틱에서 서보 한계를 살짝 넘도록 잡혀 있어(끝단 도달 보장) **양쪽 끝에서 대칭적으로 클립**된다.
     정상 동작이며, 판정 기준은 "경고 없음"이 아니라 **"양쪽 다 0.01/0.99에 도달"**이다.
   - **odom 부호 — 바퀴 공중 상태에서 실측 확인**:

     | 명령 | ERPM | `/odom` linear.x | 검산 (ERPM/3806.35) |
     |---|---|---|---|
     | 전진 1.0 | **+4462** | **+1.1723** | +1.172 ✔ |
     | 후진 −1.0 | **−4493** | **−1.1804** | −1.180 ✔ |
     | 정지 | 0 | 0 | ✔ |

     전진에 양수. 크기도 `speed_to_erpm_gain`과 정확히 일치.
   - **mux 우선권 확인**: `/drive`(prio 10)와 `/teleop`(prio 100)을 동시 발행하니
     `/ackermann_cmd`에 **joy 값만** 나옴(48샘플 연속, 섞임 없음). deadman 우선권 정상.
   - 조향 포화 스틱 비율 — 좌우 **동일 85.4%** (이식 전: 좌 90.0% / 우 도달 불가):
     좌 `0.43105/1.05417 = 0.4089 rad`, 우 `0.54895/1.34250 = 0.4089 rad`.
     서보 중립이 0.44105로 기하학적 중앙(0.5)이 아니라서 좌우 게인이 다른 것이고,
     **두 게인 모두 필요 스트로크 대비 17% 오버슈트로 동일**하다 → 설계상 대칭.

   > 측정 팁: `ros2 topic pub -1`은 discovery 경합으로 메시지가 유실된다.
   > 포트 이후 커맨드 토픽 QoS가 `KeepLast(1)`이라 더 잘 드러난다. `-r 20`으로 유지 발행할 것.
   > `ros2 topic echo --once`도 구독 시점 경합이 있어 `/odom`이 0으로 잡힐 수 있다 — 연속 로깅으로 확인.

### Phase B — 저장소 구조 정리

1. **`src/f1tenth_system` 서브모듈 등록** (자기 조직 fork라 바로 가능)
   ```
   git submodule add git@github.com:Robotics-F1tenth/f1tenth_system.git src/f1tenth_system
   ```
   **동시에 fork의 `.gitmodules`에서 `vesc`/`ackermann_mux`/`teleop_tools` 선언 제거**
   (§1 중복 패키지 판정의 ⚠️ 참조 — recursive clone 시 이름 충돌 방지)
2. **`src/vesc` fork + 서브모듈 등록** — Phase A' 이식으로 상류와 갈라지므로 필수.
   `Robotics-F1tenth/vesc` 생성 → 이식 커밋 푸시 → 서브모듈 등록
3. **`mppi_controller` 제거** (사용 안 함 확정):
   ```
   git submodule deinit -f src/control/mppi_controller
   git rm -f src/control/mppi_controller
   rm -rf .git/modules/src/control/mppi_controller
   ```
   → 빈 서브모듈이 사라져 clone하는 사람이 빈 패키지를 받지 않게 된다
4. **`smppi_cuda_controller` 브랜치 일치**: 개발 중인 패키지.
   `.gitmodules`의 `feature/obstacle`을 실제 HEAD인 `main`에 맞추거나, 반대로 브랜치를 되돌린다
5. **`ackermann_mux`**: 상류 그대로(수정 없음)라 당분간 유지. **현재 해시를 부록 B에 기록**

### Phase C — 3호차 이관

Phase A·B 완료 후. **이름이 이미 같으므로 rename 단계가 사라져 난이도가 크게 낮아졌다.**

1. **3호차 로컬 변경 감사 먼저.** 전 저장소 `git status` + `git log @{u}..HEAD`.
   3호차에만 있는 미푸시 작업(특히 드라이버 세팅)을 먼저 건져낸다.
2. 백업: `~/f1tenth_ws` 통째로 복사
3. `git pull` + `git submodule update --init --recursive`
4. **`rm -rf build/ install/ log/` 후 전체 재빌드** (`cc` alias)
5. **차량별 확인 — 4호차 값을 그대로 가정하지 말 것:**
   - IMU 축: 정지 시 `linear_acceleration.z`가 `-9.8`이면 `imu_rotation_roll:=180`, `+9.8`이면 `0`
   - odom 부호: 전진 시 `/odom` `twist.linear.x`가 양수인지
   - 서보 중립/조향 게인: 3호차 값 유지
6. **udev 룰 수동 적용** (git 밖):
   `/etc/udev/rules.d/99-vesc6.rules`에 `ATTRS{product}=="ChibiOS/RT Virtual COM Port"` 추가
   → `udevadm control --reload-rules`
   VESC와 IMU가 같은 `0483:5740`이라 없으면 `/dev/sensors/vesc`가 IMU를 가리킬 수 있다.
   (4호차 백업: `99-vesc6.rules.bak`)

### Phase D — 절대경로 제거 (중기)

rename으로 급한 불은 꺼졌으므로 후순위.

- `als_ros/params/map.yaml` `yaml_filename` → `map_name` launch 인자(기본 `map1`) +
  `get_package_share_directory('als_ros')` 조합. bag별 맵 교체 문제도 같이 해결
- `slam_bringup_launch.py:39`, `map_autosaver.py:10`, `amcl_launch.py:6` → 패키지 상대경로
- `mcl_params.yaml`의 `mae_classifier_dir` / `pose_log_file` / map_publisher 경로 →
  share 기준. **`CMakeLists.txt`에 `install(DIRECTORY classifiers ...)` 추가 필요**
  (현재 `launch/ params/ maps/`만 설치됨)
- `MCL.h` / `map_publisher.cpp`의 1세대 절대경로 기본값 제거 (헤더 → 재빌드 필요)
- **alias를 `scripts/aliases.sh`로 이전**, `~/.bashrc`는 `source ~/f1tenth_ws/scripts/aliases.sh` 한 줄만
  ```bash
  F1TENTH_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  alias mcl="cd $F1TENTH_WS && source install/setup.bash && ros2 launch als_ros mcl.launch.py"
  ```
  같이 정리: 중복 정의(`sis`,`cb`,`eb`,`sb`가 2번씩), 깨진 `resource` 경로(`src/driver/...`),
  `lidar`(urg_node2) vs `sensors`(urg_node) 불일치 확인

---

## 4. 검증

```bash
# 저장소 연결 상태 한눈에
git submodule status --recursive

# 미푸시 작업 탐지 (전 저장소)
for d in . src/*/ src/control/*/; do
  [ -e "$d/.git" ] && { echo "== $d"; git -C "$d" log --oneline @{u}..HEAD 2>/dev/null; }
done

# 경로 잔존 (빌드산출물 제외)
grep -rn "/home/a/f1tenth[^_]" src/ config/ | grep -vE "/(build|install|log)/|__pycache__"
```

주행 검증 (양 차량 동일):
1. `sensors` → `imu` → `mcl`
2. `ros2 run tf2_ros tf2_echo map laser` — 벽시계 타임스탬프로 갱신
3. 좌회전 시 rviz pose가 **같은 방향**으로 회전
4. 전진 시 `/odom` `twist.linear.x` **양수**, 추정도 전진
5. bag: `bag_mcl.launch.py` + `ros2 bag play --clock`
   구 bag은 `imu_name:=ekf/imu/data imu_rotation_roll:=0.0`

---

## 5. 리스크

| 리스크 | 영향 | 완화 |
|---|---|---|
| **`f1tenth_system` 미푸시 + 미등록** | 드라이버 설정 전량 유실, 3호차 이관 불가 | **Phase A-1 최우선** |
| **driver 이식 시 odom 부호 정정 누락** | 전진 시 `/odom` 음수 → **MCL 붕괴** | Phase A'-4를 이식과 **동시에** |
| `mux.yaml` 이식 | **수동조작 완전 정지** + deadman 우선권 소멸 | 이식 목록에서 제외 (§2) |
| `vesc.yaml` 통째 복사 | 서보 중립이 넘어와 차가 비뚤게 달림 | 개체값/공유값 분리 (§2) |
| **f1tenth_system recursive clone** | 중첩 vesc/ackermann_mux가 채워져 패키지명 충돌 → 빌드 실패 | fork `.gitmodules`에서 3개 선언 제거 (Phase B-1) |
| `mppi_controller` 빈 서브모듈 | clone하는 사람마다 빈 패키지 | Phase B-3에서 제거 |
| `ackermann_mux` 미핀 | 상류 변경 시 예기치 않은 동작 | 해시 기록 |
| 3호차 미푸시 작업 유실 | 작업 손실 | Phase C-1 감사 선행 |
| `MCL.h` 재빌드 누락 | 조용히 구 동작 | 커밋 메시지에 명시 |

**되돌리기**: Phase A~B는 각 저장소 `git revert`. Phase C는 3호차 백업 복원.

---

## 부록 A — 커밋 메시지 (실제 커밋됨)

이 절의 메시지는 초안이 아니라 **실제로 로컬 커밋에 사용된 것**이다 (2026-07-29). `git log -1`로 확인 가능.

### `src/f1_localization` — commit `9776585`

```
fix(mcl): correct IMU yaw rate and split bag/live config into launch args

MCL takes heading purely from integrating the IMU's angular_velocity.z, but on
this rig the IMU's published frame has z pointing down: at rest the
accelerometer reads z = -9.8, and rotating the car left drives
angular_velocity.z negative. The raw gyro z is therefore the negative of the
vehicle yaw rate, so the estimate turned the wrong way and diverged within a
lap of live driving. Bag replay was unaffected because the older bags carry an
already-vehicle-frame IMU on ekf/imu/data -- which is why those runs scored
13cm while live fell apart.

imu_rotation_yaw/pitch/roll already existed for exactly this but were dead:
declared, read, never applied. imuCB now rotates the gyro by them before
integrating z. Values are degrees, matching initial_pose_yaw.

use_sim_time and imu_name also differ between bag and live and were hardcoded
in the params files, which silently broke live runs after bag-verified
commits. Live publishes no /clock, so use_sim_time:=true froze the node clock
at 0 and stamped map->base_link at t=0 while /scan ran on the wall clock; with
use_odom_tf:=true it blocked forever in lookupTransform instead. imu_name
pointed at a topic that only exists in the bags. All three are launch
arguments now and gone from the yaml, so the launch file is the only source:

  mcl.launch.py     (live) use_sim_time:=false imu_name:=/imu/data
  bag_mcl.launch.py (bag)  use_sim_time:=true
  both default imu_rotation_roll:=180.0

Replay the older bags with imu_name:=ekf/imu/data imu_rotation_roll:=0.0.

Also: amcl_launch.py, slam_bringup_launch.py, map_autosaver.py had a leftover
/home/a/f1tenth_ws/src/localization/f1_localization/... path from before this
repo moved to src/f1_localization directly. Updated to match; both are inert
(SLAM/amcl tooling, not used by the live mcl path) so this is a path fix only.

Header change -- rebuild als_ros.
```

### `src/vesc` — commit `d53d999`

```
feat: split steering gain per side, drop odom negation, tighten command QoS

Ported from the 3호차 driver set. Three independent changes, all in the
command/odometry path.

ackermann_to_vesc: steering used a single steering_angle_to_servo_gain, but the
linkage is not symmetric and the servo centre (0.44105) is not the geometric
middle, so one gain cannot reach both endpoints. The cars' vesc.yaml already
carried measured _left/_right gains -- undeclared here, so ROS 2 dropped them
silently and only the single -1.0 applied. Measured on 4호차: full left hit the
servo minimum at 85% stick while full right topped out at 0.920 and never
reached the 0.99 limit, i.e. visibly asymmetric steering with no error anywhere.
Now branches on the sign of steering_angle and each side saturates at the same
85.4% of stick travel.

vesc_to_odom: speed was computed as (-state.speed - offset) / gain. That
negation does not hold for these cars -- driving forward commands about +2200
eRPM and the VESC reports about +2664, both positive -- so /odom came out
negative while moving forward. Removed. Callers must use a positive
speed_to_erpm_gain; a negative one was being used to cancel this and now has to
go back to positive, or odometry flips again.

vesc_driver / vesc_to_odom / ackermann_to_vesc: command-path QoS depth 10 -> 1.
joy publishes at autorepeat_rate 20Hz, so a stalled serial write could queue up
to half a second of stale commands and replay them.

Verified on 4호차 with the wheels off the ground: left/right servo endpoints
0.01/0.99 both reached, forward +4462 eRPM -> /odom +1.1723, reverse -4493 ->
-1.1804, mux still hands priority to the joystick over nav.
```

### `src/f1tenth_system` — commit `e288c86`

```
config: restore positive odom gain, document the steering gain split

Companion to the vesc driver port (Robotics-F1tenth/vesc). Config only.

vesc_to_odom_node speed_to_erpm_gain -3806.35 -> 3806.35. It was negative only
to cancel the hardcoded negation in vesc_to_odom.cpp; that negation is gone
now, so the two would otherwise cancel out and flip odometry again. Magnitude
is unchanged -- this is not a recalibration.

steering_angle_to_servo_gain (single) is kept even though ackermann_to_vesc no
longer reads it: vesc_to_odom still declares it to invert the servo command
when estimating angular velocity, and the node aborts on startup without it.
Noted inline so it does not look like dead config.

Manual drive scales: speed 1.5 -> 4.5 m/s and steering 0.34 -> 0.4788 rad, the
measured full lock. Also fixed the speed comment, which still described the old
1.5 cap. Per-car calibration -- marked as such so it is not copied to 3호차.

Ignore the colcon build/install/log left by a nested build in this repo.
```

### 최상위 (신규 추가분)

```
chore: track driver config, integration notes and pacejka_sysid

config/imu_params.yml pins the Microstrain to /dev/microstrain_main: it and the
VESC both enumerate as 0483:5740, so the driver's ttyACM0 default opened the
VESC, timed out, and corrupted its serial stream.

Also adds integration/ (workspace + repo consolidation plan),
src/control/pacejka_sysid, and the mocap calibration_v2 script.
```

---

## 부록 B — 저장소 핀 기록

미등록 저장소의 현재 커밋 (2026-07-29 기준, 재현·복구용):

```
src/vesc           153998df8545fe1781b975df88e411b4e71d4bfe   # f1tenth/vesc 상류 pristine (이식 전)
src/ackermann_mux  b3c0b083ac03aa8c648537d7e4d22608fcd3440c   # f1tenth/ackermann_mux 상류
src/f1tenth_system 30946d3158cad9d649b4fcba42776f7b9a901b3b   # 미푸시 (odom 부호 커밋)
```

`src/vesc` 해시는 **Phase A' 이식 직전 상태**다. 이식 후 문제가 생기면 이 커밋으로 되돌린다.
