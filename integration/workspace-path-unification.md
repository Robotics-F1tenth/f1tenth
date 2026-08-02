# 워크스페이스 통합 · 저장소 정리 계획

최초: 2026-07-23 · 갱신: 2026-07-30 (Phase A/B 완료, Phase C 3호차 이관 진행 중)
워크스페이스: `~/f1tenth_ws` (4호차 기준 작성, 3호차 이관 작업 중)

> **범위 조정 1**: driver 이식 검증 중 발견한 "deadman 떼도 계속 움직이는" 문제는
> 코드 통합과 무관한 별도 이슈라 [vesc-deadman-command-timeout.md](vesc-deadman-command-timeout.md)로
> 분리했다.
>
> **범위 조정 2**: `mocap_localization` 패키지 이름 변경(내용이 GT 발행/캘리브레이션만 남아
> 이름과 안 맞음)은 팀 논의 + 대공사가 필요해 [mocap-package-rename.md](mocap-package-rename.md)로
> 분리했다. 아직 미착수.
>
> 이 문서는 저장소 통합·이관에만 집중한다.

---

## 0. 현재 상태

**rename 완료.** 4호차가 `~/f1tenth` → `~/f1tenth_ws`로 바뀌어 3호차와 이름이 통일됐다.
센서/IMU/MCL/perception 정상, 제어도 저속 정상.

### rename으로 부수적으로 해결된 것

이전 조사에서 "3세대 잔재"로 분류했던 `/home/a/f1tenth_ws/...` 참조 9곳이
이름이 바뀌면서 다시 유효해졌다. 실제로 깨진 경로는 3개뿐이고 전부 inert
(`als_ros/logs/pose_log.txt`, `als_ros/maps/slam_map/`, `navigation2/nav2_bringup/params`).
→ 절대경로 제거(Phase D)는 중기 과제로 후순위.

### 정리한 잔여물

`/home/a/f1tenth/`는 `mocap_localization.cpp` 1개뿐이고 워크스페이스 버전이 상위집합임을 확인 → 삭제 안전.

---

## 1. 저장소 연결 분석 (2026-07-30 최종 상태)

트리 내 git 저장소 **7개**(최상위 포함, `mppi_controller` 제거 후). 최상위는 `Robotics-F1tenth/f1tenth`.

### 서브모듈 등록 상태

| 경로 | 원격 | 브랜치 | 상태 |
|---|---|---|---|
| `src/f1_localization` | `heojy/f1_localization` | main | ✅ push 완료 (`9776585`), 포인터 갱신 완료 |
| `src/perception` | `heojy/f1_perception` | `feature/acc-yaw-estimation` | ✅ push 완료 (`57cfe1f`), 포인터 갱신 완료 |
| `src/vesc` | `Robotics-F1tenth/vesc` (fork) | `ros2` | ✅ 신규 등록 완료 (`d53d999`) |
| `src/f1tenth_system` | `Robotics-F1tenth/f1tenth_system` | (main 계열, 커밋 `321f4c1`) | ✅ 신규 등록 + 포인터 갱신 완료 |
| `src/control/smppi_cuda_controller` | `JangJunhyeok21/smooth-mppi-cuda` | `main` (일치 완료) | 개발 중, 포인터만 최신화됨 — 로컬 체크아웃은 `submodule update` 필요 |
| `src/racecar_simulator` | `JangJunhyeok21/racecar_simulator` | detached HEAD | 포인터 갱신됨 — 로컬 체크아웃은 `submodule update` 필요 |

### 제거됨 — `src/control/mppi_controller`

2026-07-30, 안 쓰는 패키지로 확정돼 서브모듈 등록 자체를 해제했다 (`deinit` + `git rm` +
`.git/modules` 정리, 커밋 `d881145`). 원격 저장소(`e753d76`)엔 실제로 파일이 멀쩡히 있었고
—로컬에서만 누군가 삭제를 스테이징해놓고 커밋 안 한 상태였다— 그래서 "clone하면 빈 패키지를
받는다"는 초기 판단은 부정확했다(다른 클론엔 영향 없었음). 그래도 안 쓰는 게 확정이라 등록 자체를 없앴다.

### 등록하지 않고 유지 — `src/ackermann_mux`

- 원격: `f1tenth/ackermann_mux` (상류 그대로)
- **의도적으로 서브모듈 미등록 상태 유지.** untracked embedded repo로 남겨두고 핀 해시만 부록 B에 기록.
- 이유: 상류가 아직 수정 안 됐고, 서브모듈로 등록할 만한 변경이 없음 — 지금 등록해도 이득이 없음.

### f1tenth_system 안의 중첩 서브모듈 문제 — 해결 완료 (2026-07-30 확인)

`f1tenth_system` 자체 `.gitmodules`에 `vesc`/`ackermann_mux`/`teleop_tools`가 선언돼 있어서,
최상위에 서브모듈로 등록한 뒤 `clone --recursive` 하면 이 3개가 채워지면서
최상위 `src/vesc`·`src/ackermann_mux`와 패키지 이름이 충돌해 `colcon build`가 실패하는 문제가 있었다.

**해결 2단계 (둘 다 필요했다):**
1. fork `Robotics-F1tenth/f1tenth_system`의 `humble-devel` 브랜치에서 `.gitmodules`와
   3개 서브모듈 선언을 전부 제거하는 커밋(`321f4c1`)을 만들어 push.
   (주의: 이 fork의 기본 브랜치는 `foxy-devel`이 아니라 실제 쓰는 `humble-devel`이었음 —
   처음 `foxy-devel`에 잘못 커밋했다가 발견, `humble-devel`로 cherry-pick해서 재push함.)
2. **이 워크스페이스의 최상위 포인터를 `e288c86` → `321f4c1`로 실제로 옮기는 것.**
   1번만 하고 2번을 안 하면 이 워크스페이스는 여전히 옛 커밋(`e288c86`, 옛 `.gitmodules` 포함)을
   가리키므로 recursive clone 충돌 위험이 **그대로 남는다** — 포크에 고침 커밋이 존재하는 것과
   이 워크스페이스가 그 커밋을 가리키는 것은 별개다. 확인 후 바로 포인터를 옮겨 push 완료.

두 세션이 독립적으로 같은 결론(포인터를 `321f4c1`로)에 도달해 로컬 커밋이 서로 갈라졌었지만,
목적지가 같아 충돌 없이 병합됐다 (`f833f40`).

---

## 2. 드라이버 세팅 전략 (핵심 설계 결정 — 유지)

### 문제

3호차 수동조작이 4호차보다 나아서 `f1tenth_stack/config/vesc.yaml`을 이식하고 싶었는데,
이 파일엔 **차량별 물리 캘리브레이션**(서보 오프셋, 좌/우 게인, erpm 게인)과
**구조/로직 수정**(odom 부호 반전 취소, port 경로)과 **운전 취향**(joy_teleop scale)이 섞여 있다.
통째로 복사하면 서보 중립까지 넘어와 차가 비뚤게 달린다.

### 원인 규명 완료 — 검증 완료 (4호차)

- `ackermann_to_vesc.cpp`가 `steering_angle_to_servo_gain_left/right`를 declare하지 않아서
  ROS 2가 미선언 파라미터를 조용히 무시, 실제 적용값은 단일 게인 `-1.0`뿐이었음 → 좌우 조향 비대칭의 원인.
- `vesc_to_odom.cpp`의 하드코딩된 음수화 때문에, driver 버전(음수화 제거)으로 코드만 바꾸면
  `speed_to_erpm_gain`도 양수로 같이 바꿔야 함 — **이식과 부호 정정은 반드시 동시에.**

### 이식 결과 (4호차, 2026-07-29 실행 완료)

- 좌/우 조향 게인 분리 후 양쪽 다 서보 한계(0.01/0.99) 도달 확인, 좌우 대칭 회복(85.4% 동일).
- odom 부호: 전진 시 `/odom` linear.x 양수 확인 (+4462 eRPM → +1.1723, −4493 eRPM → −1.1804).
- mux 우선권 정상 (joy 100 > nav 10, 값 안 섞임).
- clipping 경고는 사라지지 않음 — 게인이 풀스틱에서 서보 한계를 살짝 넘도록 잡혀 있어(끝단 도달 보장)
  양쪽 끝에서 대칭적으로 클립되는 게 정상. 판정 기준은 "경고 없음"이 아니라 "양쪽 다 0.01/0.99 도달".

### ⚠️ 3호차 이관 시 반드시 지킬 것

**f1tenth_system의 `vesc.yaml`/`joy_teleop.yaml`을 pull로 그대로 덮어쓰면 안 된다.**

- `steering_angle_to_servo_offset`, `steering_angle_to_servo_gain_left/right`, `speed_to_erpm_gain`은
  **차량별 실측값**이라 4호차 값이 3호차에 그대로 들어가면 안 됨.
- 새 `src/vesc` 코드는 `_left`/`_right` 게인을 declare하므로, 3호차 yaml에 이 값이 없으면
  4호차가 실제로 겪었던 것과 같은 패턴(미선언 시 조용히 무시) 위험이 있음.
- odom 부호도 마찬가지로 **부호만** 새 로직에 맞춰 양수로 바꾸고, 크기(게인 magnitude)는 3호차 고유 실측값 유지.

→ **3호차 pull 전 반드시 `vesc.yaml`/`joy_teleop.yaml` 백업 → pull 후 차량별 값 복원 → 필요시 좌/우 게인 재실측.**
(장기적으로는 `car3.yaml`/`car4.yaml`로 차량별 값을 분리해 이 문제 자체를 없애는 게 맞다.)

### 권장 구조 (Phase D와 함께, 아직 미착수)

```
src/vesc/  (fork → 서브모듈)          ← 코드. 좌/우 게인 분기, odom 음수화 제거, QoS
src/f1tenth_system/  (fork → 서브모듈)
  f1tenth_stack/config/vesc.yaml       ← 구조·로직 (port, mux 우선순위, 공용 기본값)
  f1tenth_stack/config/joy_teleop.yaml ← 조작 프로파일

config/  (최상위, 차량별)
  imu_params.yml                        ← 이미 있음
  car4.yaml                             ← 서보 오프셋, 좌/우 게인, erpm gain, imu_rotation
```

launch에서 `parameters=[vesc.yaml, car4.yaml]` 순으로 넘기면 뒤가 앞을 덮는다
(`mcl.launch.py`에서 쓴 패턴과 동일). 차량 선택은 launch 인자(`car:=car4`) 또는 환경변수.

> 이 구조를 갖추기 전까지는 **개별 값만 손으로 옮기고 파일 통째 복사는 금지.**

---

## 3. 실행 계획 — 진행 상황

### Phase A — 유실 방지: ✅ 완료

| 저장소 | 로컬 커밋 | 푸시 | 최상위 등록 |
|---|---|---|---|
| `src/f1_localization` | `9776585` | ✅ | 포인터 갱신 완료 |
| `src/perception` | `57cfe1f` | ✅ | 포인터 갱신 완료 |
| `src/vesc` | `d53d999` | ✅ (fork로 remote 변경 후) | 신규 등록 완료 |
| `src/f1tenth_system` | `321f4c1` | ✅ | 신규 등록 + 포인터 갱신 완료 |

최상위 커밋 이력 (시간순):
- `ca11ed1` — f1_localization/perception 포인터 갱신, vesc/f1tenth_system 신규 등록
- `f9a6312` — config/, integration/, pacejka_sysid, calibration_v2.py 추가
  (`git add -A`가 `src/ackermann_mux`를 embedded repo로 잘못 포함시킨 걸 `git rm --cached` + amend로 정정)
- `bf4ff92` / `bd703d5` → 병합 `f833f40` — f1tenth_system 포인터를 `321f4c1`로 올리는 작업을
  두 세션이 독립적으로 만들어 충돌 없이 병합. smppi_cuda_controller 브랜치를 `main`으로 정정한 것도 여기 포함.
- `5907c68` / `d881145` → 병합 `75d6feb` — smppi_cuda_controller/racecar_simulator 포인터 갱신(원격)과
  mppi_controller 서브모듈 제거(로컬)가 서로 다른 경로라 충돌 없이 병합.

`git push` 완료, 원격과 동기화됨 (HTTPS 자격증명이 이 환경에 없어 SSH URL로 직접 push).

### Phase B — 저장소 구조 정리: ✅ 완료

1. ✅ `f1tenth_system` 서브모듈 등록 완료
2. ✅ `vesc` fork + 서브모듈 등록 완료
3. ✅ f1tenth_system fork의 `.gitmodules`에서 중첩 vesc/ackermann_mux/teleop_tools 선언 제거,
   **+ 이 워크스페이스의 최상위 포인터도 `321f4c1`로 갱신** (둘 다 해야 실제로 해결됨 — §1 참조)
4. ✅ `mppi_controller` 서브모듈 완전 제거
5. ✅ `smppi_cuda_controller` 브랜치 일치 (`.gitmodules` `main`, 팀원 확인 완료)
6. `ackermann_mux` 핀 해시 부록 B에 기록 완료

**남은 사소한 것**: `smppi_cuda_controller`·`racecar_simulator`는 포인터만 최신이고 로컬 작업트리는
아직 이전 커밋 체크아웃 상태 (`git submodule status`에서 `+` 표시). 필요할 때
`git submodule update --init src/control/smppi_cuda_controller src/racecar_simulator`로 동기화.
다른 사람 작업 중인 패키지라 임의로 체크아웃 안 함.

### Phase C — 3호차 이관: 진행 중

**순서**: fork `.gitmodules` 정리 + 포인터 갱신(완료) → 3호차 로컬 감사 → 백업 → pull → 재빌드 → 차량별 값 검증

#### 3호차 감사 중 발견한 문제 — smppi_cuda_controller 메타데이터 유실

3호차에서 `git status`가 다음 에러로 전체 실패:
```
fatal: not a git repository: src/control/smppi_cuda_controller/../../../.git/modules/src/control/smppi_cuda_controller
```

원인 조사 결과:
- `.gitmodules` 자체가 워킹 디렉토리에서 사라져 있었음 (`git show HEAD:.gitmodules`로 확인하니 이력엔 정상 존재 —
  디스크에서만 삭제된 상태). **2호차 설정을 만지다 코드를 git이 아니라 물리적으로 복사해온 이력**이 원인으로 추정됨
  (`smppi_cuda_controller/.git` 파일 타임스탬프가 `1970-01-01`, 즉 epoch 0 — rsync/tar류 복사의 전형적 증상).
- `.gitmodules`는 `git checkout HEAD -- .gitmodules`로 즉시 복원 완료.
- `smppi_cuda_controller`의 `.git/modules` 아래 실체는 여전히 없음 → 이 서브모듈 자체는 유실 확정.
  3호차에서 이 패키지가 원래도 동작하지 않았음을 확인 → **로컬 상태 보존 없이 폐기하고 재초기화하기로 결정.**

**해결 절차**:
```bash
git rm -f src/control/smppi_cuda_controller
rm -rf src/control/smppi_cuda_controller
rm -rf .git/modules/src/control/smppi_cuda_controller
git -c submodule.ignore=all status   # fatal 없이 떠야 정상
git submodule update --init src/control/smppi_cuda_controller   # main 브랜치로 재초기화
```

#### 남은 Phase C 절차

1. **3호차 로컬 변경 감사** (smppi 재초기화 후 재실행):
   ```bash
   git status
   git log --oneline @{u}..HEAD
   for d in src/*/ src/control/*/; do
     [ -e "$d/.git" ] && { echo "== $d"; git -C "$d" status -s; git -C "$d" log --oneline @{u}..HEAD 2>/dev/null; }
   done
   ```
2. **드라이버 config 백업** (§2 경고 반영, pull 전 필수):
   ```bash
   cp src/f1tenth_system/f1tenth_stack/config/vesc.yaml ~/vesc_3호차_backup.yaml
   cp src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml ~/joy_teleop_3호차_backup.yaml
   ```
3. 백업: `~/f1tenth_ws` 통째로 복사
4. `git pull` + `git submodule update --init --recursive`
5. **드라이버 config 복원**: `vesc.yaml`의 서보 오프셋/좌우 게인/erpm 게인은 백업본(3호차 실측값) 기준으로 복원,
   단 `speed_to_erpm_gain` 부호는 새 코드(음수화 제거)에 맞게 양수인지 재확인.
   `joy_teleop.yaml`의 `scale`도 3호차 값으로 복원.
   백업본에 `_left`/`_right` 게인이 애초에 없었다면 3호차에서 신규 실측 필요.
6. `rm -rf build/ install/ log/` 후 전체 재빌드
7. **차량별 확인 — 4호차 값을 그대로 가정하지 말 것:**
   - IMU 축: 정지 시 `linear_acceleration.z`가 `-9.8`이면 `imu_rotation_roll:=180`, `+9.8`이면 `0`
   - odom 부호: 전진 시 `/odom` `twist.linear.x`가 양수인지
   - 서보 중립/조향 게인: 3호차 값 유지
8. **udev 룰 수동 적용** (git 밖):
   `/etc/udev/rules.d/99-vesc6.rules`에 `ATTRS{product}=="ChibiOS/RT Virtual COM Port"` 추가
   → `udevadm control --reload-rules`
   VESC와 IMU가 같은 `0483:5740`이라 없으면 `/dev/sensors/vesc`가 IMU를 가리킬 수 있다.
   (4호차 백업: `99-vesc6.rules.bak`)

### Phase D — 절대경로 제거 (중기, 미착수)

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
- §2의 `car3.yaml`/`car4.yaml` 차량별 드라이버 설정 분리도 여기서 함께

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

주행 검증 (양 차량 동일, **아직 3호차에서 미실행**):
1. `sensors` → `imu` → `mcl`
2. `ros2 run tf2_ros tf2_echo map laser` — 벽시계 타임스탬프로 갱신
3. 좌회전 시 rviz pose가 같은 방향으로 회전
4. 전진 시 `/odom` `twist.linear.x` 양수, 추정도 전진
5. bag: `bag_mcl.launch.py` + `ros2 bag play --clock`
   구 bag은 `imu_name:=ekf/imu/data imu_rotation_roll:=0.0`

---

## 5. 리스크 (최종 갱신)

| 리스크 | 영향 | 상태 |
|---|---|---|
| `f1tenth_system` 미푸시 + 미등록 | 드라이버 설정 전량 유실, 3호차 이관 불가 | ✅ 해결 |
| driver 이식 시 odom 부호 정정 누락 | 전진 시 `/odom` 음수 → MCL 붕괴 | ✅ 해결, 4호차 실측 검증 완료 |
| `mux.yaml` 이식 | 수동조작 완전 정지 + deadman 우선권 소멸 | 이식 목록에서 제외 (유지) |
| `vesc.yaml` 통째 복사 | 서보 중립이 넘어와 차가 비뚤게 달림 | ⚠️ **3호차 이관 시 재발 위험 — §2, Phase C-5 참조** |
| f1tenth_system recursive clone 시 패키지명 충돌 | 중첩 vesc/ackermann_mux가 채워져 빌드 실패 | ✅ 해결 — fork `.gitmodules` 제거 + 이 워크스페이스 포인터 갱신 (둘 다 완료) |
| `mppi_controller` 빈 서브모듈 | (실제로는 로컬 전용 문제였음) | ✅ 서브모듈 자체 제거로 정리 |
| `smppi_cuda_controller` 메타데이터 유실 | 3호차 `git status` 전체 실패 | ⬜ 3호차에서 재초기화 필요 (Phase C 참조) |
| `ackermann_mux` 미핀 | 상류 변경 시 예기치 않은 동작 | 해시 기록 완료, 유지 |
| 3호차 미푸시 작업 유실 | 작업 손실 | ⬜ 감사 필요 (smppi 재초기화 후) |
| `MCL.h` 재빌드 누락 | 조용히 구 동작 | 커밋 메시지에 명시됨 |

**되돌리기**: Phase A~B는 각 저장소 `git revert`. Phase C는 3호차 백업 복원.

---

## 부록 A — 커밋 메시지 (실제 커밋됨)

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

### `src/f1tenth_system` — commit `e288c86` (코드/설정), `321f4c1` (fork, 서브모듈 정리)
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
```
chore: drop nested vesc/ackermann_mux/teleop_tools submodules (name conflicts with workspace src/)
```
(최초 `foxy-devel`에 잘못 커밋(`0ee29e2`)했다가 발견, `humble-devel`로 cherry-pick하여 재작성 후 push)

### 최상위 — 시간순 커밋

```
ca11ed1  chore(submodules): bump f1_localization and perception pointers to pushed commits
f9a6312  chore: track driver config, integration notes and pacejka_sysid
bd703d5  chore(submodules): bump f1tenth_system to drop nested vesc/ackermann_mux/teleop_tools declarations
bf4ff92  chore(submodules): track smppi main and drop nested drivers                          (원격, 병합됨)
f833f40  Merge (위 둘, 같은 목적지로 수렴해 충돌 없음)
d881145  chore(submodules): remove unused mppi_controller submodule
5907c68  chore(submodules): update smppi_cuda_controller and racecar_simulator subproject commits  (원격, 병합됨)
75d6feb  Merge (위 둘, 다른 경로라 충돌 없음)
```

`f9a6312`의 `chore: track driver config...` 본문:
```
config/imu_params.yml pins the Microstrain to /dev/microstrain_main: it and the
VESC both enumerate as 0483:5740, so the driver's ttyACM0 default opened the
VESC, timed out, and corrupted its serial stream.

Also adds integration/ (workspace + repo consolidation plan),
src/control/pacejka_sysid, and the mocap calibration_v2 script.
```
(주: 최초 커밋 시 `git add -A`가 `src/ackermann_mux`를 embedded repo로 잘못 포함시켜서
`git rm --cached src/ackermann_mux` 후 amend로 정정함.)

---

## 부록 B — 저장소 핀 기록

미등록/참고용 저장소의 커밋 (2026-07-30 기준):

```
src/ackermann_mux            b3c0b083ac03aa8c648537d7e4d22608fcd3440c   # f1tenth/ackermann_mux 상류, 의도적으로 미등록 유지
src/vesc (이식 전 pristine)   153998df8545fe1781b975df88e411b4e71d4bfe   # Phase A' 이식 직전 상태. 문제 생기면 이 커밋으로 되돌림
```

---

## 부록 C — 트러블슈팅 로그

참고용 — 다음에 비슷한 증상 만나면 여기부터 확인.

1. **VSCode 계정 전환 시 세션 위치**: Claude Code 인증은 워크스페이스가 아니라 OS 전역
   (macOS Keychain / Linux·Windows `~/.claude/.credentials.json`)에 저장됨. `/logout` → `/login`으로 전환,
   VSCode 확장은 반영 안 되면 재시작 필요.
2. **`.gitmodules` 워킹 디렉토리에서만 소실**: `git show HEAD:.gitmodules`로 이력엔 있는데 디스크에서 사라진 상태
   확인 → `git checkout HEAD -- .gitmodules`로 즉시 복원 가능. 원인은 2호차 설정 작업 중 코드를 git이 아니라
   물리적으로 복사해온 이력으로 추정 (`.git` gitlink 파일 타임스탬프가 `1970-01-01`, rsync/tar 복사의 전형적 증상).
3. **서브모듈 `.git/modules` 실체 유실**: gitlink는 있는데 `.git/modules/.../서브모듈명` 디렉토리가 없으면
   `fatal: not a git repository` 로 최상위 `git status`까지 통째로 실패함. 로컬 작업 보존이 필요 없다고 확정되면
   `git rm -f <path>` + 워킹 디렉토리 삭제 + `.git/modules` 아래 잔재 삭제 후 `submodule update --init`으로 재초기화.
4. **fork 저장소 기본 브랜치 확인 필수**: `git clone`은 기본 브랜치(`HEAD`)를 체크아웃하는데 이게 실제 작업 브랜치와
   다를 수 있음(`f1tenth_system` fork는 기본이 `foxy-devel`, 실제 필요한 건 `humble-devel`). push 전
   `git branch -r`로 확인.
5. **`git add -A`는 이 저장소에서 위험**: 아직 서브모듈로 등록 안 한 embedded repo(`ackermann_mux` 등)가
   섞여 있는 한, `git add -A`가 그걸 그대로 커밋에 포함시켜버림 (`git rm --cached`로 사후 정정 가능하나
   애초에 경로를 명시해서 add하는 게 안전).
6. **HTTPS 원격 push가 이 셸 환경에서 막힐 수 있음**: `git fetch`/`push`가
   `could not read Username for 'https://github.com'`로 실패하면 자격증명 헬퍼가 없는 것.
   `git push git@github.com:<org>/<repo>.git HEAD:main` 처럼 SSH URL을 직접 지정하면 우회 가능
   (SSH agent에 키가 등록돼 있다면). 단 이 방식은 로컬 `refs/remotes/origin/main`을 갱신하지 않으므로,
   이후 `@{u}..HEAD` 비교는 stale할 수 있다 — 정확한 비교가 필요하면 같은 방식으로 fetch부터 다시 할 것.
7. **서로 다른 세션이 같은 문제를 독립적으로 고치면 충돌 없이 병합될 수 있다**: 두 세션이 각자
   `f1tenth_system` 포인터를 `321f4c1`로 옮기는 커밋을 만들었는데, 목표 지점이 같아 3-way merge가
   충돌 없이 처리됨. 다만 이건 운이 좋았던 경우이고, 여러 세션/터미널에서 동시에 같은 저장소를 만지고
   있다면 작업 전 `git fetch` + `git log HEAD..origin/main`으로 먼저 확인하는 습관이 안전하다.
