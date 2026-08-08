# 워크스페이스 통합 · 이관 계획

최초: 2026-07-23 · 갱신: 2026-08-08
워크스페이스: `~/f1tenth_ws` (4호차 기준 작성, 3호차 이관 진행 중)

서브모듈 등록/원격/브랜치/git 트러블슈팅은 [git-submodule-version-control.md](git-submodule-version-control.md) 참조.
이 문서는 **워크스페이스 통합(경로·설정·이관)에만** 집중한다.

> **범위 조정 1**: driver 이식 검증 중 발견한 "deadman 떼도 계속 움직이는" 문제는
> 코드 통합과 무관한 별도 이슈라 [vesc-deadman-command-timeout.md](vesc-deadman-command-timeout.md)로
> 분리했다.
>
> **범위 조정 2**: `mocap_localization` 패키지 이름 변경(내용이 GT 발행/캘리브레이션만 남아
> 이름과 안 맞음)은 팀 논의 + 대공사가 필요해 [mocap-package-rename.md](mocap-package-rename.md)로
> 분리했다. 아직 미착수.

---

## 0. 현재 상태 (2026-08-03)

**rename 완료.** 4호차가 `~/f1tenth` → `~/f1tenth_ws`로 바뀌어 3호차와 이름이 통일됐다.
센서/IMU/MCL/perception 정상, 제어도 저속 정상.

- Phase A (유실 방지), Phase B (저장소 구조 정리): ✅ 완료 — 서브모듈 등록/포인터 상세는
  [git-submodule-version-control.md §1](git-submodule-version-control.md#1-서브모듈-등록-현황-2026-08-08-실측-git-submodule-status---recursive-기준) 참조.
- Phase C (3호차 이관): **진행 중, 설정 백업까지만 완료.** 아래 §3 참조.
- Phase D (절대경로 제거): 미착수, 후순위.

### rename으로 부수적으로 해결된 것

이전 조사에서 "3세대 잔재"로 분류했던 `/home/a/f1tenth_ws/...` 참조 9곳이
이름이 바뀌면서 다시 유효해졌다. 실제로 깨진 경로는 3개뿐이고 전부 inert
(`als_ros/logs/pose_log.txt`, `als_ros/maps/slam_map/`, `navigation2/nav2_bringup/params`).
→ 절대경로 제거(Phase D)는 중기 과제로 후순위.

### 정리한 잔여물

`/home/a/f1tenth/`는 `mocap_localization.cpp` 1개뿐이고 워크스페이스 버전이 상위집합임을 확인 → 삭제 안전.

---

## 1. 드라이버 세팅 전략 (핵심 설계 결정)

### 문제

3호차 수동조작이 4호차보다 나아서 기존 단일 `f1tenth_stack/config/vesc.yaml`을 이식하고 싶었는데,
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

**차량별 VESC YAML과 `joy_teleop.yaml`을 다른 차량 값으로 덮어쓰면 안 된다.**

- `steering_angle_to_servo_offset`, `steering_angle_to_servo_gain_left/right`, `speed_to_erpm_gain`은
  **차량별 실측값**이라 4호차 값이 3호차에 그대로 들어가면 안 됨.
- 새 `src/vesc` 코드는 `_left`/`_right` 게인을 declare하므로, 3호차 yaml에 이 값이 없으면
  4호차가 실제로 겪었던 것과 같은 패턴(미선언 시 조용히 무시) 위험이 있음.
- odom 부호도 마찬가지로 **부호만** 새 로직에 맞춰 양수로 바꾸고, 크기(게인 magnitude)는 3호차 고유 실측값 유지.

→ 기존 단일 파일을 수정한 배포에서는 pull 전 반드시 외부 백업한다. 현재는 아래처럼
차량별 파일을 분리했고 bringup에서 `vesc_config` 선택을 필수로 만들었다.

### 적용된 권장 구조 (2026-08-08)

```
src/vesc/  (fork → 서브모듈)          ← 코드. 좌/우 게인 분기, odom 음수화 제거, QoS
src/f1tenth_system/  (fork → 서브모듈)
  f1tenth_stack/config/vesc_car3.yaml  ← 3호차 실측값과 장치 경로
  f1tenth_stack/config/vesc_car4.yaml  ← 4호차 실측값과 장치 경로
  f1tenth_stack/config/joy_teleop.yaml ← 조작 프로파일
```

모든 차량 bringup은 `vesc_config:=.../vesc_car3.yaml` 또는
`vesc_config:=.../vesc_car4.yaml`을 반드시 받는다. 암묵적 기본 차량은 없다.
SLAM의 `publish_odom_tf`는 차량 YAML에 넣지 않고 launch 인자로만 관리한다.

후보 캘리브레이션을 실차에서만 유지해야 한다면 tracked 파일에 `skip-worktree`를
걸지 말고 저장소 밖 파일을 `vesc_config`로 전달한다.

---

## 2. Phase C — 3호차 이관 (진행 중)

**순서**: 서브모듈/포인터 정리(완료) → 3호차 로컬 감사 → 백업(✅ 완료) → pull → 재빌드 → 차량별 값 검증(⬜ 미착수)

### 진행 상황 (2026-08-03 확인)

`integration/backups/car3-20260729/`에 `vesc.yaml`/`joy_teleop.yaml` 백업이 이미 존재
(3호차 실측값 확인됨: `speed_to_erpm_gain: 4185.0`, `steering_angle_to_servo_gain_left/right: -1.882`,
`steering_angle_to_servo_offset: 0.52`, joy `scale: 2.0`/`0.2243`).

하지만 **현재 워크스페이스의 `src/f1tenth_system/f1tenth_stack/config/`는 여전히 4호차 값**
(`speed_to_erpm_gain: 3806.35`, gain `-1.05417`/`-1.34250`, offset `0.44105`, joy `scale: 4.5`/`0.4788`) —
즉 아래 절차의 2번(백업)까지만 끝났고 4~7번(복원·재빌드·차량별 검증)은 아직.

### 남은 절차

1. **3호차 로컬 변경 감사**:
   ```bash
   git status
   git log --oneline @{u}..HEAD
   for d in src/*/ src/control/*/; do
     [ -e "$d/.git" ] && { echo "== $d"; git -C "$d" status -s; git -C "$d" log --oneline @{u}..HEAD 2>/dev/null; }
   done
   ```
2. ~~드라이버 config 백업~~ ✅ 완료 (`integration/backups/car3-20260729/`)
3. 워크스페이스 전체 백업 (`~/f1tenth_ws` 통째로 복사)
4. `git pull` + `git submodule update --init --recursive`
5. **드라이버 config 복원**: `vesc.yaml`의 서보 오프셋/좌우 게인/erpm 게인은 `integration/backups/car3-20260729/`
   기준으로 복원, `speed_to_erpm_gain` 부호는 새 코드(음수화 제거)에 맞게 양수인지 재확인
   (백업본은 이미 양수 `4185.0`이라 문제없음).
   `joy_teleop.yaml`의 `scale`도 백업본 값(`2.0`/`0.2243`)으로 복원.
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

### 이관 전 확인해야 할 서브모듈 이슈

`smppi_cuda_controller` 브랜치 불일치는 해결됨 — overtake commit 위에서 진단 작업이
계속된 현재 브랜치 `emergency-diagnosis`를 `.gitmodules`가 추적한다
([git-submodule-version-control.md §2](git-submodule-version-control.md#2-현재-알려진-불일치--리스크-2026-08-08)
참조). 현재 포인터와 하위 branch는 2026-08-08 origin에 push 완료했다.

과거 3호차 감사에서 `smppi_cuda_controller` `.git/modules` 실체 유실로 `git status`가 전체 실패한 전례가
있으니, pull 후 서브모듈 메타데이터도 한 번 확인하는 게 안전하다 (해결 절차는 해당 문서 §6 참조).
(`mppi_controller`는 미등록 상태로 로컬에 재생성돼 있던 것을 2026-08-03 `rm -rf`로 정리함 — git 이력엔 원래
없던 것이라 이관과 무관.)

---

## 3. Phase D — 절대경로 제거 (중기, 미착수)

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
- §1의 `car3.yaml`/`car4.yaml` 차량별 드라이버 설정 분리도 여기서 함께

---

## 4. 검증

```bash
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

## 5. 리스크 (2026-08-03 갱신)

| 리스크 | 영향 | 상태 |
|---|---|---|
| `vesc.yaml` 통째 복사 | 서보 중립이 넘어와 차가 비뚤게 달림 | ⚠️ **3호차 복원 단계에서 재발 위험 — §1, §2-5 참조. 백업은 됐으나 복원 전** |
| `mux.yaml` 이식 | 수동조작 완전 정지 + deadman 우선권 소멸 | 이식 목록에서 제외 (유지) |
| 3호차 미푸시 작업 유실 | 작업 손실 | ⬜ 감사 필요 (Phase C-1 미실행) |
| `MCL.h` 재빌드 누락 | 조용히 구 동작 | 커밋 메시지에 명시됨 |
| als_ros 등 1세대 절대경로 잔존 | 다른 머신/사용자에서 조용히 깨짐 | Phase D 미착수, 영향 3곳 모두 inert 확인됨 |

서브모듈/git 관련 리스크(브랜치 불일치, 미푸시 커밋 등)는
[git-submodule-version-control.md §2](git-submodule-version-control.md#2-현재-알려진-불일치--리스크-2026-08-08) 참조.

**되돌리기**: Phase C는 3호차 백업(`integration/backups/car3-20260729/` 또는 워크스페이스 전체 백업) 복원.
