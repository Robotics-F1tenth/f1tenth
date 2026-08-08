# 저장소 · 서브모듈 버전관리

최초: 2026-07-23 · 갱신: 2026-08-08
워크스페이스 통합/이관 계획은 [workspace_unification.md](workspace_unification.md) 참조.
이 문서는 **git 서브모듈 등록·원격·브랜치 상태와 관련 트러블슈팅만** 다룬다.

---

## 1. 서브모듈 등록 현황 (2026-08-08 실측, `git submodule status --recursive` 기준)

| 경로 | 원격 | `.gitmodules` 브랜치 | 현재 체크아웃 | 상태 |
|---|---|---|---|---|
| `src/f1_localization` | `heojy/f1_localization` | (미지정) | `43c9fb8` (`feature/slam-map-workflow`) | ✅ origin에 push 완료, `.omc/`만 로컬 IDE 캐시로 남음 |
| `src/perception` | `heojy/f1_perception` | `feature/acc-yaw-estimation` | `404d224` | ✅ origin에 존재 확인 |
| `src/vesc` | `Robotics-F1tenth/vesc` (fork) | `ros2` | `d53d999` | clean |
| `src/f1tenth_system` | `Robotics-F1tenth/f1tenth_system` (fork) | (미지정) | `a2754cc` (`feature/slam-map-workflow`) | ✅ origin에 push 완료 — mapping용 odom TF launch 인자 포함 |
| `src/control/smppi_cuda_controller` | `JangJunhyeok21/smooth-mppi-cuda` | `emergency-diagnosis` | `ff0c252` | ✅ origin에 push 완료 — overtake commit 위에 진단·handoff 누적 |
| `src/racecar_simulator` | `JangJunhyeok21/racecar_simulator` | (미지정) | `e56a91d` (detached, origin/HEAD) | clean |

### 등록하지 않고 유지 — `src/ackermann_mux`

- 원격: `f1tenth/ackermann_mux` (상류 그대로)
- **의도적으로 서브모듈 미등록.** untracked embedded repo로 남겨두고 핀 해시만 §4에 기록.
- 이유: 상류가 아직 수정 안 됐고, 서브모듈로 등록할 만한 변경이 없음.

### f1tenth_system 중첩 서브모듈 문제 — 해결 완료

`f1tenth_system` 자체 `.gitmodules`에 `vesc`/`ackermann_mux`/`teleop_tools`가 선언돼 있어서,
최상위에 서브모듈로 등록한 뒤 `clone --recursive` 하면 이 3개가 채워지면서 최상위
`src/vesc`·`src/ackermann_mux`와 패키지 이름이 충돌해 `colcon build`가 실패했다.

**해결은 2단계 모두 필요했다:**
1. fork `humble-devel` 브랜치에서 `.gitmodules`와 3개 서브모듈 선언을 제거하는 커밋(`321f4c1`) push.
   (fork 기본 브랜치가 `foxy-devel`이라 처음 거기에 잘못 커밋 → `humble-devel`로 cherry-pick해 재push)
2. **이 워크스페이스의 최상위 포인터를 실제로 `321f4c1`로 이동.** 1번만으로는 안 됨 —
   포크에 고침 커밋이 있는 것과 이 워크스페이스가 그 커밋을 가리키는 것은 별개.

두 세션이 독립적으로 같은 결론(포인터를 `321f4c1`로)에 도달해 로컬 커밋이 갈라졌지만
목적지가 같아 충돌 없이 병합됐다 (`f833f40`).

---

## 2. 현재 알려진 불일치 / 리스크 (2026-08-08)

| 항목 | 내용 | 조치 |
|---|---|---|
| `src/f1_localization` untracked `.omc/` | 로컬 IDE 캐시, 무해 | 방치 가능, 필요시 `.gitignore` 추가 검토 |

### 해결됨 — `smppi_cuda_controller` 브랜치 불일치

처음에는 `.gitmodules`가 `main`인데 실제 체크아웃은 overtake commit `905d68b`라
`feature/overtake-integration`으로 맞췄다. 이후 진단 작업이 그 commit 위에서
`emergency-diagnosis`로 계속되어 현재 포인터는 `ff0c252`가 됐다.

**현재 결정:** `.gitmodules`도 `emergency-diagnosis`를 추적한다. `ff0c252`의 ancestry에
`905d68b`가 포함되므로 overtake 기능을 잃는 변경이 아니다. 일반적인
`git submodule update --init --recursive`는 최상위 gitlink에 고정된 SHA를 사용하고,
`branch` 값은 주로 `git submodule update --remote`에서 사용된다.

---

## 3. 커밋/푸시 절차 & 검증

```bash
# 저장소 연결 상태 한눈에
git submodule status --recursive

# 미푸시 작업 탐지 (전 저장소)
for d in . src/*/ src/control/*/; do
  [ -e "$d/.git" ] && { echo "== $d"; git -C "$d" log --oneline @{u}..HEAD 2>/dev/null; }
done
```

HTTPS 원격이 이 셸 환경에서 막히면(`could not read Username for 'https://github.com'`)
SSH URL로 직접 push: `git push git@github.com:<org>/<repo>.git HEAD:main`
(단 로컬 `refs/remotes/origin/main`은 갱신 안 되므로, 이후 `@{u}..HEAD` 비교가 필요하면 같은 방식으로 fetch부터 다시 할 것.)

---

## 4. 부록 A — 핀 기록 (미등록/참고용 저장소)

```
src/ackermann_mux            b3c0b083ac03aa8c648537d7e4d22608fcd3440c   # f1tenth/ackermann_mux 상류, 의도적으로 미등록 유지
src/vesc (이식 전 pristine)   153998df8545fe1781b975df88e411b4e71d4bfe   # 이식 직전 상태. 문제 생기면 이 커밋으로 되돌림
```

---

## 5. 부록 B — 주요 커밋 메시지 (실제 커밋됨)

### `src/f1_localization` — `9776585`
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

### `src/vesc` — `d53d999`
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

### `src/f1tenth_system` — `e288c86` (설정), `321f4c1` (fork, 서브모듈 정리)
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
c3eed6d  update submodule pointer: perception
045c1cb  Fix submodule pointer: point to feature/acc-yaw-estimation
affe3f8  chore: update submodule pointers and refine mocap localization parameters
b4eb55f  chore(submodules): re-remove mppi_controller
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

## 6. 부록 C — git 트러블슈팅 로그

참고용 — 다음에 비슷한 증상 만나면 여기부터 확인.

1. **`.gitmodules` 워킹 디렉토리에서만 소실**: `git show HEAD:.gitmodules`로 이력엔 있는데 디스크에서 사라진 상태
   확인 → `git checkout HEAD -- .gitmodules`로 즉시 복원 가능. 원인은 설정 작업 중 코드를 git이 아니라
   물리적으로 복사해온 이력으로 추정 (`.git` gitlink 파일 타임스탬프가 `1970-01-01`, rsync/tar 복사의 전형적 증상).
2. **서브모듈 `.git/modules` 실체 유실**: gitlink는 있는데 `.git/modules/.../서브모듈명` 디렉토리가 없으면
   `fatal: not a git repository` 로 최상위 `git status`까지 통째로 실패함. 로컬 작업 보존이 필요 없다고 확정되면
   `git rm -f <path>` + 워킹 디렉토리 삭제 + `.git/modules` 아래 잔재 삭제 후 `submodule update --init`으로 재초기화.
3. **fork 저장소 기본 브랜치 확인 필수**: `git clone`은 기본 브랜치(`HEAD`)를 체크아웃하는데 이게 실제 작업 브랜치와
   다를 수 있음(`f1tenth_system` fork는 기본이 `foxy-devel`, 실제 필요한 건 `humble-devel`). push 전
   `git branch -r`로 확인.
4. **`git add -A`는 이 저장소에서 위험**: 아직 서브모듈로 등록 안 한 embedded repo(`ackermann_mux` 등)가
   섞여 있는 한, `git add -A`가 그걸 그대로 커밋에 포함시켜버림 (`git rm --cached`로 사후 정정 가능하나
   애초에 경로를 명시해서 add하는 게 안전).
5. **HTTPS 원격 push가 이 셸 환경에서 막힐 수 있음**: `git fetch`/`push`가
   `could not read Username for 'https://github.com'`로 실패하면 자격증명 헬퍼가 없는 것.
   `git push git@github.com:<org>/<repo>.git HEAD:main` 처럼 SSH URL을 직접 지정하면 우회 가능
   (SSH agent에 키가 등록돼 있다면). 단 이 방식은 로컬 `refs/remotes/origin/main`을 갱신하지 않으므로,
   이후 `@{u}..HEAD` 비교는 stale할 수 있다 — 정확한 비교가 필요하면 같은 방식으로 fetch부터 다시 할 것.
6. **서로 다른 세션이 같은 문제를 독립적으로 고치면 충돌 없이 병합될 수 있다**: 두 세션이 각자
   `f1tenth_system` 포인터를 `321f4c1`로 옮기는 커밋을 만들었는데, 목표 지점이 같아 3-way merge가
   충돌 없이 처리됨. 다만 이건 운이 좋았던 경우이고, 여러 세션/터미널에서 동시에 같은 저장소를 만지고
   있다면 작업 전 `git fetch` + `git log HEAD..origin/main`으로 먼저 확인하는 습관이 안전하다.
7. **서브모듈 포인터가 원격에 없는(미푸시) 커밋을 가리키는 사고**: `git fetch`가
   `fatal: 리모트 오류: upload-pack: not our ref <sha>`로 실패하면, 최상위가 가리키는 서브모듈 커밋이
   해당 서브모듈 원격 어디에도 없는 것. 담당자 로컬에 커밋이 남아있으면 push, 없으면 최상위 포인터를
   실제 존재하는 최신 커밋으로 되돌려야 함 (`git checkout origin/<branch>` 후 최상위에서 재커밋).
   `src/perception`에서 한 번 발생(`c479dd5`)했으나 이후 push되어 해소됨 (`c3eed6d`).
8. **서브모듈 안 untracked 파일이 체크아웃을 막는 경우**: `git submodule update --init`이
   "추적하지 않는 파일을 덮어씀" 에러로 중단되면, 서브모듈 안에 커밋 안 된 로컬 전용 파일
   (`.gitignore`, IDE 캐시 등)이 있는 것. 내용 확인 후 순수 로컬용이면 삭제하고 재진행.
9. **최상위 `.gitignore` merge 충돌**: 여러 머신에서 각자 `.gitignore`를 건드리면 pull 시 충돌 나기 쉬움.
   `.gitignore` 변경은 가능하면 한 머신에서만 하고 빠르게 push/pull해서 충돌을 줄이는 게 좋음.
