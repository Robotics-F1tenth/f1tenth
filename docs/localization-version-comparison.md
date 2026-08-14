# Localization 버전 비교 — src/f1_localization vs src/localization (2026-07-20)

두 디렉토리의 정체·전체 diff·모캡 실험구조 해부(07-14 분석) + 정량 성능 비교(07-20 실측).
재현 도구: [eval/localization-bag-replay/](../eval/localization-bag-replay/), 분석기: [eval/tools/analyze_localization.py](../eval/tools/analyze_localization.py).

## 0. 결론 요약

- **`src/f1_localization/` = 정식 서브모듈, 최신** (remote `heojy/f1_localization.git`, HEAD `a547822` — `a7c6815` "오차 15cm version" + P0 fix). **`src/localization/` = 상위 repo 미추적 파일 스냅샷**(자체 .git 없음) + 모캡 실험 패키지.
- 모캡 실험구조(`mocap_localization`)는 **MCL 성능과 무관** — MCL은 odom의 pose를 아예 안 읽음(twist+IMU만). 정체는 GT/평가용 pose 퍼블리셔이며, 실전 GT는 이미 드라이버 직발행 `/mocap_pose1,2`를 쓰므로 **이중 orphan. 신경 쓸 필요 없음.**
- **정량 비교(record5 replay, 동일조건)**: 신·구 코드 위치 정확도 median 동일(p50 0.145 vs 0.147m), **코너는 신형 우세**(p50 0.186→0.100m), 코너 yaw만 신형 열세(1.9°→5.5°).
- **§8(yaw-lag-plan)의 두 걱정 해소**: "신 MCL 2배 회귀"와 "−13cm laser-offset 버그 의심"은 **둘 다 재생환경 속성** — 구 코드를 같은 환경에서 재생해도 정확히 같은 수치(0.145~0.153 / −16cm)가 나옴. 코드 회귀 아님.

## 1. 두 디렉토리의 정체

| | `src/f1_localization/` | `src/localization/` |
|---|---|---|
| git | 정식 서브모듈, `heojy/f1_localization.git`, HEAD `a547822` | **자체 .git 없음** — 순수 파일 스냅샷, 상위 repo도 미추적 |
| 구성 | `als_ros` + `mocap_odom` | `f1_localization/`(구버전 스냅샷) + **`mocap_localization/`(신규 C++ 실험 패키지)** |
| 성격 | 실차 튜닝(15cm) + dead-reckoning 등 기능 추가된 본선 | 이전 갈래(capstone_ws 시절 자동덤프 파라미터) + 모캡 보정 실험 |

> 정정: 07-14 분석에서 이 스냅샷을 "commit 8d23e8c / remote 팀 repo"라 했는데, 그 dir엔 .git이 없어서 **부모 repo의 로그가 대신 응답한 것**이었음. 버전 출처는 불명(파일 타임스탬프·경로상 capstone_ws 시절).

## 2. 파라미터 차이 (`als_ros/params/mcl_params.yaml`)

| 항목 | `src/f1_localization` (신) | `src/localization/f1_localization` (구) |
|---|---|---|
| 파일 형태 | 손정리 + 한글 주석 | 알파벳 정렬 자동 덤프(`ros2 param dump`/optimizer 산출물 흔적 — `optimize_mcl_params.py` 동봉) |
| 맵 | iccas2025 | map1 / map9(map_publisher) |
| initial pose | (-1.49, -1.36, 87°) iccas2025용 | (-0.1805, -9.2037, 175.58°) record5용 |
| particle_num | 500 | 450 |
| 측정모델 | z_hit 0.9, z_rand 0.03, var_hit 0.01 | z_hit 0.65, z_rand 0.06, var_hit 0.025 |
| scan_step | 5 | 6 |
| estimate_reliability | false | true (MAE classifier 사용) |
| known_class_prior | 0.7 | 0.9 |
| `state_ekf:` 블록 | 있음 (LPF/EKF, 노이즈 공분산) | 없음 (별도 `slam_params.yaml`은 있음) |
| mae_classifier_dir | `/home/a/f1tenth_ws/...` | `/home/user/capstone_ws/...` (다른 머신 경로) |
| TF 스위치 주석 | broadcast_tf/use_odom_tf 설명 있음 (P0 fix) | 없음 |

부수 파일: 구 쪽에만 `evaluate_localization.py`, `optimize_mcl_params.py`, `slam_params.yaml`, map9/map_30 · 신 쪽에만 `bag_mcl.launch.py`.

## 3. 코드 차이 (위치인식 파이프라인)

- **`MCL.h` — 실질 기능 격차 (신형에만 존재):**
  - **dead-reckoning 모드 전환**: likelihood가 baseline 대비 30% 밑으로 5사이클 연속 떨어지면 측정업데이트 중단·추측항법 폴백, 10사이클 회복 시 복귀 (`dead_reckoning_mode_`, `bad_count_`, `good_count_`, `likelihood_baseline_`)
  - `motionYawSource_` (모션 yaw 소스 선택), odom yaw 누적(`odomYawSum_`, `prevOdomYaw_`), `newPosePub_`
- **`mcl.cpp`**: 차이 전부 한글 주석/공백, 로직 동일. **`state_ekf.cpp`**: include 한 줄(`.h`↔`.hpp`)뿐.
- **모션모델 입력 (신·구 완전 동일한 구조, 핵심 사실):**
  - `odomCB`는 **`odom.twist.twist.linear`(속도)만 사다리꼴 적분** — `odom.pose`는 **한 번도 안 읽음** (신형 [MCL.h:1882](../src/f1_localization/als_ros/include/als_ros/MCL.h#L1882), 구형도 동일)
  - 모션 델타: `deltaX/Y/Dist = odom twist 적분`, `deltaYaw = IMU 각속도 적분` (신형 [MCL.h:1843-1846](../src/f1_localization/als_ros/include/als_ros/MCL.h#L1843-L1846))
  - TF도 odom이 아니라 `mclPose_`(추정값)로 발행. odom pose 적분 코드는 주석처리 상태.

## 4. 모캡 파이프라인 실험구조 (`src/localization`에만) — 전체 해부

**구조**: ① 신규 `mocap_localization`(C++ 140줄) — `/rigid_bodies` 구독 → 캘리브레이션 R(3x3)·t(3x1) 적용(`p' = R·p + t`, `q' = q_R·q`) → map 프레임 `/mocap_pose`(PoseStamped) + 옵션 TF 발행. R·t는 params.yaml(world←mocap 변환, 예시는 45° yaw + (4.06, 4.84)). ② `mocap_odom_node.py` 개조 — 자체 yaw→쿼터니언 계산을 주석처리하고 `/mocap_pose`를 구독해 **odom의 pose만** 덮어씀(twist는 raw 모캡 델타 유지).

**판정 — MCL 성능 기여 0 (코드 경로 사실, 실측 불요):**
- 실험은 odom **pose**만 바꾸는데, §3대로 MCL(신·구 모두)은 odom pose를 **버림**. 보정 R·t가 추정기에 도달하지 못함.
- 진짜 용도 = **모캡 기반 GT/치트 로컬라이저** (mm급 map 프레임 pose·TF).
- 그마저도 현재 실전 GT 파이프라인은 **모캡 드라이버 직발행 `/mocap_pose1`(ego), `/mocap_pose2`(opp)** 사용 ([eval/tools/record_ros_topics.py](../eval/tools/record_ros_topics.py), [pose_to_tf.py](../pose_to_tf.py), ekf_pose 평가 등) — bag 뜰 때 같이 녹화하면 끝. `mocap_localization`은 **이중 orphan**.

**적용성/버그 (참고용):**
- 그 노드는 현재 **빌드 불가 추정**: TF 분기에서 `tf.header.frame_id = 'map';` — 작은따옴표 = 멀티캐릭터 문자상수 (mocap_localization.cpp:113-114).
- `publish_tf: true`로 켜면 신형 MCL의 `broadcast_tf: true`(P0 fix)와 **map→base_link 이중 발행 충돌** — GT TF로 쓸 땐 MCL TF를 끌 것.

## 5. 정량 성능 비교 (2026-07-20 실측, record5)

### 5.1 방법 = "최신 평가 기준"

- **하니스 업그레이드** ([eval/tools/analyze_localization.py](../eval/tools/analyze_localization.py)): ① robust percentile(p50/p90/p95) ② **GT 글리치 자동 제외** — 변위 기준 `dist > 0.15m + 8m/s·dt`(속도 기준은 로거 타임스탬프 지터로 오탐, 실측으로 폐기), 오염 스팬 ±1.5s(실측 폭) 병합 제외. record5에서 4개 윈도우/17.7s 검출 — §8의 5개 클러스터(17.8/23.3/27.9/29.6/33.6s)와 일치, 최대 스텝 185.9cm = 문서의 186cm ③ **차체 프레임 상수 편향**(전방/횡) 출력. selftest 포함.
- **구 코드 격리 빌드**: `src/localization/f1_localization/als_ros`를 별도 워크스페이스에 빌드(humble에서 무수정 빌드됨) → 동일 bag replay. 도구·재현 커맨드는 §5.4.
- 공통: record5 bag `--topics /scan /odom /ekf/imu/data /mocap_pose1 /mocap_pose2 --clock 200`, map1, map_server+lifecycle+static TF(base_link→laser 0.27m), GT=`/mocap_pose1`, 글리치 제외 ON, steady(>2s) 구간.

### 5.2 결과 (position err [m], steady)

| 구성 | p50 | p90 | 코너 p50 | 코너 p90 | 직선 p50 | yaw MAE(코너) | 전방편향 |
|---|---|---|---|---|---|---|---|
| **bag박제 구코드** (실차 당일 03-13) | **0.073** | 0.130 | 0.090 | 0.155 | 0.073 | 2.1° | −0.5cm |
| 구코드+구파라미터 (replay) | 0.153 | 0.277 | 0.242 | 0.366 | 0.152 | 4.5° | **−17.1cm** |
| 구코드+**신**파라미터 (replay) | 0.145 | 0.283 | 0.186 | 0.262 | 0.145 | 1.9° | **−15.8cm** |
| 신코드+신파라미터 (replay 07-16) | 0.145 | 0.283 | 0.110 | 0.241 | 0.145 | 5.7° | **−12.4cm** |
| 신코드+신파라미터 (replay 07-20) | 0.147 | 0.316 | **0.100** | 0.238 | 0.148 | 5.5° | **−13.0cm** |

run-to-run 분산(신코드 07-16 vs 07-20): p50 ±0.002, p90 ±0.03, 코너 p50 ±0.01 — 표의 0.04m급 차이는 유의미.

### 5.3 판정

1. **"신 MCL median 2배 회귀(0.073→0.145)"는 재생 아티팩트 — §8 미결 ① 종결.** 구 코드도 같은 replay 환경에서 0.145~0.153. 어떤 코드/파라미터 조합도 replay에서는 bag박제 실차 수치를 못 냄 → +7cm 바닥은 환경 속성.
2. **"−13cm 후방 편향 = laser-offset 버그" 혐의 해소 — §8 후속진단 종결.** 구 코드 replay도 −16~−17cm 동일 방향·크기. 신·구 공통 = 코드 diff가 원인이 아님(팀원에게 laser-offset diff 요청할 필요 없음). 원인은 재생환경(정확 기전 미상 — §8의 time-shift ±100ms 기각과 정합인 상수 공간 편향). **실차 검증 시 재측정 필요.**
3. **코드 차이의 실제 효과 (동일 파라미터·동일 환경, 구코드+신파라미터 vs 신코드):**
   - 위치 median 동률(0.145 vs 0.147) — 전반 정확도 회귀 없음.
   - **코너 위치는 신형이 뚜렷이 우세**: p50 0.186→0.100, p90 0.262→0.238.
   - **코너 yaw는 신형 열세**: 1.9°→5.5° (직선 1.3→1.6°). 유일한 신형 회귀 항목, 원인 미조사(DR 게이팅/모션 yaw 경로 변경 추정). 위치 15cm 대비 우선순위 낮음.
   - 이번 클린 replay에서는 어느 쪽도 track loss 없음(MAX<0.53m) → DR 폴백의 강건성 이득은 이 bag으로는 미검증(정상 주행 bag이라 발동 조건 자체가 드묾).
4. **파라미터 재튜닝(구→신)의 효과 (구코드 고정 A/B)**: 코너 p50 0.242→0.186 개선, 코너 yaw 4.5→1.9°. "15cm version" 튜닝은 실제로 코너를 개선한 정당한 리튠.
5. 결과 종합: **src/f1_localization(신형)이 성능상으로도 우세하거나 동률**(코너 yaw 제외). src/localization 스냅샷은 성능·관리 어느 면에서도 유지할 이유 없음.

### 5.4 재현 방법 + 운영 주의

```bash
# 구코드 격리 빌드 (1회)
mkdir -p /tmp/old_ws/src && cp -r src/localization/f1_localization/als_ros /tmp/old_ws/src/
cd /tmp/old_ws && source /opt/ros/humble/setup.bash && source /home/a/f1tenth/install/setup.bash
colcon build --packages-select als_ros --cmake-args -DCMAKE_BUILD_TYPE=Release

# 실행 (eval/localization-bag-replay/run_one.sh: <run이름> <params.yaml> [old_ws경로])
eval/localization-bag-replay/run_one.sh oldcode_newparam eval/localization-bag-replay/mcl_params.yaml /tmp/old_ws
eval/localization-bag-replay/run_one.sh newcode_newparam eval/localization-bag-replay/mcl_params.yaml ""
python3 eval/tools/analyze_localization.py <run>_mcl.csv <run>_gt_ego.csv
```

- ⚠️ **DDS 도메인 격리 필수 (07-20 실제 사고)**: 이 머신 기본 `ROS_DOMAIN_ID=7`에서 사용자 라이브 세션(record5 `-l` 루프 재생 + perception + rviz)과 eval 재생이 충돌 — /clock·/odom·/scan 이중 발행으로 **p50 7.5m급 전면 오염**(GT 행수 2배가 오염 신호). `run_one.sh`는 `ROS_DOMAIN_ID=42`로 격리해 둠. 향후 모든 replay 평가는 격리 도메인에서.
- 구 파라미터로 돌릴 때(`old_params.yaml`) 패치돼 있는 것: `use_sim_time`, `pose_name→newmcl_pose`(로거 호환), `mae_classifier_dir`(dead 경로→스냅샷 내 실경로; estimate_reliability:true라 필요), pose_log 경로.
- bag: `/home/a/dataset_custom/F1tenth/paper/T_iv/record5/rosbag2_2026_03_13-21_52_51`. MCL은 런치 후 ~30s 내 /scan 필요(아니면 fatal). bag 자체 `/tf` 재생 금지.
