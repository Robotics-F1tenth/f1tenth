# 모션캡처 캘리브레이션 분석 — 차량별 캘리 구조와 GT 정확도 (2026-08-05)

질문: "3호차/4호차 각각 캘리브레이션하면 GT 정확도가 더 높지 않나? 불가능한가?
지금 코드 구조는? 어딜 바꿔야 하나?"

**한 줄 답**: 차량별 캘리는 **이미 구현돼 있고 이미 쓰이고 있다**. 진짜 문제는 다른 데
있다 — (1) 두 차가 **공유해야 할 변환(T_map_mocap)을 각자 따로 적합**해서 좌표계가
13cm 어긋나 있고, (2) **상대차 GT 헤딩이 10~12° 틀어져 있다.**

분석 도구: `eval/mocap-calibration-diagnostics/` · 데이터: 0730 bag(`22_52_38`, `22_25_24`)

---

## 0. 결론 요약

| # | 발견 | 크기 | 확신도 |
|---|---|---|---|
| 1 | 차량별 캘리는 이미 구현·사용 중 (전제가 outdated) | — | 코드 확인 |
| 2 | `T_map_mocap`을 차마다 따로 적합 — 물리적으로 공유돼야 함 | 1.43°, 9.3cm | 파라미터 직접 비교 |
| 3 | └ 주행영역에서 두 차 좌표계가 어긋남 | **p50 13cm, max 20cm** | bag 실측 |
| 4 | └ 두 차 사이 거리가 왜곡됨 (SE(2) 불변량인데) | **p50 10cm, bias +5.8cm** | bag 실측 |
| 5 | 손으로 찍은 소수 샘플 적합 → 전궤적 재적합이 훨씬 나음 | 10.84 → 7.05cm (-35%) | bag 재적합 |
| 6 | car1의 `T_map_mocap`이 더 틀림 (car2 값이 car1 데이터에 더 잘 맞음) | 7.79 vs 7.31cm | 민감도 스캔 |
| 7 | **상대차 GT 헤딩이 크게 틀어짐** | **10~12°** | 2개 bag 교차검증 |

---

## 1. 지금 코드 구조

### 변환 체인 (3단)

```
T_map_base  =  T_map_mocap          o  T_mocap_point       o  T_point_base
               (params: rotation,      (mocap 측정값,          (params:
                translation)            input_axes로 축변환)     tracked_point_to_base_*)
             = "방 -> 맵" 공유변환      = 리지드바디 원시 포즈     = "마커중심 -> 후륜축" 차량별
```

`mocap_localization.cpp:160-178`이 이 순서 그대로 구현. `calibration_v2.py`가 6개
파라미터(`a,tx,ty` = T_map_mocap / `b,bx,by` = T_point_base)를 동시에 적합.

### 차량별 분리는 이미 다 돼 있음

| 항목 | car1 (params.yaml) | car2 (params2.yaml) |
|---|---|---|
| `rigid_body_name` | `'6'` | `'7'` |
| `output_topic` | `/mocap_pose1` | `/mocap_pose2` |
| launch | `mocap_localization.launch.py` | `mocap_localization2.launch.py` |
| 캘리 실행 | `calibration_v2.py --ros-args -p obj:=1` | `... -p obj:=2` |

`.cpp`가 `rigid_body_name`으로 필터링(`:140-155`)하고 `output_topic`으로 따로 발행하며
`tracked_point_to_base_*`까지 적용한다. **"한 대 기준으로만 캘리한다"는 전제는 outdated** —
`calibration_v2.py`/`params2.yaml`/`launch2`가 8/2에 추가되면서 이미 차량별로 분리됨.

> 참고: 예전 분석(`localization-version-comparison.md` §4)에서 지적했던
> `tf.header.frame_id = 'map'` 작은따옴표 버그도 이때 같이 고쳐졌다.

### 체인 재현 검증

raw `/rigid_bodies`에서 현재 params로 체인을 재현하니 bag에 구워진
`/mocap_pose1`과 **p50 0.9mm**로 일치 (시간축 정렬 기준). 즉 현재 params가 이 bag을
만든 그 캘리브레이션이고, **bag으로 현재 캘리를 직접 평가할 수 있다.**

---

## 2. 문제 1 — 공유돼야 할 변환을 따로 적합하고 있다 ★

`T_map_mocap`은 **모캡 원점과 맵 원점 사이의 변환**이다. 방과 맵의 성질이지
차량의 성질이 아니다 — **두 차가 반드시 같은 값을 써야 한다.** 그런데:

| | T_map_mocap yaw | translation | T_point_base yaw | T_point_base xy |
|---|---|---|---|---|
| car1 | **-89.951°** | [+0.0605, -4.7448] | +2.875° | [-0.0367, -0.0646] |
| car2 | **-91.382°** | [+0.1095, -4.6660] | -3.079° | [-0.0159, +0.0622] |
| 차이 | **1.431°** | **9.28cm** | | |

### 왜 이렇게 됐나 — 파라미터 축퇴(degeneracy)

yaw 잔차식은 `yaw_pred = a + yaw_moc + b` 이라 **`a`와 `b`가 합(a+b)으로만 나타난다.**
`a`는 위치식의 `R(a)·p_moc` 항으로만 따로 관측되는데, 손으로 찍은 소수 샘플에서는 이
항의 정보가 약하다. 그 결과 `a`가 얕은 골짜기에 놓여 세션마다 다른 값으로 착지한다.

실제로 `a`를 스캔하며 나머지 5개를 매번 재적합한 결과 (car1 데이터, 전궤적):

| a [deg] | 잔차 p50 [cm] | |
|---|---|---|
| -89.00 | 10.11 | |
| **-89.95** | **7.79** | ← car1 현재값 |
| -90.71 | 7.05 | 전궤적 6-DOF 최적 |
| -91.00 | **6.89** | 스캔 최소 |
| **-91.38** | **7.31** | ← car2 현재값 |
| -92.00 | 8.43 | |

**car2의 값(-91.38°)이 car1 자신의 값(-89.95°)보다 car1 데이터를 더 잘 설명한다.**
즉 참값은 -91° 근처이고 **car1의 캘리가 더 틀렸다.**

### 실제 피해 — bag 실측

두 `T_map_mocap`을 같은 mocap 점들에 적용했을 때의 좌표계 어긋남
(주행영역: 모캡 원점에서 p50 3.5m, max 5.3m):

```
body '6' 주행영역:  p50 13.04cm   p95 19.12cm   max 20.07cm
body '7' 주행영역:  p50 13.09cm   p95 19.59cm   max 21.48cm
```

더 결정적인 것 — **두 차 사이의 거리는 어떤 강체변환에도 불변**이어야 하는데,
각자 다른 프레임을 쓰니 왜곡된다:

```
원본(모캡 프레임) 두 차 거리:  p50 1.931m
각자 캘리 적용 후 거리 오차 :  p50 9.95cm   p95 17.93cm   max 19.53cm
                    계통 bias:  +5.81cm
```

**이건 perception 벤치마크에 직접 영향을 준다.** 트래커는 ego의 MCL 포즈(car1 프레임에
고정)에 라이다 관측을 얹어 상대차 위치를 낸다. 그런데 그 정답은 `/mocap_pose2`(car2
프레임)다 — **두 프레임이 13cm 어긋나 있으면 그 차이가 그대로 "트래커 오차"로 잡힌다.**
0730 perception 벤치마크의 위치오차가 0.185~0.411m였으니 무시할 수 없는 몫이다.

---

## 3. 문제 2 — 상대차 GT 헤딩이 10~12° 틀어져 있다 ★★

직진 구간(|omega|<0.15 rad/s, 속도 1~8 m/s)에서는 슬립이 거의 없으므로
**진행방향(course)과 차체 헤딩이 일치해야 한다.** 최종 산출물에서 직접 측정:

| bag | `/mocap_pose1` (ego) | `/mocap_pose2` (상대차) |
|---|---|---|
| 22_52_38 | 중앙값 **+1.02°** (원형평균 +0.98°) | 중앙값 **+12.08°** (원형평균 +10.60°) |
| 22_25_24 | 중앙값 **+5.19°** (원형평균 +5.26°) | 중앙값 **+10.53°** (원형평균 +3.56°) |

**상대차 GT 헤딩이 10~12° 계통 오차를 갖고 있다.** ego는 1~5°로 훨씬 낫다.

독립적으로 교차검증됨 — 후륜축 횡속도 RMS(비홀로노믹 구속)로 봐도 body '7'은
**현재 보정을 적용하면 무보정보다 오히려 나빠진다**(0.747 vs 0.627 m/s). 보정 부호가
반대 방향이라는 뜻이고, 위 course 측정과 결론이 일치한다.

원인 추정: Motive에서 리지드바디 '7'을 정의할 때 차가 비스듬한 상태였거나 피벗 축이
차체 전방과 어긋나게 잡힘. `calibration_v2.py`가 `b`를 적합하긴 하는데, §2의 축퇴 때문에
`a`와 뒤섞여 잘못된 지점에 착지한 것으로 보인다.

### perception 벤치마크에 미치는 영향

- **yaw 지표는 영향 없음** — `eval/tools/analyze_perception_tracking.py`는 GT course를 **위치 차분에서**
  구하고 모캡 쿼터니언을 안 쓴다(docstring에 명시). 이 설계 덕분에 우연히 방어됨.
- **위치 지표는 §2의 13cm 프레임 어긋남 쪽이 지배적**이고, 헤딩 오차가 `t_point_base`
  (6.4cm)를 잘못 회전시켜 더하는 몫은 ~2cm로 작다.

---

## 4. 문제 3 — 소수 샘플 손 캘리의 한계

`calibration_v2.py`는 스페이스바로 샘플을 찍고 최소 3개면 적합한다. 같은 데이터를
**전궤적(2247샘플)로 재적합**하면:

| | a | b | 잔차 p50 | p95 |
|---|---|---|---|---|
| 현재 params.yaml | -89.951° | +2.875° | **10.84cm** | 34.48cm |
| 전궤적 재적합 | -90.706° | +4.585° | **7.05cm** | 29.60cm |

**35% 개선.** 참고로 이 bag의 MCL 자체 오차가 p50 7.6cm 수준이므로 7.05cm는 사실상
노이즈 바닥에 도달한 값이다 — 즉 **전궤적 적합은 더 짜낼 게 없을 만큼 잘 맞고,
현재 손 캘리는 그 위에 ~8cm의 캘리 오차를 얹고 있다.**

> 주의: 이 캘리 방식은 **MCL을 기준으로 모캡을 맞추는** 구조라, 모캡 GT가 MCL의 계통
> 오차를 그대로 물려받는다. "GT가 MCL보다 정확할 수 없다"는 구조적 상한이 있다.
> 진짜로 독립적인 GT를 원하면 맵-모캡 정합을 측량(surveyed point) 기반으로 해야 한다.

---

## 5. 어딜 바꿔야 하나

### 5-A. 즉시 (코드 수정 없이, 가장 큰 이득)

**두 params 파일의 `rotation`/`translation`을 같은 값으로 통일.** §2 스캔 기준
`a ≈ -91.0°` 근방이 최적이고, 그 뒤 각 차의 `tracked_point_to_base_*`만 다시 적합.
이것만으로 13cm 프레임 어긋남과 10cm 차간거리 왜곡이 사라진다.

### 5-B. `calibration_v2.py` 구조 수정

현재: 차마다 6개 파라미터를 **독립적으로** 적합 → `a`가 세션마다 다르게 착지
```python
# 지금: obj=1이면 params.yaml, obj=2면 params2.yaml 을 각각 6-DOF 적합
params = [a, tx, ty, b, bx, by]   # 차량마다 따로
```
바꿀 것: **`T_map_mocap`을 두 차가 공유하는 단일 미지수로 묶고, 두 세션의 샘플을
한꺼번에 적합**
```python
# 제안: 공유 3개 + 차량별 3개씩
params = [a, tx, ty,          # 공유 (두 차 샘플 전부가 함께 구속)
          b1, bx1, by1,       # car1 전용
          b2, bx2, by2]       # car2 전용
```
- 샘플도 손으로 찍지 말고 **주행 궤적 전체**를 쓸 것 (§4, 35% 개선)
- 각 차가 ego(MCL 구동) 역할일 때의 세션이 하나씩 필요 — 지금 절차와 동일, 다만
  **두 세션 데이터를 합쳐서 한 번에 푸는 것**이 핵심

#### 구현됨 (2026-08-15)

`calibration_v2.py` 를 고치는 대신, 표본 기록과 적합을 분리한 도구 두 개를 추가했다.
기존 스크립트는 그대로 두고(호환), 새 캘리는 이 경로를 쓴다.

```bash
# 1) 각 차량이 ego(MCL 구동) 일 때 한 번씩, 트랙을 돌면서 전 궤적 표본을 기록
ros2 run mocap_localization calibration_record --ros-args \
    -p mocap_topic:=/mocap_pose3 -p out:=/tmp/car3_samples.csv
ros2 run mocap_localization calibration_record --ros-args \
    -p mocap_topic:=/mocap_pose4 -p out:=/tmp/car4_samples.csv

# 2) 두 표본을 한꺼번에 풀어 두 params 를 동시에 갱신
ros2 run mocap_localization calibration_solve_joint \
    car3=/tmp/car3_samples.csv:src/localization/mocap_localization/params/params3.yaml \
    car4=/tmp/car4_samples.csv:src/localization/mocap_localization/params/params4.yaml
```

`a, tx, ty` 는 공유 미지수, `b_i, bx_i, by_i` 는 차량별 미지수인 LM 문제를 푼다.
출력된 두 params 의 `rotation`/`translation` 은 **정의상 동일**하므로 §2 의 13 cm
어긋남이 재발할 수 없다.

셀프테스트(`calibration_solve_joint.py --selftest`)가 축퇴를 직접 재현한다:

```
참값 a = -91.000 deg
표본 많고 넓음: 공유 -90.998 / 독립 [-90.997, -91.0]   -> 둘 다 맞음
표본 적고 좁음: 공유 -89.609 / 독립 [-91.131, -87.354] -> 독립은 차마다 3.78 deg 갈림
```

**주의**: 소표본에서는 공유 적합도 참값에서 1.4° 벗어난다. 즉 **공유 적합만으로는
부족하고 전 궤적 표본이 함께 필요하다** — 공유 적합이 보장하는 것은 "두 차가 같은
프레임에 산다"이지 "그 프레임이 정확하다"가 아니다.

### 5-C. 근본 해결 (권장, 하드웨어/절차)

**Motive에서 리지드바디 피벗을 차량 후륜축(base_link)에 맞춰 정의.**
지금 `T_point_base`가 6~7cm/3° 붙어있는 건 Motive가 마커 클라우드 중심을 자동으로
피벗으로 잡기 때문이다. 피벗을 base_link에 맞추면 `T_point_base ≈ identity`가 되어
**차량별 캘리 자체가 필요 없어지고, 공유 `T_map_mocap` 하나만 맞추면 된다.**
§3의 헤딩 10~12° 문제도 이 단계에서 같이 사라진다.

### 5-D. 헤딩만이라도 지금 고치기 (MCL 세션 불필요)

§3에서 쓴 방법 — **직진 구간의 course-heading 편차**는 모캡 데이터만으로 구해지고
MCL도 맵도 필요 없다. 상대차의 10~12° 헤딩 오차는 이 값으로 바로 보정 가능하다
(`eval/mocap-calibration-diagnostics/heading.py`). 5-C 전까지의 임시 조치로 유효.

---

## 6. "차량별 캘리가 불가능한가?" 에 대한 직접 답

**불가능하지 않고, 이미 하고 있다.** 다만 지금 방식은 차량별로 **너무 많이** 자유롭게
적합하고 있는 게 문제다 — 공유돼야 할 `T_map_mocap`까지 차마다 따로 풀어서 두 차가
서로 다른 맵 프레임에 살고 있다. 올바른 분해는:

| 변환 | 성질 | 어떻게 |
|---|---|---|
| `T_map_mocap` | **방+맵의 성질 → 공유** | 두 세션 데이터 합쳐 한 번에, 또는 측량 |
| `T_point_base` | **차량의 성질 → 차량별** | 차량별 적합, 또는 Motive 피벗으로 제거 |

"각각 캘리하면 GT가 더 정확해진다"는 직관은 `T_point_base`에 대해서는 맞고,
`T_map_mocap`에 대해서는 **오히려 반대**다 — 따로 풀수록 두 차의 GT가 서로 어긋난다.

---

## 7. 재현

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 eval/mocap-calibration-diagnostics/chain_check.py    # 체인 재현 + baked 대조
python3 eval/mocap-calibration-diagnostics/impact.py         # 프레임 어긋남 / 차간거리 왜곡
python3 eval/mocap-calibration-diagnostics/refit.py          # 전궤적 재적합 + a 민감도 스캔
python3 eval/mocap-calibration-diagnostics/heading.py        # 직진구간 헤딩 오프셋 (모캡 단독)
python3 eval/mocap-calibration-diagnostics/final_check.py    # 최종 baked 포즈 헤딩 검증 (2 bag)
```

## 8. 남은 것 / 한계

- [ ] 5-A(두 params의 T_map_mocap 통일) 적용 후 perception 벤치마크 재측정 —
      위치오차가 실제로 줄어드는지 확인 (예상: 수 cm)
- [ ] `T_point_base`의 xy(dx,dy)는 이번 분석으로 확정 못 함. 비홀로노믹 구속만으로는
      실제 슬립각(4~7°)과 혼입돼 분리가 안 되고, `dy`는 관측성이 매우 낮다
      (최적화가 비물리적 0.87m로 발산). 차량별 xy는 **직접 실측**(마커 중심 ↔ 후륜축)
      또는 5-C가 정답.
- [ ] 이 캘리 방식의 구조적 상한(모캡 GT가 MCL 오차를 물려받음, §4 주의) 해소는
      측량 기반 정합이 필요 — 별도 과제.
