# C-03 — MCL TF 시간 계약 실험 (2026-08-15)

로드맵(`localization-architecture-performance-roadmap-2026-08.md`)의 **C-03**
"pose 는 scan stamp, TF 는 `now()+0.05` 라 동일 상태의 시간 의미가 소비자마다
다르다"를 정량화하고, 수정안을 파라미터로 구현해 A/B 한 기록.

## 결론

| | 값 |
|---|---|
| TF 라벨이 실제 유효시각보다 앞선 양 (실측) | **70.1 ms** (p50) |
| 그로 인한 자차 위치오차 (오프라인 스윕) | **17.40 cm → 6.10 cm** (shift 80 ms 에서 최소) |
| 상대차 트래커 위치오차 (풀스택 A/B) | **0.175 → 0.152 m** (mean), rmse 0.207 → 0.188 |
| 대가 | 처리 스캔 3553 → 3404 (**-4.2%**), 매칭 82.6% → 80.2% |
| TF lookup 실패 | **0 → 0** (우려했던 가용성 문제는 발생 안 함) |

**기본값은 `publish_time`(레거시) 유지.** 아래 §4 의 처리량 손실을 먼저 해결해야
하고, 로드맵이 C-03 을 대회 직전 변경에 섞지 말라고 명시했기 때문.

## 1. 왜 생겼나

`broadcastTF()` 가 `now() + transform_tolerance` 로 stamp 를 찍는다. 이건 AMCL 관행인데,
**AMCL 에서는 안전하고 여기서는 위험하다**:

| | AMCL | 이 노드 |
|---|---|---|
| 발행 TF | `map -> odom` (느리게 변하는 보정) | **`map -> base_link`** (차의 전체 운동) |
| 빠른 운동은 어디에 | `odom -> base_link`, 오도메트리가 정확히 stamp | 없음 (`use_odom_tf=false`) |
| 미래 stamp 의 결과 | 거의 무해 | **그대로 pose 지연** |

구성 요소:
```
스캔 시각 s 의 자세
  → 계산 완료 s + 20~26 ms (측정모델이 지배적)
  → 라벨 = (s + 25 ms) + transform_tolerance 0.05 = s + 75 ms
실측 70.1 ms
```

`/newmcl_pose` 는 `scan_.header.stamp` 를 그대로 쓴다(실측 차이 0.000 ms). 그래서
**같은 자세인데 발행 경로에 따라 시간 라벨이 다르다** — 토픽 소비자(`ekf_pose`)는
정상, TF 소비자(`perception`)만 지연을 받는다.

## 2. 오프라인 검증 (`tf_lag_test.py`)

bag 의 baked `/tf` 와 mocap GT 만으로, TF 조회 시각을 `T + shift` 로 옮겨가며
자차 위치오차를 잰다. 오차가 최소가 되는 shift 가 곧 라벨이 앞선 양.

```
shift[ms]   p50[cm]
        0     17.40      <- 현재 동작
       70      6.47
       80      6.10      <- 최소
      100      7.30
```

노드를 하나도 안 띄우고 재현 가능하다:
```bash
python3 eval/tf-time-contract-c03/tf_lag_test.py [bag_path]
```

## 3. 풀스택 A/B (`run_ab.sh`)

라이브 MCL + 라이브 perception 을 bag 센서로 구동. bag 의 baked `/tf` 는 재생하지
않고(`/tf_static` 만 재생) 라이브 MCL 이 `map->base_link` 를 발행한다.

```bash
bash eval/tf-time-contract-c03/run_ab.sh legacy    eval/tf-time-contract-c03/legacy.yaml
bash eval/tf-time-contract-c03/run_ab.sh statetime eval/tf-time-contract-c03/statetime.yaml
python3 eval/tools/analyze_localization.py <pfx>_mcl.csv <pfx>_gt_ego.csv
python3 eval/tools/analyze_perception_tracking.py <pfx>
```

| | legacy (`publish_time`) | `state_time` |
|---|---|---|
| 자차 MCL p50 | 0.099 m | 0.096 m |
| 자차 종방향 편향 | +0.0 cm | −0.7 cm |
| 상대차 위치오차 mean | 0.175 m | **0.152 m** |
| 상대차 위치오차 rmse | 0.207 m | **0.188 m** |
| 처리 스캔 | 3553 | 3404 |
| 매칭률 | 82.6% | 80.2% |
| `tf_not_aligned_drop` | 0 | 0 |

자차 오차가 두 모드에서 사실상 같은 건 **정상이고 sanity check 역할**을 한다 —
`tf_stamp_source` 는 TF 만 바꾸고 `/newmcl_pose` 는 안 건드리기 때문.

상대차 개선폭(−13%)이 오프라인 자차 예측(−65%)보다 작은 건 트래커 오차에 다른
성분이 많기 때문이다. 특히 **mocap 캘리브레이션의 차량 간 프레임 불일치 13 cm**
(`docs/mocap-calibration-analysis-2026-08.md`)가 바닥으로 깔려 있어, TF 수정으로는
건드릴 수 없다.

## 4. 대가의 정체 — lookup 실패가 아니라 파이프라인 직렬화

수정 전 예상은 "미래 시각 조회가 실패해 측정이 폐기된다"였으나 **실측 `tf_not_aligned_drop`
은 양쪽 0**이었다. perception 의 `lookupTransform` 은 0.1 s 타임아웃으로 **블로킹**
하고 `TransformListener` 가 자체 스레드로 버퍼를 채우므로, 추가 대기는 흡수된다.

실제로 나타난 손실은 **처리량**이다:
- `publish_time`: 요청 시각 `T` 가 이미 미래 라벨 항목들로 덮여 있어 **즉시 반환**
- `state_time`: `T` 를 만족하려면 **스캔 `T` 의 보정 결과**가 필요 → MCL 을 ~25 ms 대기
- 단일스레드 executor 라 이 대기를 다른 일과 겹칠 수 없어 사이클이 길어지고, 그동안
  들어온 스캔을 건너뛴다 (3553 → 3404)

즉 **정확도 ↔ 처리량**의 교환이며, 원인은 시간 계약이 아니라 executor 구조다.
`state_time` 을 기본으로 올리기 전에 perception 을 멀티스레드 executor /
콜백 그룹으로 바꿔 TF 대기와 클러스터링을 겹치게 하는 것이 순서다.

## 5. 남은 것

- [ ] perception executor 구조 개선 후 `state_time` 재측정 — 처리량 손실이 사라지면
      기본값 전환 근거가 된다
- [ ] `transform_tolerance` 의 의미 재정의. 현재 `state_time` 은 tolerance 를 아예
      안 쓴다(정확한 stamp). tolerance 가 필요한 소비자가 있는지 확인 필요
- [ ] 반복 측정 — 위 표는 조건당 1회다. MCL 은 파티클 필터라 run-to-run 분산이 있으므로
      기본값 전환 전에는 조건당 5회 이상으로 재확인할 것
