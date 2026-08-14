# 평가 도구 및 실험 아카이브

이 디렉터리는 ROS bag 재생 평가 도구와 과거 실험 산출물을 함께 보관한다.
처음 실행할 때는 공통 도구인 [`tools/`](tools/README.md)부터 확인한다.

## 디렉터리 안내

| 분류 | 디렉터리 | 구체적인 목적 |
|---|---|---|
| 공통 | [`tools/`](tools/README.md) | ROS topic CSV 기록, perception·localization 정량 분석, 경계 발행 |
| Localization | [`localization-bag-replay/`](localization-bag-replay/README.md) | bag에 기록된 MCL과 동일 bag 재계산 MCL 비교 |
| Mocap | [`mocap-calibration-diagnostics/`](mocap-calibration-diagnostics/README.md) | 2026-07-30 mocap 변환 체인·레버암·SE(2) 진단 |
| Perception | [`perception-realcar-benchmark-2026-07-30/`](perception-realcar-benchmark-2026-07-30/README.md) | 2026-07-30 실차 bag 기준선과 파라미터 sweep |
| Perception | [`perception-static-landmark-methodology-2026-08-04/`](perception-static-landmark-methodology-2026-08-04/README.md) | 2026-08-04 정적 랜드마크 방법론 단계별 실험 |
| Perception | [`perception-march-baseline-comparison/`](perception-march-baseline-comparison/README.md) | 3월 기준 커밋과 현재 코드의 동일 bag 비교 |
| Perception | [`perception-box-recovery/`](perception-box-recovery/README.md) | box recall, wall recovery, 크기 cap 분리 검증 |
| Perception | [`perception-marker-validation-2026-08-10/`](perception-marker-validation-2026-08-10/README.md) | 2026-08-10 marker/bbox bag 분석 결과 아카이브 |
| 캡처 | [`comparison-video-capture/`](comparison-video-capture/README.md) | lap accumulation 및 SMPPI 비교 영상 실행 |

디렉터리명은 `분야-실험목적-날짜`를 사용한다. 날짜가 없는 디렉터리는 현재도
이어지는 공통 도구 또는 반복 실험이다. 기존 결과 파일의 prefix는 역사적 비교를
위해 변경하지 않는다.

## 빠른 시작

```bash
cd /home/a/f1tenth
source install/setup.bash

# 공통 분석기의 계산 자체를 ROS 없이 점검
python3 eval/tools/analyze_perception_tracking.py --selftest
python3 eval/tools/analyze_localization.py --selftest

# bag 재생·대상 노드 실행 후 공통 CSV 기록
python3 eval/tools/record_ros_topics.py --out /tmp/run_name \
  --ros-args -p use_sim_time:=true

# opponent tracking과 ego localization 분석
python3 eval/tools/analyze_perception_tracking.py /tmp/run_name
python3 eval/tools/analyze_localization.py \
  /tmp/run_name_mcl.csv /tmp/run_name_gt_ego.csv \
  --lane src/perception/data/lane/map1_path.csv
```

`record_ros_topics.py`는 한 prefix에 다음 파일을 만든다.

| suffix | 내용 |
|---|---|
| `_tracks.csv` | perception 동적 track |
| `_gt_opp.csv` | 상대 차량 mocap pose |
| `_gt_ego.csv` | ego 차량 mocap pose |
| `_mcl.csv` | `/newmcl_pose` |

## 실험 이름과 산출물 규칙

- 새 prefix: `YYYYMMDD_<bag-key>_<variant>`
- 같은 실행의 파일은 prefix를 공유하고 artifact만 suffix로 구분한다.
- `base`, `candidate`, `replay`처럼 비교 의미가 드러나는 variant를 쓴다.
- 실행 조건과 bag 경로는 README 또는 별도 manifest에 남긴다.
- 생산 YAML은 직접 덮어쓰지 않는다. 후보 파일 또는 백업·복원 trap을 사용한다.

## 판정 원칙

- 서로 다른 topic은 timestamp-nearest 방식으로 맞추고 큰 gap은 제외한다.
- mocap yaw를 GT로 쓰기 전 pivot, 축 방향, 차량별 transform을 검증한다.
- MCL과 mocap은 body pose끼리, opponent yaw는 필요하면 position course끼리 비교한다.
- 평균 하나보다 p50/p90/p95와 제외 표본 수를 함께 남긴다.
- 로컬 bag 결과와 실차/Victus 배포 검증을 같은 결론으로 취급하지 않는다.

## Git에 포함되는 것

`eval/`의 Python, shell, YAML, Markdown은 재현성을 위해 추적한다. CSV, log,
PNG, JSON, rosbag 및 `__pycache__`는 큰 산출물이므로 계속 제외한다. 보존이
필요한 결과 요약은 해당 패키지의 `docs/`에 수치와 bag 식별자를 기록한다.
