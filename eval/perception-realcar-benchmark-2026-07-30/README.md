# 2026-07-30 실차 perception bag 기준선 및 sweep

2026-07-30 bag을 replay해 동적 track, 정적 landmark, MCL/mocap 보조 지표를
수집한 가장 큰 결과 아카이브다.

- `run_bag_perception.sh <bag-path> <out-prefix>`: 단일 bag 공통 수집
- `run_all.sh`, `run_remaining.sh`: 당시 batch 실행 기록
- `sweep_assoc_dist_bag.sh`: bag과 값을 명시하는 association-distance sweep
- `parse_static_obstacles.py`: topic echo를 landmark 메시지 단위로 요약
- `fragmentation_correlate.py`: landmark birth와 ego yaw-rate 관계 분석

새 실행에는 의미가 불분명한 `p3`, `p7c` 같은 prefix를 추가하지 말고 상위
[명명 규칙](../README.md#실험-이름과-산출물-규칙)을 사용한다. 기존 prefix는
과거 문서·결과 링크 보존을 위해 변경하지 않는다.
