# Perception box recall 및 wall recovery 분리 검증

기존 크기 cap 제거, `wall_recovery_enabled`, off-ROI false positive의 영향을
같은 bag에서 분리해 비교한 실험 묶음이다.

- `run_variant.sh <bag-key> <out-prefix> <true|false>`: wall recovery만 전환
- `run_variant_cap.sh <bag-key> <out-prefix> <size-threshold>`: 이전 크기 cap 비교
- `summarize_box.py <prefix> [box-x box-y radius]`: box ROI와 외부 검출 요약

`on_`, `off_`, `cap04_`, `regress_` prefix는 각각 기능 on/off, 0.4m cap,
회귀 확인을 뜻한다. CSV/log/txt는 산출물이며 Git에는 포함하지 않는다.
