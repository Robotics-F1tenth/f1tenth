# 2026-07-30 mocap 변환 체인 진단

raw `/rigid_bodies`에서 `mocap_localization` 변환 체인을 재현하고, 기존
pose·레버암·차량별 `T_map_mocap` 차이를 조사한 분석 스크립트다.

- `chain_check.py [bag]`: 공통 bag reader와 변환 체인 재현
- `se2_check.py`: 재현 pose와 baked pose 차이가 SE(2)인지 확인
- `impact.py`: 차량별 map transform 불일치 영향 확인
- `heading.py`, `leverarm.py`: 직진 heading offset과 비홀로노믹 레버암 진단
- `refit.py`: 과거 6-parameter 적합 재현
- `final_check.py`: baked pose의 course-heading 확인

일부 스크립트의 기본 bag은 `/home/a/Downloads/0730`이다. 이는 과거 bag
진단 도구이며 결과를 생산 calibration으로 바로 쓰지 않는다. 최종 변경은
`mocap_localization/scripts/calibrate_shared_map.py`의 candidate/holdout 흐름을
따르고 두 차량에 하나의 shared `T_map_mocap`을 사용한다.
