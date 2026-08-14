# 공통 평가 도구

| 파일 | 역할 | ROS 필요 |
|---|---|---|
| `record_ros_topics.py` | perception, MCL, ego/opponent mocap을 공통 prefix CSV로 기록 | 필요 |
| `analyze_perception_tracking.py` | opponent GT course 대비 track 위치·yaw·가용성 분석 | 불필요 |
| `analyze_localization.py` | ego mocap body pose 대비 MCL 위치·yaw 분석 | 불필요 |
| `publish_track_boundaries.py` | control 없이 center/left/right path를 반복 발행 | 필요 |

공통 실행 예제와 산출물 suffix는 [상위 사용 설명서](../README.md)를 따른다.
분석 로직을 바꿀 때는 두 analyzer의 `--selftest`를 먼저 통과시킨다.
