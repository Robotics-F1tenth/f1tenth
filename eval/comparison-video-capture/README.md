# 비교 영상 캡처 실행

- `run_lap_accum.sh before|after <bag-key>`: 정적 landmark lap 누적 비교
- `run_smppi_sim.sh base|after`: SMPPI base와 barrier/grip 후보 비교
- `params_base.yaml`: SMPPI base 영상용 고정 파라미터

두 스크립트는 별도 `ROS_DOMAIN_ID`와 종료 trap을 사용한다. 실행 전 경로와
대상 프로세스를 읽고, 현재 차량 세션과 domain이 겹치지 않는지 확인한다.
