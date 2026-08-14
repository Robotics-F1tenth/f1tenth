# 2026-08-04 정적 랜드마크 방법론 실험

정적 랜드마크 누적·분절 문제를 단계별로 분해한 perception 실험이다.
`phase0.sh`부터 `phase8_combo.sh`까지는 당시 순서와 파라미터 묶음을 보존한다.

- 실행 공통부: `run_cfg.sh`, `setparam.py`
- 정량 요약: `summarize.py`, `lap_metrics.py`, `live_list.py`
- 교차 bag 판정: `crossbag_gt.py`
- 공간/원인 분석: `plot_landmarks.py`, `scatter_decomp.py`, `scatter_vs_mcl.py`
- crash 재현: `phase1_gdb.sh`, `gdb_cmds.txt`

새 기준선에 전체 phase를 무조건 재실행하지 말고, 해당 패키지 docs의 판정
기준과 bag 자산을 먼저 확인한다. `*_cfg.log`, `*_run.log`, plot은 산출물이다.
