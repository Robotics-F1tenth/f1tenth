#!/bin/bash
M=/home/a/f1tenth/eval/perception-static-landmark-methodology-2026-08-04
COMBO="static_landmark.assoc_dist=0.70 static_landmark.vel_thresh=0.15 preprocess.track_assoc_gate=1.5"
echo "===== Phase 8: combined config, both sides ====="
bash "$M/run_cfg.sh" 22_52_38 p8_combo_52 $COMBO
bash "$M/run_cfg.sh" 22_55_09 p8_combo_55 $COMBO
bash "$M/run_cfg.sh" 22_48_25 p8_combo_48 $COMBO
echo
echo "===== Phase 8b: gate1.5 3rd rep + cross-bag (yaw claim) ====="
bash "$M/run_cfg.sh" 22_52_38 p8_gate1.5_c preprocess.track_assoc_gate=1.5
bash "$M/run_cfg.sh" 22_48_25 p8_gate1.5_48 preprocess.track_assoc_gate=1.5
bash "$M/run_cfg.sh" 22_48_25 p8_base_48
