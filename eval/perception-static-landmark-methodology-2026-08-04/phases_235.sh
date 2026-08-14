#!/bin/bash
# Phases 2/3/5 on 22_52_38 (same bag as the Phase 0 repeatability baseline, so every
# number here is comparable against that noise floor). Baseline-valued configs are
# skipped -- the Phase 0 reps ARE the baseline.
M=/home/a/f1tenth/eval/perception-static-landmark-methodology-2026-08-04
B=22_52_38

echo "===== Phase 2: fragmentation layer decomposition (assoc_dist fixed 0.50) ====="
echo "-- (a) clustering layer: euclidean tolerance --"
bash "$M/run_cfg.sh" $B p2_euc0.3  euclidean_cluster.min_range_cluster=0.3
bash "$M/run_cfg.sh" $B p2_euc0.8  euclidean_cluster.min_range_cluster=0.8
echo "-- (a') clustering layer: min points per cluster --"
bash "$M/run_cfg.sh" $B p2_mcs3    euclidean_cluster.min_cluster_size=3
bash "$M/run_cfg.sh" $B p2_mcs5    euclidean_cluster.min_cluster_size=5
echo "-- (b) track association gate --"
bash "$M/run_cfg.sh" $B p2_gate0.5 preprocess.track_assoc_gate=0.5
bash "$M/run_cfg.sh" $B p2_gate1.5 preprocess.track_assoc_gate=1.5

echo
echo "===== Phase 3: re-validate record5-tuned params on real-car data ====="
bash "$M/run_cfg.sh" $B p3_alpha0.3 yaw_tracker.alpha=0.3
bash "$M/run_cfg.sh" $B p3_alpha0.7 yaw_tracker.alpha=0.7
bash "$M/run_cfg.sh" $B p3_W5line   preprocess.fit_window=5  preprocess.fit_curve=false
bash "$M/run_cfg.sh" $B p3_W20line  preprocess.fit_window=20 preprocess.fit_curve=false

echo
echo "===== Phase 5: remaining static-landmark params ====="
bash "$M/run_cfg.sh" $B p5_vel0.15  static_landmark.vel_thresh=0.15
bash "$M/run_cfg.sh" $B p5_vel0.6   static_landmark.vel_thresh=0.6
bash "$M/run_cfg.sh" $B p5_consec4  static_landmark.min_consec_frames=4
bash "$M/run_cfg.sh" $B p5_consec16 static_landmark.min_consec_frames=16
bash "$M/run_cfg.sh" $B p5_drift0.3 static_landmark.drift_max=0.3
bash "$M/run_cfg.sh" $B p5_drift1.0 static_landmark.drift_max=1.0
echo "===== phases 2/3/5 done ====="
