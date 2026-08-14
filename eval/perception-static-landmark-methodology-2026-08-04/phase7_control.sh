#!/bin/bash
# Two-sided evaluation without positional GT:
#   bags WITH obstacles (48/52) -> landmarks are mostly true positives, want them kept
#   bag WITHOUT obstacles (55)  -> landmarks are false positives, want them gone
# A config is only "better" if it cuts 55 without gutting 48/52.
M=/home/a/f1tenth/eval/perception-static-landmark-methodology-2026-08-04
echo "===== Phase 7: negative control (22_55_09, obstacles removed) ====="
bash "$M/run_cfg.sh" 22_55_09 p7c_base
bash "$M/run_cfg.sh" 22_55_09 p7c_ad0.70   static_landmark.assoc_dist=0.70
bash "$M/run_cfg.sh" 22_55_09 p7c_vel0.15  static_landmark.vel_thresh=0.15
bash "$M/run_cfg.sh" 22_55_09 p7c_consec16 static_landmark.min_consec_frames=16
echo
echo "===== Phase 7b: positive side for the same configs (22_52_38) ====="
bash "$M/run_cfg.sh" 22_52_38 p7p_vel0.15b  static_landmark.vel_thresh=0.15
bash "$M/run_cfg.sh" 22_52_38 p7p_consec16b static_landmark.min_consec_frames=16
echo
echo "===== Phase 7c: repeat the two yaw-improving configs (noise check) ====="
bash "$M/run_cfg.sh" 22_52_38 p7r_euc0.3_b  euclidean_cluster.min_range_cluster=0.3
bash "$M/run_cfg.sh" 22_52_38 p7r_gate1.5_b preprocess.track_assoc_gate=1.5
