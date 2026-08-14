#!/bin/bash
M=/home/a/f1tenth/eval/perception-static-landmark-methodology-2026-08-04
echo "===== Phase 0: repeatability (22_52_38, identical config x5) ====="
for i in 1 2 3 4 5; do
  bash "$M/run_cfg.sh" 22_52_38 "p0_rep${i}"
done
