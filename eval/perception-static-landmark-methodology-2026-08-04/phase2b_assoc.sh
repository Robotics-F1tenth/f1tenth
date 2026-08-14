#!/bin/bash
# assoc_dist re-sweep now that the OOB crash is fixed -- the >=0.70 range was
# previously unmeasurable, so 0.50 was picked as "highest safe value", not as an optimum.
M=/home/a/f1tenth/eval/perception-static-landmark-methodology-2026-08-04
echo "===== Phase 2b: assoc_dist re-sweep (post crash-fix), 22_52_38 ====="
for v in 0.20 0.40 0.50 0.70 1.00 1.50; do
  bash "$M/run_cfg.sh" 22_52_38 "p2b_ad${v}" static_landmark.assoc_dist=$v
done
echo "===== Phase 2b: same on 22_48_25 (cross-check) ====="
for v in 0.50 0.70 1.00; do
  bash "$M/run_cfg.sh" 22_48_25 "p2b48_ad${v}" static_landmark.assoc_dist=$v
done
