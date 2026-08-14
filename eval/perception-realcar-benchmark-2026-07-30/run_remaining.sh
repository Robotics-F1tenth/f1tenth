#!/bin/bash
L="$(cd "$(dirname "$0")" && pwd)"
D=/home/a/Downloads/0730
BAGS="22_28_57 22_32_09 22_35_37 22_41_13 22_48_25 22_52_38 22_55_09"
for b in $BAGS; do
  full=$(ls -d "$D"/rosbag2_2026_07_30-${b}/ 2>/dev/null)
  if [ -z "$full" ]; then echo "MISSING bag $b"; continue; fi
  echo "===== $b ====="
  bash "$L/run_bag_perception.sh" "$full" "$b"
done
