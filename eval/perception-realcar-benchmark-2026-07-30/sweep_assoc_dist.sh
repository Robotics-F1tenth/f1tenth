#!/bin/bash
# sweep_assoc_dist.sh -- rerun one bag repeatedly with different static_landmark.assoc_dist,
# no rebuild needed (yaml param, loaded at runtime). Captures static_obstacles per value.
set -e
L="$(cd "$(dirname "$0")" && pwd)"
BAG=/home/a/Downloads/0730/rosbag2_2026_07_30-22_35_37
IY=/home/a/f1tenth/install/f1_perception/share/f1_perception/config/perception/f1tenth.yaml
VALUES="0.20 0.30 0.40 0.50 0.70 1.00"
ORIG_LINE=$(grep "assoc_dist:" "$IY")

for v in $VALUES; do
  sed -i "s/^        assoc_dist: .*/        assoc_dist: $v           # [sweep]/" "$IY"
  echo "[sweep] assoc_dist=$v"
  bash "$L/run_bag_perception.sh" "$BAG" "sweep_${v}"
done

sed -i "s|^        assoc_dist: .*|$ORIG_LINE|" "$IY"
echo "[sweep] restored: $ORIG_LINE"
