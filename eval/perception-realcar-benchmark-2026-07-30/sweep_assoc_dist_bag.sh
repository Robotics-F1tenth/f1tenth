#!/bin/bash
# sweep_assoc_dist_bag.sh <bag_path> <prefix> <values...>
set -e
L="$(cd "$(dirname "$0")" && pwd)"
BAG=$1; PFX=$2; shift 2
IY=/home/a/f1tenth/install/f1_perception/share/f1_perception/config/perception/f1tenth.yaml
ORIG_LINE=$(grep "assoc_dist:" "$IY")

for v in "$@"; do
  sed -i "s/^        assoc_dist: .*/        assoc_dist: $v           # [sweep]/" "$IY"
  echo "[sweep-$PFX] assoc_dist=$v"
  bash "$L/run_bag_perception.sh" "$BAG" "${PFX}_sweep_${v}"
done

sed -i "s|^        assoc_dist: .*|$ORIG_LINE|" "$IY"
echo "[sweep-$PFX] restored: $ORIG_LINE"
