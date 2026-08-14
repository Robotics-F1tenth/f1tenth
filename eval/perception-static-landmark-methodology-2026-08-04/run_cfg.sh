#!/bin/bash
# run_cfg.sh <bag_key> <out_prefix> [section.key=value ...]
# Applies params to the INSTALLED yaml, runs the bag, restores the yaml afterward.
set -e
M=/home/a/f1tenth/eval/perception-static-landmark-methodology-2026-08-04
R=/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30
IY=/home/a/f1tenth/install/f1_perception/share/f1_perception/config/perception/f1tenth.yaml
BAGKEY=$1; PFX=$2; shift 2
BAG=$(ls -d /home/a/Downloads/0730/rosbag2_2026_07_30-${BAGKEY}/ 2>/dev/null)
[ -z "$BAG" ] && { echo "MISSING bag $BAGKEY"; exit 1; }

cp "$IY" "$M/.yaml_backup_$PFX"
for kv in "$@"; do
  python3 "$M/setparam.py" "$IY" "${kv%%=*}" "${kv#*=}" > /dev/null
done
bash "$R/run_bag_perception.sh" "$BAG" "$PFX" > "$M/${PFX}_run.log" 2>&1 || true
cp "$M/.yaml_backup_$PFX" "$IY"
python3 "$M/summarize.py" "$R/$PFX" "$PFX"
