#!/bin/bash
L="$(cd "$(dirname "$0")" && pwd)"
bash "$L/sweep_assoc_dist_bag.sh" /home/a/Downloads/0730/rosbag2_2026_07_30-22_48_25 22_48_25 0.20 0.40 0.50 0.70
bash "$L/sweep_assoc_dist_bag.sh" /home/a/Downloads/0730/rosbag2_2026_07_30-22_52_38 22_52_38 0.20 0.40 0.50 0.70
