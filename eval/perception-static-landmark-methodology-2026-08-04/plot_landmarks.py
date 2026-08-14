#!/usr/bin/env python3
"""Plot final confirmed landmarks over the bag's own /map, colored by birth lap.

The question this answers: the run ends with more landmarks than there were real
obstacles, and every late-born one keeps being re-detected to the last lap. So
WHERE are they? On the wall? On the racing line? Stacked next to a real cone?

    python3 plot_landmarks.py <bag_key> <run_prefix> [out.png]
"""
import sys, os
sys.path.insert(0, "/home/a/f1tenth/eval/perception-static-landmark-methodology-2026-08-04")
sys.path.insert(0, "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30")
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid

from parse_static_obstacles import parse, parse_stamps
from lap_metrics import ego_track, lap_bounds

R = "/home/a/f1tenth/eval/perception-realcar-benchmark-2026-07-30"

for p in ["/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
          "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"]:
    if os.path.exists(p):
        font_manager.fontManager.addfont(p)
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=p).get_name()
        break
plt.rcParams["axes.unicode_minus"] = False


def read_map(bag_key):
    bag = f"/home/a/Downloads/0730/rosbag2_2026_07_30-{bag_key}"
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/map"]))
    while rd.has_next():
        _, d, _ = rd.read_next()
        return deserialize_message(d, OccupancyGrid)
    return None


def main():
    bag_key, pfx = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else f"/tmp/{pfx}_landmarks.png"

    laps, dur, t0 = lap_bounds(bag_key)
    t, ex, ey, _ = ego_track(bag_key)
    grid = read_map(bag_key)

    path = f"{R}/{pfx}_static_obstacles.txt"
    msgs, stamps = parse(path), parse_stamps(path)
    times = [s - t0 for s in stamps]

    def lap_of(tt):
        for i, lt in enumerate(laps):
            if tt < lt:
                return i
        return len(laps) - 1

    born, last = {}, {}
    for tt, m in zip(times, msgs):
        for id_, x, y, v in m:
            if id_ not in born:
                born[id_] = lap_of(tt)
            last[id_] = (x, y)   # final refined position

    fig, ax = plt.subplots(figsize=(11, 9))
    if grid is not None:
        w, h = grid.info.width, grid.info.height
        res, ox, oy = grid.info.resolution, grid.info.origin.position.x, grid.info.origin.position.y
        img = np.array(grid.data, dtype=np.int8).reshape(h, w).astype(float)
        img[img < 0] = 50
        ax.imshow(img, cmap="gray_r", origin="lower", alpha=0.45,
                  extent=[ox, ox + w * res, oy, oy + h * res], vmin=0, vmax=100)

    ax.plot(ex, ey, "-", color="#4a90d9", lw=0.7, alpha=0.5, label="자차 궤적")

    cmap = plt.get_cmap("autumn_r")
    nl = max(len(laps), 1)
    for id_, (x, y) in sorted(last.items()):
        b = born[id_]
        early = b == 0
        ax.scatter(x, y, s=230 if early else 170,
                   facecolor=("#2ca02c" if early else cmap(0.25 + 0.7 * b / nl)),
                   edgecolor="black", linewidth=1.4, zorder=5,
                   marker="o" if early else "^")
        ax.annotate(f"{int(id_) - 1000}\nL{b+1}", (x, y), fontsize=7.5, ha="center",
                    va="center", zorder=6,
                    color="white" if early else "black", weight="bold")

    n_early = sum(1 for v in born.values() if v == 0)
    ax.scatter([], [], s=200, facecolor="#2ca02c", edgecolor="black",
               marker="o", label=f"랩1에 생김 ({n_early}개)")
    ax.scatter([], [], s=170, facecolor=cmap(0.7), edgecolor="black",
               marker="^", label=f"랩2 이후 생김 ({len(born)-n_early}개)")
    ax.legend(loc="upper right", fontsize=10, framealpha=0.92)
    ax.set_title(f"{bag_key} — 최종 확정 정적장애물 {len(born)}개 (숫자=id, L=생긴 랩)",
                 fontsize=13)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect("equal"); ax.grid(alpha=0.25)
    pad = 1.0
    ax.set_xlim(min(ex) - pad, max(ex) + pad)
    ax.set_ylim(min(ey) - pad, max(ey) + pad)
    fig.tight_layout()
    fig.savefig(out, dpi=155)
    print(f"saved {out}  ({len(born)} landmarks, {n_early} from lap1)")


if __name__ == "__main__":
    main()
