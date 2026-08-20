#!/usr/bin/env python3
"""Click obstacle positions on a bag's accumulated scan, save them as ground truth.

The auto-derived GT (persistent unmapped returns) cannot tell a real obstacle from
an unmapped wall segment or a smear, so every score carries an asterisk. This
replaces the asterisk with a human decision, once per bag, in a couple of minutes.

    python3 annotate_bag.py <bag_dir> [--out gt.json]

You get a map-frame plot with the occupancy map in grey, everything the lidar saw
that the map does not explain in colour (brighter = seen more often), and the ego
path as a thin line. Obstacles show up as compact bright blobs.

  left click   add an obstacle
  right click  remove the nearest one
  r            toggle the raw-return layer (to see the map underneath)
  u            undo last add
  enter        save and quit
  q            quit without saving
"""
import argparse, json, math, os, sys
import numpy as np
try:
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("matplotlib is required: pip install matplotlib")
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

SCAN_TOPICS = ["/scan", "/car4/scan"]
POSE_TOPICS = ["/newmcl_pose", "/car4/newmcl_pose"]


def read_bag(bag, stride):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    scans, poses, grid, info = [], [], None, None
    while r.has_next():
        tn, data, _ = r.read_next()
        if tn == "/map" and grid is None:
            m = deserialize_message(data, OccupancyGrid)
            info = m.info
            grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        elif tn in POSE_TOPICS:
            m = deserialize_message(data, PoseStamped)
            q = m.pose.orientation
            poses.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                          m.pose.position.x, m.pose.position.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))))
        elif tn in SCAN_TOPICS:
            m = deserialize_message(data, LaserScan)
            scans.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                          np.array(m.ranges, dtype=np.float32),
                          m.angle_min, m.angle_increment, m.range_max))
    if grid is None:
        sys.exit("no /map in this bag -- annotation needs the map frame")
    if not poses:
        sys.exit(f"no pose topic (looked for {POSE_TOPICS})")
    if not scans:
        sys.exit(f"no scan topic (looked for {SCAN_TOPICS})")
    P = np.array(poses); P = P[np.argsort(P[:, 0])]
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    W, H = info.width, info.height
    acc = np.zeros((H, W), dtype=np.int32)
    for k in range(0, len(scans), stride):
        t, rng, amin, ainc, rmax = scans[k]
        i = np.searchsorted(P[:, 0], t)
        if i <= 0 or i >= len(P):
            continue
        px, py, yaw = P[i, 1], P[i, 2], P[i, 3]
        ang = amin + np.arange(len(rng)) * ainc
        ok = np.isfinite(rng) & (rng > 0.1) & (rng < min(rmax, 12.0))
        aa = ang[ok] + yaw
        X = px + rng[ok] * np.cos(aa); Y = py + rng[ok] * np.sin(aa)
        mx = ((X - ox) / res).astype(int); my = ((Y - oy) / res).astype(int)
        v = (mx >= 0) & (my >= 0) & (mx < W) & (my < H)
        np.add.at(acc, (my[v], mx[v]), 1)
    return acc, grid, info, P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--out", default=None)
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--dilate", type=float, default=0.25,
                    help="metres the map grows by before subtraction (MCL error)")
    a = ap.parse_args()
    bag = a.bag.rstrip("/")
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                f"gt_{os.path.basename(bag)}.json")
    acc, grid, info, P = read_bag(bag, a.stride)
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    occ = grid > 0
    try:
        from scipy import ndimage
        k = int(np.ceil(a.dilate / res))
        occ_d = ndimage.binary_dilation(occ, structure=np.ones((2*k+1, 2*k+1)))
    except ImportError:
        occ_d = occ; ndimage = None
        print("scipy missing -- map not dilated, wall returns will show through")
    unexplained = np.where(occ_d, 0, acc)
    extent = [ox, ox + info.width*res, oy, oy + info.height*res]
    fig, ax = plt.subplots(figsize=(9, 11))
    ax.imshow(np.where(occ, 1.0, np.nan), origin="lower", extent=extent,
              cmap="Greys", vmin=0, vmax=1.4, interpolation="nearest")
    vmax = int(np.percentile(unexplained[unexplained > 0], 97)) if (unexplained > 0).any() else 1
    raw = ax.imshow(np.where(unexplained > 0, unexplained, np.nan), origin="lower",
                    extent=extent, cmap="viridis", vmin=0, vmax=max(1, vmax),
                    interpolation="nearest")
    ax.plot(P[:, 1], P[:, 2], lw=0.4, color="tab:red", alpha=0.5, label="ego path")
    ax.set_aspect("equal"); ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"{os.path.basename(bag)}\n"
                 "left=add  right=remove  r=toggle  u=undo  enter=save  q=quit", fontsize=10)
    pts, marks = [], []

    def redraw():
        for m in marks:
            m.remove()
        marks.clear()
        for i, (x, y) in enumerate(pts):
            marks.append(ax.plot(x, y, "o", ms=11, mfc="none", mec="red", mew=2)[0])
            marks.append(ax.annotate(str(i+1), (x, y), color="red", fontsize=9,
                                     xytext=(6, 6), textcoords="offset points"))
        fig.canvas.draw_idle()

    def on_click(ev):
        if ev.inaxes is not ax or ev.xdata is None:
            return
        if ev.button == 1:
            pts.append((float(ev.xdata), float(ev.ydata)))
        elif ev.button == 3 and pts:
            d = [math.hypot(x-ev.xdata, y-ev.ydata) for x, y in pts]
            pts.pop(int(np.argmin(d)))
        redraw()

    state = {"raw": True, "saved": False}

    def on_key(ev):
        if ev.key == "r":
            state["raw"] = not state["raw"]; raw.set_visible(state["raw"]); fig.canvas.draw_idle()
        elif ev.key == "u" and pts:
            pts.pop(); redraw()
        elif ev.key == "enter":
            state["saved"] = True; plt.close(fig)
        elif ev.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    plt.show()
    if not state["saved"]:
        print("not saved"); return
    dist = ndimage.distance_transform_edt(~occ, sampling=res) if ndimage else None
    obstacles = []
    for i, (x, y) in enumerate(pts):
        mx = int((x-ox)/res); my = int((y-oy)/res)
        dw = float(dist[my, mx]) if dist is not None and 0 <= my < info.height and 0 <= mx < info.width else None
        obstacles.append({"x": x, "y": y, "label": f"obs{i+1}", "wall_dist": dw})
    json.dump({"bag": bag, "dilate": a.dilate, "obstacles": obstacles}, open(out, "w"), indent=2)
    print(f"saved {len(obstacles)} obstacles -> {out}")
    for o in obstacles:
        d = "n/a" if o["wall_dist"] is None else f"{o['wall_dist']:.2f} m"
        print(f"  {o['label']:8} ({o['x']:6.2f},{o['y']:6.2f})  wall {d}")


if __name__ == "__main__":
    main()
