#!/usr/bin/env python3
"""Score a bag's baked /newmcl_pose against its mocap GT, no replay needed."""
import bisect, glob, math, sqlite3, sys
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def load(cur, topic, typ):
    cur.execute("SELECT id FROM topics WHERE name=?", (topic,))
    r = cur.fetchone()
    if not r:
        return []
    out = []
    for ts, d in cur.execute(f"SELECT timestamp,data FROM messages WHERE topic_id={r[0]} ORDER BY timestamp"):
        m = deserialize_message(d, typ)
        p = m.pose.pose.position if typ is Odometry else m.pose.position
        o = m.pose.pose.orientation if typ is Odometry else m.pose.orientation
        out.append((ts * 1e-9, p.x, p.y, math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))))
    return out


def pct(v, p):
    s = sorted(v)
    return s[min(int(len(s) * p / 100), len(s) - 1)]


def main(bagdir, gt_topic):
    db = glob.glob(f"{bagdir}/*.db3")[0]
    cur = sqlite3.connect(db).cursor()
    mcl = load(cur, "/newmcl_pose", PoseStamped)
    gt = load(cur, gt_topic, PoseStamped)
    if not mcl or not gt:
        print(f"  {bagdir.split('/')[-1]}: mcl={len(mcl)} gt={len(gt)} -- skip")
        return
    gts = [g[0] for g in gt]
    err, yerr, matched = [], [], []
    for t, x, y, yw in mcl:
        i = bisect.bisect_left(gts, t)
        best = None
        for j in (i - 1, i, i + 1):
            if 0 <= j < len(gt):
                dd = abs(gt[j][0] - t)
                if best is None or dd < best[0]:
                    best = (dd, gt[j])
        if best and best[0] < 0.03:
            g = best[1]
            err.append(math.hypot(g[1] - x, g[2] - y))
            dy = g[3] - yw
            yerr.append(abs(math.degrees(math.atan2(math.sin(dy), math.cos(dy)))))
            matched.append((t, x, y, yw, g))
    dur = mcl[-1][0] - mcl[0][0]
    name = bagdir.rstrip("/").split("/")[-1]
    print(f"\n=== {name} ({dur:.0f}s, {gt_topic}) ===")
    print(f"  MCL 발행률 {len(mcl)/dur:.1f} Hz   GT {len(gt)/dur:.1f} Hz   매칭 {len(err)}")
    if not err:
        print("  !! GT/MCL 시간축 매칭 0 -- 프레임 또는 시계 불일치")
        return
    print(f"  위치  p50 {pct(err,50)*100:5.1f}cm  p90 {pct(err,90)*100:5.1f}cm  p95 {pct(err,95)*100:5.1f}cm  max {max(err)*100:.0f}cm")
    print(f"  yaw   p50 {pct(yerr,50):5.2f}°   p90 {pct(yerr,90):5.2f}°   max {max(yerr):.0f}°")
    return matched


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
