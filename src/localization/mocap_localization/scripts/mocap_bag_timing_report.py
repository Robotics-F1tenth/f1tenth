#!/usr/bin/env python3
"""
Report measurement-header versus rosbag-record timing for mocap topics.

For the upstream OptiTrack driver, header.stamp is a ROS-host reconstruction of
camera mid-exposure time and frame_number is a driver-local callback counter.
"""

import argparse
from pathlib import Path
import sqlite3
from geometry_msgs.msg import PoseStamped
from mocap4r2_msgs.msg import RigidBodies
from rclpy.serialization import deserialize_message


TOPIC_TYPES = {
    "/rigid_bodies": RigidBodies,
    "/mocap/car3/pose": PoseStamped,
    "/mocap/car4/pose": PoseStamped,
    "/mocap_pose1": PoseStamped,
    "/mocap_pose2": PoseStamped,
}


def percentile(values, fraction):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def find_db3(path):
    path = Path(path).expanduser().resolve()
    if path.is_file():
        return [path]
    return sorted(path.glob("*.db3"))


def read_rows(databases):
    rows = {topic: [] for topic in TOPIC_TYPES}
    rigid_body_counts = {}
    for database in databases:
        connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
        try:
            topics = dict(connection.execute("SELECT id, name FROM topics"))
            for topic_id, topic_name in topics.items():
                msg_type = TOPIC_TYPES.get(topic_name)
                if msg_type is None:
                    continue
                query = "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp"
                for receive_ns, raw in connection.execute(query, (topic_id,)):
                    msg = deserialize_message(raw, msg_type)
                    stamp = msg.header.stamp
                    header_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                    frame = int(msg.frame_number) if topic_name == "/rigid_bodies" else None
                    rows[topic_name].append((int(receive_ns), header_ns, frame))
                    if topic_name == "/rigid_bodies":
                        for rigid_body in msg.rigidbodies:
                            name = str(rigid_body.rigid_body_name)
                            rigid_body_counts[name] = rigid_body_counts.get(name, 0) + 1
        finally:
            connection.close()
    for topic in rows:
        rows[topic].sort(key=lambda row: row[0])
    return rows, rigid_body_counts


def report_topic(topic, rows, gap_threshold_ms):
    if len(rows) < 2:
        print(f"{topic}: fewer than two messages")
        return

    receive_dt = [(b[0] - a[0]) * 1e-6 for a, b in zip(rows, rows[1:])]
    header_dt = [(b[1] - a[1]) * 1e-6 for a, b in zip(rows, rows[1:])]
    ages = [(receive - header) * 1e-6 for receive, header, _ in rows if header != 0]
    span_sec = (rows[-1][0] - rows[0][0]) * 1e-9
    average_hz = (len(rows) - 1) / span_sec if span_sec > 0.0 else 0.0
    long_indices = [i for i, value in enumerate(receive_dt) if value > gap_threshold_ms]

    print(f"\n{topic}: n={len(rows)}, receive-average={average_hz:.2f}Hz")
    print(
        "  receive dt ms p50/p95/p99/max: "
        f"{percentile(receive_dt, .50):.2f}/"
        f"{percentile(receive_dt, .95):.2f}/"
        f"{percentile(receive_dt, .99):.2f}/{max(receive_dt):.2f}")
    print(
        "  header   dt ms p50/p95/p99/max: "
        f"{percentile(header_dt, .50):.2f}/"
        f"{percentile(header_dt, .95):.2f}/"
        f"{percentile(header_dt, .99):.2f}/{max(header_dt):.2f}")
    print(f"  receive gaps >{gap_threshold_ms:.0f}ms: {len(long_indices)}")
    if ages:
        print(
            "  record-header age ms p50/p95/p99/max: "
            f"{percentile(ages, .50):.2f}/"
            f"{percentile(ages, .95):.2f}/"
            f"{percentile(ages, .99):.2f}/{max(ages):.2f} "
            f"(p99-p50={percentile(ages, .99) - percentile(ages, .50):.2f})")

    if topic == "/rigid_bodies":
        frame_deltas = [b[2] - a[2] for a, b in zip(rows, rows[1:])]
        driver_sequence_gaps = sum(max(0, delta - 1) for delta in frame_deltas)
        batching = sum(
            receive_dt[i] > gap_threshold_ms
            and frame_deltas[i] == 1
            and header_dt[i] < gap_threshold_ms
            for i in long_indices)
        print(f"  driver callback-sequence gaps: {driver_sequence_gaps}")
        print(
            "  long receive gaps with driver sequence +1 and short header dt "
            f"(delivery batching): {batching}/{len(long_indices)}")


def report_pose_coverage(rows):
    raw = rows.get("/rigid_bodies", [])
    if not raw:
        return
    for topic, pose_rows in rows.items():
        if topic == "/rigid_bodies" or not pose_rows:
            continue
        start_ns = pose_rows[0][0]
        end_ns = pose_rows[-1][0]
        raw_overlap = sum(start_ns <= record_ns <= end_ns for record_ns, _, _ in raw)
        if raw_overlap == 0:
            continue
        loss = 100.0 * (1.0 - len(pose_rows) / raw_overlap)
        print(
            f"{topic}: {len(pose_rows)}/{raw_overlap} raw records in receive-time "
            f"overlap ({loss:.2f}% fewer derived poses)")


def report_rigid_body_visibility(rows, rigid_body_counts):
    raw_count = len(rows.get("/rigid_bodies", []))
    if raw_count == 0:
        return
    print("\nRigid-body visibility in recorded callbacks:")
    for name, count in sorted(rigid_body_counts.items()):
        print(f"  body {name}: {count}/{raw_count} ({100.0 * count / raw_count:.2f}%)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="rosbag2 directory or .db3 file")
    parser.add_argument("--gap-ms", type=float, default=60.0)
    args = parser.parse_args()

    databases = find_db3(args.bag)
    if not databases:
        raise SystemExit(f"No .db3 files found under {args.bag}")
    print("databases:", ", ".join(str(path) for path in databases))
    rows, rigid_body_counts = read_rows(databases)
    for topic in TOPIC_TYPES:
        if rows[topic]:
            report_topic(topic, rows[topic], args.gap_ms)
    print("\nDerived-pose coverage relative to raw:")
    report_pose_coverage(rows)
    report_rigid_body_visibility(rows, rigid_body_counts)


if __name__ == "__main__":
    main()
