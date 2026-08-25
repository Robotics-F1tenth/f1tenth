#!/usr/bin/env python3
"""Does the TF broadcaster's queue depth decide how stale a slow subscriber's data gets?

This is the mechanism behind the 0824 competition bags, where /tf arrived a
median 2494 ms after its own header stamp while every other topic stayed under
25 ms. 2494 ms is 100 samples at 40 Hz, and 100 is tf2's default broadcaster
queue depth -- so the reading was that one stalled subscriber filled a RELIABLE
depth-100 queue and every subscriber then got 2.5 s old transforms.

Publishes at 40 Hz into a deliberately slow subscriber and reports how far
behind its own header stamp each message arrives, at several depths. If the
reading is right, worst-case staleness tracks depth/rate.

    python3 tf_queue_staleness_test.py            # depths 100, 10
    python3 tf_queue_staleness_test.py 100 50 10 5
"""
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster


def tf_qos(depth):
    """tf2's DynamicBroadcasterQoS in Python: RELIABLE, KEEP_LAST, VOLATILE."""
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.VOLATILE,
                      depth=depth)

RATE = 40.0
SECONDS = 4.0
SUBSCRIBER_STALL = 0.05   # 50 ms per callback, i.e. half the publish rate


class Slow(Node):
    """A subscriber that cannot keep up, which is what fills the queue."""

    def __init__(self, depth):
        super().__init__("slow_tf_sub")
        self.lags = []
        self.create_subscription(
            TFMessage, "/tf", self.cb, tf_qos(depth))

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        for tr in msg.transforms:
            st = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            self.lags.append(now - st)
        time.sleep(SUBSCRIBER_STALL)


def run(depth):
    rclpy.init()
    pub_node = Node("tf_pub_test")
    br = TransformBroadcaster(pub_node, qos=tf_qos(depth))
    sub = Slow(depth)

    import threading
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(sub)
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            ex.spin_once(timeout_sec=0.05)

    t = threading.Thread(target=spin, daemon=True)
    t.start()

    time.sleep(0.5)
    n = int(RATE * SECONDS)
    for i in range(n):
        tr = TransformStamped()
        tr.header.stamp = pub_node.get_clock().now().to_msg()
        tr.header.frame_id = "map"
        tr.child_frame_id = "base_link"
        tr.transform.rotation.w = 1.0
        br.sendTransform(tr)
        time.sleep(1.0 / RATE)
    time.sleep(1.0)

    lags = sub.lags[:]
    stop.set()
    t.join(timeout=2.0)
    ex.remove_node(sub)
    sub.destroy_node()
    pub_node.destroy_node()
    rclpy.shutdown()
    return n, lags


def main():
    depths = [int(a) for a in sys.argv[1:]] or [100, 10]
    print(f"  발행 {RATE:.0f} Hz x {SECONDS:.0f}s, 구독자는 콜백마다 "
          f"{SUBSCRIBER_STALL*1000:.0f}ms 지연 (따라갈 수 없음)\n")
    print(f"  {'depth':>6} {'수신':>6} {'지연 p50':>9} {'지연 max':>9}  {'이론 상한 depth/rate':>18}")
    for d in depths:
        n, lags = run(d)
        if not lags:
            print(f"  {d:>6} {0:>6}   (수신 없음)")
            continue
        lags.sort()
        p50 = lags[len(lags) // 2] * 1000
        mx = lags[-1] * 1000
        print(f"  {d:>6} {len(lags):>6} {p50:8.0f}ms {mx:8.0f}ms  {d/RATE*1000:16.0f}ms")


if __name__ == "__main__":
    main()
