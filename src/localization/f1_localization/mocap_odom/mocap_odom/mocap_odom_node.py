#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R 

# mocap4r2 메시지
from mocap4r2_msgs.msg import RigidBodies  # /rigid_bodies 토픽


def quat_to_yaw(q: Quaternion) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def unwrap_angle(prev: float, new: float) -> float:
    """prev 주변에서 new(rad)를 언랩(연속화)"""
    d = (new - prev + math.pi) % (2.0 * math.pi) - math.pi
    return prev + d


def _stamp_to_float_seconds(stamp) -> Optional[float]:
    """builtin_interfaces/Time -> float seconds; 0 타임스탬프는 무효 처리."""
    if stamp is None:
        return None
    try:
        # (0,0)도 무효로 간주
        if getattr(stamp, "sec", 0) == 0 and getattr(stamp, "nanosec", 0) == 0:
            return None
        return Time.from_msg(stamp).nanoseconds * 1e-9
    except Exception:
        return None


class MocapOdomNode(Node):
    def __init__(self):
        super().__init__('mocap_odom')

        # Parameters (ROS 2 스타일)
        self.declare_parameter('mocap_topic', '/rigid_bodies')  # mocap4r2_msgs/msg/RigidBodies
        self.declare_parameter('odom_topic', '/mocap_odom')
        self.declare_parameter('frame_id', 'mocap_odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('vel_lowpass_alpha', 0.7)  # 0<alpha<=1 (1: no filter)
        self.declare_parameter('rigid_body_name', '')     # 비워두면 첫 번째 바디 사용

        # 일관된 파라미터 접근 (.value 는 파이썬 타입)
        self.mocap_topic: str = self.get_parameter('mocap_topic').value
        self.odom_topic: str = self.get_parameter('odom_topic').value
        self.frame_id: str = self.get_parameter('frame_id').value
        self.child_frame_id: str = self.get_parameter('child_frame_id').value
        self.publish_tf: bool = bool(self.get_parameter('publish_tf').value)
        self.alpha: float = float(self.get_parameter('vel_lowpass_alpha').value)
        self.rigid_body_name: str = str(self.get_parameter('rigid_body_name').value).strip()

        # alpha 범위 보정
        if not (0.0 < self.alpha <= 1.0):
            self.get_logger().warn(
                f"vel_lowpass_alpha={self.alpha} out of (0,1]; clamping."
            )
            self.alpha = max(1e-6, min(1.0, self.alpha))

        # State
        self.prev_t: Optional[float] = None
        self.prev_x: Optional[float] = None
        self.prev_y: Optional[float] = None
        self.prev_yaw: Optional[float] = None
        self.vx_f: Optional[float] = None
        self.vy_f: Optional[float] = None
        self.yawrate_f: Optional[float] = None
        self._warned_name_missing = False

        # QoS: 센서 데이터용 (Best Effort, 깊이 10)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.x = 0.0
        self.y = 0.0
        posestamp = PoseStamped()
        self.orientation = posestamp.pose.orientation

        # I/O
        self.pose_sub = self.create_subscription(PoseStamped, "/mocap_pose", self.cb_mocap_pose, 5)
        self.sub = self.create_subscription(RigidBodies, self.mocap_topic, self.cb_rigid_bodies, sensor_qos)
        self.pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # TF broadcaster (옵션)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        sel = f"name='{self.rigid_body_name}'" if self.rigid_body_name else "first body"
        self.get_logger().info(
            f"mocap_odom started. Sub: {self.mocap_topic} ({sel}) -> Pub: {self.odom_topic}"
        )

    def lowpass(self, prev_filt: Optional[float], new_val: float) -> float:
        a = self.alpha  # 이미 보정됨
        if math.isfinite(new_val) is False:
            return prev_filt if prev_filt is not None else 0.0
        return new_val if prev_filt is None else (a * new_val + (1.0 - a) * prev_filt)

    def _pick_body(self, msg: RigidBodies):
        bodies = getattr(msg, 'rigidbodies', None)
        if not bodies:
            return None
        if self.rigid_body_name:
            for b in bodies:
                if getattr(b, 'name', '') == self.rigid_body_name:
                    return b
            if not self._warned_name_missing:
                self._warned_name_missing = True
                self.get_logger().warn(
                    f"rigid_body_name='{self.rigid_body_name}' not found. Falling back to first body."
                )
        return bodies[0]

    def _get_msg_time(self, msg: RigidBodies, body) -> float:
        """
        바디 개별 헤더가 있으면 그걸 우선 사용.
        없으면 메시지 헤더, 그것도 없으면 now().
        """
        t = None
        # 바디 헤더(stamp) 우선
        body_header = getattr(body, 'header', None)
        if body_header:
            t = _stamp_to_float_seconds(getattr(body_header, 'stamp', None))
        # 메시지 헤더(stamp)
        if t is None:
            header = getattr(msg, 'header', None)
            if header:
                t = _stamp_to_float_seconds(getattr(header, 'stamp', None))
        # 최종 fallback: now()
        if t is None:
            t = self.get_clock().now().nanoseconds * 1e-9
        return t

    def _make_stamp_msg(self, t_sec: float):
        # float seconds -> builtin_interfaces/Time msg (근사)
        # rclpy의 Clock.now().to_msg()와 달리 외부 시간 그대로 쓰고 싶을 때 사용
        ns = int(round(t_sec * 1e9))
        from builtin_interfaces.msg import Time as TimeMsg
        tm = TimeMsg()
        tm.sec = ns // 1_000_000_000
        tm.nanosec = ns % 1_000_000_000
        return tm

    def _publish_odom_and_tf(self, x, y, z, yaw, stamp_msg):
        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        # odom.pose.pose.orientation = yaw_to_quat(yaw)
        # odom.pose.pose.orientation = R.from_euler('zyx', [yaw, 0, 0]).as_quat()
        ori = R.from_euler('zyx', [yaw, 0, 0]).as_quat()
        odom.pose.pose.orientation.x = ori[0]
        odom.pose.pose.orientation.y = ori[1]
        odom.pose.pose.orientation.z = ori[2]
        odom.pose.pose.orientation.w = ori[3]

        self.pub.publish(odom)

        if self.tf_br is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp_msg
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_br.sendTransform(tf_msg)
            
    def cb_mocap_pose(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.orientation = msg.pose.orientation

    def cb_rigid_bodies(self, msg: RigidBodies):
        body = self._pick_body(msg)
        if body is None:
            self.get_logger().warn("No rigid bodies in message.")
            return

        pose = body.pose  # geometry_msgs/Pose

        # Pose in world
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        # yaw_meas = quat_to_yaw(pose.orientation)
        yaw_meas = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_euler('zyx', degrees=False)[0]  # scipy 사용

        # 타임스탬프 결정(바디 헤더 > 메시지 헤더 > now)
        t = self._get_msg_time(msg, body)
        stamp_ros = self._make_stamp_msg(t)

        if self.prev_t is None:
            # 첫 프레임: 포즈만 게시(속도 0)
            # self._publish_odom_and_tf(x, y, z, yaw_meas, stamp_ros)
            self.prev_t, self.prev_x, self.prev_y, self.prev_yaw = t, x, y, yaw_meas
            self.vx_f = self.vy_f = self.yawrate_f = 0.0
            return

        dt = t - self.prev_t
        if not (dt > 0.0 and math.isfinite(dt)):
            # 비정상 시간(역행/정지/NaN) 무시
            return

        # Unwrap
        yaw_unwrapped = unwrap_angle(self.prev_yaw, yaw_meas)

        # World-frame velocities
        vx_w = (x - self.prev_x) / dt
        vy_w = (y - self.prev_y) / dt
        yaw_rate = (yaw_unwrapped - self.prev_yaw) / dt

        # Low-pass
        vx_w = self.lowpass(self.vx_f, vx_w)
        vy_w = self.lowpass(self.vy_f, vy_w)
        yaw_rate = self.lowpass(self.yawrate_f, yaw_rate)
        self.vx_f, self.vy_f, self.yawrate_f = vx_w, vy_w, yaw_rate

        # World -> Body (현재 yaw 기준)
        cy, sy = math.cos(yaw_unwrapped), math.sin(yaw_unwrapped)
        v_long =  cy * vx_w + sy * vy_w    # base_link x (forward)
        v_lat  = -sy * vx_w + cy * vy_w    # base_link y (left)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = stamp_ros
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # odom.pose.pose.position.x = x
        # odom.pose.pose.position.y = y
        # odom.pose.pose.position.z = z
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = z
        # ori = R.from_euler('zyx', [yaw_unwrapped, 0, 0]).as_quat()
        # odom.pose.pose.orientation.x = ori[0]
        # odom.pose.pose.orientation.y = ori[1]
        # odom.pose.pose.orientation.z = ori[2]
        # odom.pose.pose.orientation.w = ori[3]
        odom.pose.pose.orientation = self.orientation

        odom.twist.twist.linear.x = -v_lat
        odom.twist.twist.linear.y = v_long
        odom.twist.twist.linear.z = math.sqrt(vx_w*vx_w + vy_w*vy_w)  # 전체 속도 크기
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = yaw_rate

        self.pub.publish(odom)
        print("callback rigid body")

        # Optional TF (odom -> base_link)
        if self.tf_br is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp_ros
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_br.sendTransform(tf_msg)

        # Update prev
        self.prev_t, self.prev_x, self.prev_y, self.prev_yaw = t, x, y, yaw_unwrapped


def main(args=None):
    rclpy.init(args=args)
    node = MocapOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
