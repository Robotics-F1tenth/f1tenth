#!/usr/bin/env python3
import os
import math
import shutil
import sys
import threading
import termios
import tty
import select
from typing import Optional

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory


# ---------------- YAML pretty dump helpers ----------------
class FlowList(list):
    pass


class MyDumper(yaml.SafeDumper):
    pass


def _flow_list_representer(dumper, data):
    return dumper.represent_sequence(
        'tag:yaml.org,2002:seq',
        list(data),
        flow_style=True
    )


MyDumper.add_representer(FlowList, _flow_list_representer)


class QuotedStr(str):
    pass


def _quoted_str_representer(dumper, data: QuotedStr):
    return dumper.represent_scalar('tag:yaml.org,2002:str', str(data), style='"')


MyDumper.add_representer(QuotedStr, _quoted_str_representer)


def keep_signed_zero(x: float, eps: float = 1e-12) -> float:
    if abs(x) < eps:
        return -0.0 if math.copysign(1.0, x) < 0 else 0.0
    return float(x)


def as_flow_list(arr) -> FlowList:
    return FlowList([keep_signed_zero(float(x)) for x in arr])


def ensure_quoted_string_fields(p: dict) -> None:
    for k in ("input_topic", "output_frame", "output_topic", "frame_id", "child_frame_id"):
        if k in p and isinstance(p[k], str):
            p[k] = QuotedStr(p[k])


def write_all_params_yaml(
    params_path: str,
    R_map_mocap: np.ndarray,
    t_map_mocap: np.ndarray,
    R_point_base: np.ndarray,
    t_point_base: np.ndarray,
    logger=None
) -> None:
    if not os.path.exists(params_path):
        raise FileNotFoundError(f"params file not found: {params_path}")

    with open(params_path, "r") as f:
        data = yaml.safe_load(f)

    if "/**" not in data or "ros__parameters" not in data["/**"]:
        raise KeyError(f"Invalid params yaml structure: {params_path}")

    p = data["/**"]["ros__parameters"]
    ensure_quoted_string_fields(p)

    p["rotation"] = as_flow_list(np.asarray(R_map_mocap, dtype=np.float64).reshape(-1))
    p["translation"] = as_flow_list(np.asarray(t_map_mocap, dtype=np.float64).reshape(-1))
    p["tracked_point_to_base_rotation"] = as_flow_list(
        np.asarray(R_point_base, dtype=np.float64).reshape(-1)
    )
    p["tracked_point_to_base_translation"] = as_flow_list(
        np.asarray(t_point_base, dtype=np.float64).reshape(-1)
    )

    with open(params_path, "w") as f:
        yaml.dump(
            data,
            f,
            Dumper=MyDumper,
            sort_keys=False,
            default_flow_style=False,
            width=120,
            indent=2
        )

    if logger:
        logger.info(f"Updated params in: {params_path}")


def reset_yaml_identity(params_path: str, logger=None) -> None:
    R_map_mocap = np.eye(3, dtype=np.float64)
    t_map_mocap = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    R_point_base = np.eye(3, dtype=np.float64)
    t_point_base = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    write_all_params_yaml(
        params_path,
        R_map_mocap,
        t_map_mocap,
        R_point_base,
        t_point_base,
        logger=logger
    )
# ----------------------------------------------------------


def quat_to_yaw(qx, qy, qz, qw) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rot2d(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=np.float64)


def circular_mean(angles_rad: np.ndarray) -> float:
    s = np.sin(angles_rad).mean()
    c = np.cos(angles_rad).mean()
    return math.atan2(s, c)


def fmt_matrix3(R: np.ndarray) -> str:
    r = np.asarray(R, dtype=np.float64).reshape(-1).tolist()
    return ("[{: .6f}, {: .6f}, {: .6f},\n"
            "           {: .6f}, {: .6f}, {: .6f},\n"
            "           {: .6f}, {: .6f}, {: .6f}]").format(*r)


def fmt_vec3(t: np.ndarray) -> str:
    t = np.asarray(t, dtype=np.float64).reshape(3).tolist()
    return "[{: .6f}, {: .6f}, {: .6f}]".format(*t)


def pose_compose(x1, y1, yaw1, x2, y2, yaw2):
    c, s = math.cos(yaw1), math.sin(yaw1)
    x = x1 + c * x2 - s * y2
    y = y1 + s * x2 + c * y2
    yaw = wrap_pi(yaw1 + yaw2)
    return x, y, yaw


def rmse(vals):
    vals = np.asarray(vals, dtype=np.float64)
    return float(np.sqrt(np.mean(vals * vals))) if len(vals) > 0 else 0.0


# ---------------- Keyboard listener (space / q) ----------------
class KeyboardListener:
    """
    Listen for calibration keyboard commands.

    Space captures one sample, and Q finishes the session.
    """

    def __init__(self, on_space, on_quit, logger=None):
        self.on_space = on_space
        self.on_quit = on_quit
        self.logger = logger
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._old_term = None

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._restore_terminal()

    def _set_raw(self):
        fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(fd)
        tty.setcbreak(fd)

    def _restore_terminal(self):
        if self._old_term is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old_term)
            except Exception:
                pass
            self._old_term = None

    def _run(self):
        try:
            self._set_raw()
            if self.logger:
                self.logger.info("Keyboard: press SPACE to save a sample, press 'q' to finish.")
            while not self._stop.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self.on_space()
                elif ch in ('q', 'Q'):
                    self.on_quit()
                    break
        finally:
            self._restore_terminal()
# --------------------------------------------------------------


class SE2OffsetEstimator(Node):
    def __init__(self):
        super().__init__('se2_offset_estimator')

        self.declare_parameter('obj', 1)
        self.obj = int(self.get_parameter('obj').value)

        if self.obj not in (1, 2):
            self.get_logger().warn(
                f"obj={self.obj} is invalid. Fallback to obj=1 params."
            )
            self.obj = 1

        params_name = 'params.yaml' if self.obj == 1 else 'params2.yaml'
        try:
            package_share = get_package_share_directory('mocap_localization')
            default_params_path = os.path.join(package_share, 'params', params_name)
        except Exception:
            default_params_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), 'params', params_name)

        self.declare_parameter('params_path', default_params_path)
        self.declare_parameter('candidate_path', '')
        self.declare_parameter('overwrite_source_params', False)
        self.declare_parameter('overwrite_candidate', False)
        self.declare_parameter('prepare_identity_candidate', True)

        self.params_path = os.path.abspath(
            os.path.expanduser(str(self.get_parameter('params_path').value)))
        candidate_path = str(self.get_parameter('candidate_path').value).strip()
        overwrite_source = bool(self.get_parameter('overwrite_source_params').value)
        overwrite_candidate = bool(self.get_parameter('overwrite_candidate').value)
        prepare_identity = bool(self.get_parameter('prepare_identity_candidate').value)

        if not os.path.exists(self.params_path):
            raise FileNotFoundError(f"params file not found: {self.params_path}")

        if overwrite_source:
            self.target_params_path = self.params_path
            self.get_logger().warn(
                "overwrite_source_params=true: the source params file may be modified")
        else:
            if not candidate_path:
                candidate_path = os.path.abspath(
                    f'mocap_calibration_obj{self.obj}_candidate.yaml')
            self.target_params_path = os.path.abspath(os.path.expanduser(candidate_path))
            if self.target_params_path == self.params_path:
                raise ValueError(
                    "candidate_path equals params_path; choose another path or explicitly set "
                    "overwrite_source_params:=true")
            if os.path.exists(self.target_params_path) and not overwrite_candidate:
                raise FileExistsError(
                    f"candidate already exists: {self.target_params_path}; choose a new path "
                    "or explicitly set overwrite_candidate:=true")
            shutil.copy2(self.params_path, self.target_params_path)

        self.get_logger().info(f"Source params: {self.params_path}")
        self.get_logger().info(f"Candidate output: {self.target_params_path}")

        self.mocap_topic = '/mocap_pose1' if self.obj == 1 else '/mocap_pose2'

        self.declare_parameter('mcl_topic', '/newmcl_pose')
        self.declare_parameter('time_tolerance_sec', 0.05)

        self.mcl_topic = self.get_parameter('mcl_topic').value
        self.tol = float(self.get_parameter('time_tolerance_sec').value)

        self.latest_mcl: Optional[PoseStamped] = None
        self.latest_mocap: Optional[PoseStamped] = None

        # sample: (x_mcl, y_mcl, yaw_mcl, x_moc, y_moc, yaw_moc)
        self.samples = []

        self.pending_sample = None
        self.pending_lock = threading.Lock()

        self.finish_requested = threading.Event()
        self.finish_timer = None

        if prepare_identity:
            reset_yaml_identity(self.target_params_path, logger=self.get_logger())
            self.get_logger().info(
                f"Prepared identity candidate (source untouched): {self.target_params_path}")

        self.get_logger().info(
            "Launch a separate mocap_localization instance with the candidate file before "
            "capturing samples. Do not use the production output topic at the same time."
        )
        self.get_logger().warn(
            "This legacy six-parameter fit cannot uniquely separate shared T_map_mocap "
            "from vehicle-specific T_point_base. Treat its output as a candidate only; "
            "both vehicles must ultimately share one validated T_map_mocap.")

        self.sub_mcl = self.create_subscription(PoseStamped, self.mcl_topic, self.cb_mcl, 10)
        self.sub_mocap = self.create_subscription(PoseStamped, self.mocap_topic, self.cb_mocap, 10)

        self.kb = KeyboardListener(self.on_space, self.on_quit, logger=self.get_logger())
        self.kb.start()

        self.get_logger().info(
            f"obj={self.obj}\n"
            f"mcl_topic={self.mcl_topic}\n"
            f"mocap_topic={self.mocap_topic}\n"
            f"tol={self.tol}s\n"
            f"Press SPACE to save sample, 'q' to finish."
        )

    def destroy_node(self):
        try:
            if hasattr(self, "kb") and self.kb is not None:
                self.kb.stop()
        except Exception:
            pass
        super().destroy_node()

    def stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_space(self):
        with self.pending_lock:
            if self.pending_sample is None:
                self.get_logger().warn(
                    "No pending paired sample within tol yet. "
                    "Move robot / wait for synced messages."
                )
                return

            self.samples.append(self.pending_sample)
            n = len(self.samples)

            x_mcl, y_mcl, yaw_mcl, x_moc, y_moc, yaw_moc = self.pending_sample
            self.get_logger().info(
                f"[CAPTURE] saved sample #{n}: "
                f"mcl=({x_mcl:.3f}, {y_mcl:.3f}, {math.degrees(yaw_mcl):.1f}deg), "
                f"moc=({x_moc:.3f}, {y_moc:.3f}, {math.degrees(yaw_moc):.1f}deg)"
            )

    def on_quit(self):
        self.get_logger().info("Quit requested (q). Finishing and saving results...")
        self.finish_requested.set()
        if self.finish_timer is None:
            self.finish_timer = self.create_timer(0.0, self._finish_once)

    def _finish_once(self):
        if not self.finish_requested.is_set():
            return
        self.finish_requested.clear()

        if len(self.samples) < 3:
            self.get_logger().warn("Need at least 3 samples. Nothing to save.")
        else:
            self.report_and_save()

        rclpy.shutdown()

    def cb_mcl(self, msg: PoseStamped):
        self.latest_mcl = msg
        self.try_pair_update_pending()

    def cb_mocap(self, msg: PoseStamped):
        self.latest_mocap = msg
        self.try_pair_update_pending()

    def try_pair_update_pending(self):
        if self.latest_mcl is None or self.latest_mocap is None:
            return

        t_mcl = self.stamp_to_sec(self.latest_mcl.header.stamp)
        t_mocap = self.stamp_to_sec(self.latest_mocap.header.stamp)
        if abs(t_mcl - t_mocap) > self.tol:
            return

        pm = self.latest_mcl.pose.position
        om = self.latest_mcl.pose.orientation
        x_mcl, y_mcl = float(pm.x), float(pm.y)
        yaw_mcl = quat_to_yaw(om.x, om.y, om.z, om.w)

        pc = self.latest_mocap.pose.position
        oc = self.latest_mocap.pose.orientation
        x_moc, y_moc = float(pc.x), float(pc.y)
        yaw_moc = quat_to_yaw(oc.x, oc.y, oc.z, oc.w)

        with self.pending_lock:
            self.pending_sample = (x_mcl, y_mcl, yaw_mcl, x_moc, y_moc, yaw_moc)

    def predict_base_from_mocap_point(self, params, x_moc, y_moc, yaw_moc):
        """
        Predict the base pose from the tracked mocap point.

        Params are ``[a, tx, ty, b, bx, by]``. The first triplet describes
        ``T_map_mocap`` and the second describes ``T_point_base``.
        """
        a, tx, ty, b, bx, by = params

        x1, y1, yaw1 = pose_compose(tx, ty, a, x_moc, y_moc, yaw_moc)
        x2, y2, yaw2 = pose_compose(x1, y1, yaw1, bx, by, b)
        return x2, y2, yaw2

    def residual_vector(self, params):
        res = []
        yaw_weight = 1.0

        for s in self.samples:
            x_mcl, y_mcl, yaw_mcl, x_moc, y_moc, yaw_moc = s
            x_pred, y_pred, yaw_pred = self.predict_base_from_mocap_point(
                params, x_moc, y_moc, yaw_moc
            )
            res.append(x_pred - x_mcl)
            res.append(y_pred - y_mcl)
            res.append(yaw_weight * wrap_pi(yaw_pred - yaw_mcl))

        return np.array(res, dtype=np.float64)

    def numerical_jacobian(self, params, eps=1e-6):
        r0 = self.residual_vector(params)
        J = np.zeros((len(r0), len(params)), dtype=np.float64)

        for k in range(len(params)):
            p1 = params.copy()
            p1[k] += eps
            r1 = self.residual_vector(p1)
            J[:, k] = (r1 - r0) / eps

        return J

    def estimate_map_and_point_to_base(self):
        if len(self.samples) < 3:
            raise RuntimeError("Need at least 3 samples")

        yaw_diffs = np.array([wrap_pi(s[2] - s[5]) for s in self.samples], dtype=np.float64)
        gamma = float(circular_mean(yaw_diffs))  # roughly a + b

        a0 = gamma
        b0 = 0.0

        xy0 = []
        for s in self.samples:
            x_mcl, y_mcl, _, x_moc, y_moc, _ = s
            p_mcl = np.array([x_mcl, y_mcl], dtype=np.float64)
            p_moc = np.array([x_moc, y_moc], dtype=np.float64)
            xy0.append(p_mcl - rot2d(a0) @ p_moc)

        xy0 = np.array(xy0, dtype=np.float64)
        tx0 = float(np.mean(xy0[:, 0]))
        ty0 = float(np.mean(xy0[:, 1]))
        bx0 = 0.0
        by0 = 0.0

        params = np.array([a0, tx0, ty0, b0, bx0, by0], dtype=np.float64)

        damping = 1e-3
        last_cost = None

        for _ in range(60):
            r = self.residual_vector(params)
            cost = 0.5 * float(r @ r)

            J = self.numerical_jacobian(params)
            H = J.T @ J + damping * np.eye(len(params), dtype=np.float64)
            g = J.T @ r

            try:
                dp = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                dp = -np.linalg.lstsq(H, g, rcond=None)[0]

            cand = params + dp
            cand[0] = wrap_pi(cand[0])
            cand[3] = wrap_pi(cand[3])

            r_new = self.residual_vector(cand)
            cost_new = 0.5 * float(r_new @ r_new)

            if last_cost is None or cost_new < cost:
                params = cand
                damping = max(1e-6, damping * 0.7)
                last_cost = cost_new
                if np.linalg.norm(dp) < 1e-8:
                    break
            else:
                damping = min(1e3, damping * 2.0)

        return params

    def report_and_save(self):
        params = self.estimate_map_and_point_to_base()
        a, tx, ty, b, bx, by = params

        ca, sa = math.cos(a), math.sin(a)
        R_map_mocap = np.array([
            [ca, -sa, 0.0],
            [sa,  ca, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        t_map_mocap = np.array([tx, ty, 0.0], dtype=np.float64)

        cb, sb = math.cos(b), math.sin(b)
        R_point_base = np.array([
            [cb, -sb, 0.0],
            [sb,  cb, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        t_point_base = np.array([bx, by, 0.0], dtype=np.float64)

        ex_list, ey_list, eyaw_list = [], [], []
        for s in self.samples:
            x_mcl, y_mcl, yaw_mcl, x_moc, y_moc, yaw_moc = s
            x_pred, y_pred, yaw_pred = self.predict_base_from_mocap_point(
                params, x_moc, y_moc, yaw_moc
            )
            ex_list.append(x_pred - x_mcl)
            ey_list.append(y_pred - y_mcl)
            eyaw_list.append(wrap_pi(yaw_pred - yaw_mcl))

        self.get_logger().info("==== Joint calibration result ====")
        self.get_logger().info(f"samples = {len(self.samples)}")

        self.get_logger().info("---- T_map_mocap ----")
        self.get_logger().info(f"yaw_map_mocap [deg] = {math.degrees(a):.3f}")
        self.get_logger().info(f"translation_map_mocap [m] = [{tx:.4f}, {ty:.4f}, 0.0]")
        self.get_logger().info("rotation_map_mocap: " + fmt_matrix3(R_map_mocap))
        self.get_logger().info("translation_map_mocap: " + fmt_vec3(t_map_mocap))

        self.get_logger().info("---- T_point_base (tracked point -> base_link) ----")
        self.get_logger().info(f"yaw_point_base [deg] = {math.degrees(b):.3f}")
        self.get_logger().info(f"translation_point_base [m] = [{bx:.4f}, {by:.4f}, 0.0]")
        self.get_logger().info("rotation_point_base: " + fmt_matrix3(R_point_base))
        self.get_logger().info("translation_point_base: " + fmt_vec3(t_point_base))

        self.get_logger().info("---- fit error ----")
        self.get_logger().info(f"rmse_x [m] = {rmse(ex_list):.4f}")
        self.get_logger().info(f"rmse_y [m] = {rmse(ey_list):.4f}")
        self.get_logger().info(f"rmse_yaw [deg] = {math.degrees(rmse(eyaw_list)):.3f}")

        try:
            write_all_params_yaml(
                self.target_params_path,
                R_map_mocap,
                t_map_mocap,
                R_point_base,
                t_point_base,
                logger=self.get_logger()
            )
            self.get_logger().info(
                f"Saved T_map_mocap and T_point_base to: {self.target_params_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to write params yaml: {e}")


def main():
    rclpy.init()
    node = SE2OffsetEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
