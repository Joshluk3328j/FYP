#!/usr/bin/env python3
"""
Navigation Benchmark Script – Live Robot (enhanced)

Adds target-level measurements that can be collected directly from a Nav2 run:
- trial and target identifiers (one trial contains all five targets)
- planned path length
- actual travelled distance
- detected replanning events (global-plan changes)
- time between detected replanning events
- recovery-event count from NavigateToPose feedback
- planner/controller CPU mean + peak
- planner/controller RAM mean + peak

Important:
- A standard Nav2 NavigateToPose action does NOT expose "nodes expanded".
  That metric must be instrumented inside the planner implementation if required.
- A true replanning *latency* needs a known replanning trigger time (e.g. the
  exact time a dynamic obstacle blocks the path). This script records plan
  change times and inter-replan intervals, but does not falsely label them as
  replanning latency.
"""

import argparse
import csv
import math
import os
import sys
import time
import threading
from datetime import datetime

import psutil
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros


# ── Edit these to match your map ──────────────────────────────────────────────

# GOALS = [
#     # (x, y, orientation_z, orientation_w)
#     ( 0.53,  -0.51,  -0.71,  0.71),
#     # ( 0.00,  -0.00,  -0.71,  0.71),
#     ( 0.37,  -1.18,  -0.71,  0.71),
#     ( 0.12,  -2.57,  -0.71,  0.71),
#     ( 2.05,  -0.41,   0.00,  1.00),
#     ( 0.56,   0.23,   0.00,  1.00),
# ]

GOALS = [
    # (x, y, orientation_z, orientation_w)
    ( 1.00,  -0.12,  -0.71,  0.71),
    ( 1.25,  -1.04,  -0.71,  0.71),
    ( 1.41,  -2.61,   0.00,  1.00),
    ( 1.80,  -0.20,   0.00,  1.00),
    ( 2.12,   0.45,   1.00,  0.00),
    ( 0.29,   0.08,   0.00,  1.00),
]

PLANNER_PROC = "planner_server"
CONTROLLER_PROC = "controller_server"
CPU_SAMPLE_HZ = 2
POSE_SAMPLE_HZ = 10
PLAN_TOPIC = "/plan"


# ── Process monitor ───────────────────────────────────────────────────────────

class CPUMonitor:
    """Samples CPU and RSS memory of named ROS2 processes."""

    def __init__(self):
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._planner_cpu = []
        self._controller_cpu = []
        self._planner_ram = []
        self._controller_ram = []

    def _find_proc(self, name_fragment: str):
        for p in psutil.process_iter(["pid", "cmdline", "cpu_percent", "memory_info"]):
            try:
                cmd = " ".join(p.info["cmdline"] or [])
                if name_fragment in cmd:
                    return p
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return None

    def _sample_loop(self):
        p_proc = self._find_proc(PLANNER_PROC)
        c_proc = self._find_proc(CONTROLLER_PROC)

        if p_proc:
            try:
                p_proc.cpu_percent()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        if c_proc:
            try:
                c_proc.cpu_percent()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        time.sleep(1.0 / CPU_SAMPLE_HZ)

        while self._running:
            p_proc = self._find_proc(PLANNER_PROC)
            c_proc = self._find_proc(CONTROLLER_PROC)

            with self._lock:
                if p_proc:
                    try:
                        self._planner_cpu.append(p_proc.cpu_percent())
                        self._planner_ram.append(
                            p_proc.memory_info().rss / (1024 * 1024)
                        )
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass

                if c_proc:
                    try:
                        self._controller_cpu.append(c_proc.cpu_percent())
                        self._controller_ram.append(
                            c_proc.memory_info().rss / (1024 * 1024)
                        )
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        pass

            time.sleep(1.0 / CPU_SAMPLE_HZ)

    def start(self):
        self._planner_cpu = []
        self._controller_cpu = []
        self._planner_ram = []
        self._controller_ram = []
        self._running = True
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    @staticmethod
    def _stats(values):
        if not values:
            return None, None
        return round(sum(values) / len(values), 2), round(max(values), 2)

    def results(self):
        with self._lock:
            p_cpu = list(self._planner_cpu)
            c_cpu = list(self._controller_cpu)
            p_ram = list(self._planner_ram)
            c_ram = list(self._controller_ram)

        p_cpu_avg, p_cpu_peak = self._stats(p_cpu)
        c_cpu_avg, c_cpu_peak = self._stats(c_cpu)
        p_ram_avg, p_ram_peak = self._stats(p_ram)
        c_ram_avg, c_ram_peak = self._stats(c_ram)

        return (
            p_cpu_avg, p_cpu_peak,
            c_cpu_avg, c_cpu_peak,
            p_ram_avg, p_ram_peak,
            c_ram_avg, c_ram_peak,
        )


# ── Helpers ───────────────────────────────────────────────────────────────────

def get_robot_pose(tf_buffer: tf2_ros.Buffer):
    try:
        t = tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        return t.transform.translation.x, t.transform.translation.y
    except Exception as e:
        print(f"[benchmark] TF lookup failed: {e}")
        return None, None


def make_goal_pose(x: float, y: float, oz: float = 0.0, ow: float = 1.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = oz
    pose.pose.orientation.w = ow
    return pose


def euclidean(ax, ay, bx, by):
    if None in (ax, ay, bx, by):
        return None
    return round(math.hypot(bx - ax, by - ay), 4)


def path_length(path_msg: Path):
    """Calculate the geometric length of a nav_msgs/Path."""
    if len(path_msg.poses) < 2:
        return 0.0

    total = 0.0
    for a, b in zip(path_msg.poses[:-1], path_msg.poses[1:]):
        total += math.hypot(
            b.pose.position.x - a.pose.position.x,
            b.pose.position.y - a.pose.position.y,
        )
    return round(total, 4)


def path_signature(path_msg: Path):
    """Compact signature so repeated publication of the same path is not a replan."""
    if not path_msg.poses:
        return None

    pts = tuple(
        (
            round(p.pose.position.x, 3),
            round(p.pose.position.y, 3),
        )
        for p in path_msg.poses
    )
    return pts


# ── Main benchmark node ───────────────────────────────────────────────────────

class BenchmarkNode(Node):

    def __init__(self, planner: str, output_dir: str, trial_number: int):
        super().__init__("nav_benchmark")
        self.planner = planner
        self.output_dir = output_dir
        self.trial_number = trial_number

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cpu = CPUMonitor()
        self._results = []

        # Per-target path/replanning state.
        self._latest_plan_length = None
        self._last_plan_signature = None
        self._plan_count = 0
        self._replan_times = []
        self._first_plan_time = None
        self._trial_start_time = None

        # Actual travelled distance from TF.
        self._last_robot_xy = None
        self._travelled_distance = 0.0
        self._pose_sampling = False
        self._pose_thread = None

        # NavigateToPose feedback recovery count.
        self._recovery_count = 0

        self._plan_sub = self.create_subscription(
            Path,
            PLAN_TOPIC,
            self._plan_callback,
            10,
        )

    def _reset_trial_state(self):
        self._latest_plan_length = None
        self._last_plan_signature = None
        self._plan_count = 0
        self._replan_times = []
        self._first_plan_time = None
        self._trial_start_time = None
        self._last_robot_xy = None
        self._travelled_distance = 0.0
        self._recovery_count = 0

    def _plan_callback(self, msg: Path):
        if not self._pose_sampling or self._trial_start_time is None:
            return

        now = time.time()
        signature = path_signature(msg)
        self._latest_plan_length = path_length(msg)
        self._plan_count += 1

        if self._first_plan_time is None and signature is not None:
            self._first_plan_time = now

        # A changed global plan after the first plan is treated as a detected
        # replanning event. This is intentionally not called replanning latency.
        if (
            signature is not None
            and self._last_plan_signature is not None
            and signature != self._last_plan_signature
        ):
            self._replan_times.append(now)

        if signature is not None:
            self._last_plan_signature = signature

    def _pose_sample_loop(self):
        while self._pose_sampling:
            x, y = get_robot_pose(self._tf_buffer)
            if x is not None and y is not None:
                if self._last_robot_xy is not None:
                    self._travelled_distance += math.hypot(
                        x - self._last_robot_xy[0],
                        y - self._last_robot_xy[1],
                    )
                self._last_robot_xy = (x, y)
            time.sleep(1.0 / POSE_SAMPLE_HZ)

    def _start_pose_sampling(self):
        self._pose_sampling = True
        self._pose_thread = threading.Thread(
            target=self._pose_sample_loop,
            daemon=True,
        )
        self._pose_thread.start()

    def _stop_pose_sampling(self):
        self._pose_sampling = False
        if self._pose_thread:
            self._pose_thread.join(timeout=2.0)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Nav2 exposes cumulative recovery count in NavigateToPose feedback.
        if hasattr(feedback, "number_of_recoveries"):
            self._recovery_count = feedback.number_of_recoveries

    def send_goal(self, goal_index: int, trial_number: int,
                  x: float, y: float, oz: float = 0.0, ow: float = 1.0):
        self.get_logger().info(
            f"Trial={trial_number} | "
            f"Target {goal_index + 1}/{len(GOALS)}: ({x}, {y})"
        )

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            return

        self._reset_trial_state()
        self._trial_start_time = time.time()
        self._pose_sampling = True
        self._start_pose_sampling()
        self._cpu.start()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_goal_pose(x, y, oz, ow)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        send_time = time.time()
        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            self._cpu.stop()
            self._stop_pose_sampling()
            self._record(
                goal_index, trial_number, x, y, oz, ow,
                None, None, None, None,
                "REJECTED", None, None, None, None,
                None, None, None, None,
                0, None, None, None, None,
                self._recovery_count,
            )
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        total_time = time.time() - send_time

        self._cpu.stop()
        self._stop_pose_sampling()

        (
            p_avg, p_peak,
            c_avg, c_peak,
            p_ram_avg, p_ram_peak,
            c_ram_avg, c_ram_peak,
        ) = self._cpu.results()

        status = result_future.result().status
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(status, f"STATUS_{status}")

        final_x, final_y = get_robot_pose(self._tf_buffer)
        error = euclidean(x, y, final_x, final_y)

        initial_plan_time = None
        if self._first_plan_time is not None:
            initial_plan_time = round(self._first_plan_time - send_time, 4)

        replan_count = len(self._replan_times)
        replan_intervals = []
        for a, b in zip(self._replan_times[:-1], self._replan_times[1:]):
            replan_intervals.append(b - a)

        mean_replan_interval = (
            round(sum(replan_intervals) / len(replan_intervals), 4)
            if replan_intervals else None
        )
        worst_replan_interval = (
            round(max(replan_intervals), 4)
            if replan_intervals else None
        )

        replan_frequency = (
            round(replan_count / total_time, 4)
            if total_time > 0 else None
        )

        navigation_interrupted = replan_count > 0
        recovery_success = (
            True if self._recovery_count > 0 and status_str == "SUCCEEDED" else
            False if self._recovery_count > 0 else None
        )

        self.get_logger().info(
            f"  → {status_str} | initial_plan≈{initial_plan_time}s | "
            f"total={total_time:.2f}s | actual_dist={self._travelled_distance:.2f}m | "
            f"planned_dist={self._latest_plan_length}m | replans={replan_count} | "
            f"recoveries={self._recovery_count} | err={error}m | "
            f"planner CPU avg={p_avg}% peak={p_peak}% | "
            f"planner RAM avg={p_ram_avg}MB peak={p_ram_peak}MB"
        )

        self._record(
            goal_index, trial_number, x, y, oz, ow,
            final_x, final_y,
            initial_plan_time, total_time,
            status_str,
            p_avg, p_peak, c_avg, c_peak,
            p_ram_avg, p_ram_peak, c_ram_avg, c_ram_peak,
            replan_count,
            replan_frequency,
            mean_replan_interval,
            worst_replan_interval,
            self._latest_plan_length,
            self._travelled_distance,
            self._recovery_count,
            navigation_interrupted,
            recovery_success,
        )

    def _record(self, idx, trial_number, gx, gy, goz, gow,
                fx, fy, plan_t, total_t, status,
                p_avg, p_peak, c_avg, c_peak,
                p_ram_avg, p_ram_peak, c_ram_avg, c_ram_peak,
                replan_count, replan_frequency,
                mean_replan_interval, worst_replan_interval,
                planned_path_length, actual_path_length,
                recovery_count, navigation_interrupted, recovery_success):
        self._results.append({
            "trial_num": trial_number,
            "target_num": idx + 1,
            "given_x": gx,
            "given_y": gy,
            "given_oz": goz,
            "given_ow": gow,
            "final_x": fx,
            "final_y": fy,
            "position_error_m": euclidean(gx, gy, fx, fy),
            "initial_plan_time_s": plan_t,
            "total_time_s": total_t,
            "planned_path_length_m": planned_path_length,
            "actual_path_length_m": round(actual_path_length, 4)
            if actual_path_length is not None else None,
            "status": status,
            "navigation_interrupted": navigation_interrupted,
            "recovery_events": recovery_count,
            "recovery_success": recovery_success,
            "replanning_events": replan_count,
            "replanning_frequency_hz": replan_frequency,
            "mean_replan_interval_s": mean_replan_interval,
            "worst_replan_interval_s": worst_replan_interval,
            "planner_cpu_avg_%": p_avg,
            "planner_cpu_peak_%": p_peak,
            "controller_cpu_avg_%": c_avg,
            "controller_cpu_peak_%": c_peak,
            "planner_ram_avg_mb": p_ram_avg,
            "planner_ram_peak_mb": p_ram_peak,
            "controller_ram_avg_mb": c_ram_avg,
            "controller_ram_peak_mb": c_ram_peak,
            "nodes_expanded": None,
            "planner_set": self.planner,
        })

    def run(self):
        self.get_logger().info(
            f"Starting benchmark | planner='{self.planner}' | "
            f"trial={self.trial_number}"
        )
        time.sleep(1.0)

        # One execution = ONE trial containing all five targets.
        for i, (gx, gy, goz, gow) in enumerate(GOALS):
            self.send_goal(
                i, self.trial_number, gx, gy, goz, gow
            )
            time.sleep(2)

        self._save_results()

    def _save_results(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"nav_benchmark_{self.planner}_{ts}.csv"
        filepath = os.path.join(self.output_dir, filename)

        fields = [
            "trial_num", "target_num",
            "given_x", "given_y", "given_oz", "given_ow",
            "final_x", "final_y", "position_error_m",
            "initial_plan_time_s", "total_time_s",
            "planned_path_length_m", "actual_path_length_m",
            "status", "navigation_interrupted",
            "recovery_events", "recovery_success",
            "replanning_events", "replanning_frequency_hz",
            "mean_replan_interval_s", "worst_replan_interval_s",
            "planner_cpu_avg_%", "planner_cpu_peak_%",
            "controller_cpu_avg_%", "controller_cpu_peak_%",
            "planner_ram_avg_mb", "planner_ram_peak_mb",
            "controller_ram_avg_mb", "controller_ram_peak_mb",
            "nodes_expanded", "planner_set",
        ]

        os.makedirs(self.output_dir, exist_ok=True)
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self._results)

        print(f"\n[benchmark] Results saved → {filepath}\n")
        print("-" * 150)
        print(
            f"{'Trial':<7} {'Target':<8} {'Status':<11} {'Plan':>8} "
            f"{'Total':>8} {'Path':>9} {'Actual':>9} {'Err':>8} "
            f"{'Replan':>8} {'Recover':>8} {'P-CPU':>8} {'P-RAM':>9}"
        )
        print("-" * 150)

        for r in self._results:
            def fmt(value, suffix=""):
                return f"{value}{suffix}" if value is not None else "N/A"

            print(
                f"{r['trial_num']:<7} "
                f"{r['target_num']:<8} "
                f"{r['status']:<11} "
                f"{fmt(r['initial_plan_time_s'], 's'):>8} "
                f"{fmt(r['total_time_s'], 's'):>8} "
                f"{fmt(r['planned_path_length_m'], 'm'):>9} "
                f"{fmt(r['actual_path_length_m'], 'm'):>9} "
                f"{fmt(r['position_error_m'], 'm'):>8} "
                f"{r['replanning_events']:>8} "
                f"{r['recovery_events']:>8} "
                f"{fmt(r['planner_cpu_avg_%'], '%'):>8} "
                f"{fmt(r['planner_ram_avg_mb'], 'MB'):>9}"
            )
        print("-" * 150)


def main():
    parser = argparse.ArgumentParser(description="Nav2 live robot benchmark runner")
    parser.add_argument(
        "--planner", default="dstar_apf",
        help="Planner set name e.g. dstar_apf, navfn_rpp"
    )
    parser.add_argument(
        "--trial", type=int, required=True,
        help="Trial number. One execution runs all five targets for this trial."
    )
    parser.add_argument(
        "--output-dir", default=os.path.expanduser("~/benchmark_results"),
        help="Directory to save CSV results"
    )
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = BenchmarkNode(
        planner=args.planner,
        output_dir=args.output_dir,
        trial_number=args.trial,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        print("\n[benchmark] Interrupted — saving partial results.")
        node._save_results()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()