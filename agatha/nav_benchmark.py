#!/usr/bin/env python3
"""
Navigation Benchmark Script – Live Robot
Sends a sequence of goals and logs timing + position + CPU results
to a timestamped CSV file.

Usage:
  python3 nav_benchmark_live.py --planner dstar_apf
  python3 nav_benchmark_live.py --planner navfn_rpp --output-dir ~/results
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
import tf2_ros


# ── Edit these to match your map ──────────────────────────────────────────────

GOALS = [
    # (x, y, orientation_z, orientation_w)
    ( 1.17,  -0.05,  -0.71,  0.71),
    ( 1.64,  -2.47,  -0.71,  0.71),
    ( 1.96,  -0.06,   0.00,  1.00),
    ( 0.56,   0.23,   0.00,  1.00),
    ( 1.47,  0.56,    0.00,  1.00),
    ( 0.56,   0.23,   0.00,  1.00),

]

# ── Process name fragments to monitor ────────────────────────────────────────

PLANNER_PROC    = "planner_server"
CONTROLLER_PROC = "controller_server"
CPU_SAMPLE_HZ   = 2   # samples per second


# ── CPU monitor ───────────────────────────────────────────────────────────────

class CPUMonitor:
    """Samples CPU usage of named ROS2 processes in a background thread."""

    def __init__(self):
        self._running    = False
        self._thread     = None
        self._lock       = threading.Lock()
        self._planner    = []
        self._controller = []

    def _find_proc(self, name_fragment: str):
        for p in psutil.process_iter(["pid", "cmdline", "cpu_percent"]):
            try:
                cmd = " ".join(p.info["cmdline"] or [])
                if name_fragment in cmd:
                    return p
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return None

    def _sample_loop(self):
        # warm up cpu_percent (first call always returns 0)
        p_proc = self._find_proc(PLANNER_PROC)
        c_proc = self._find_proc(CONTROLLER_PROC)
        if p_proc:
            p_proc.cpu_percent()
        if c_proc:
            c_proc.cpu_percent()
        time.sleep(0.5)

        while self._running:
            p_proc = self._find_proc(PLANNER_PROC)
            c_proc = self._find_proc(CONTROLLER_PROC)
            with self._lock:
                if p_proc:
                    try:
                        self._planner.append(p_proc.cpu_percent())
                    except psutil.NoSuchProcess:
                        pass
                if c_proc:
                    try:
                        self._controller.append(c_proc.cpu_percent())
                    except psutil.NoSuchProcess:
                        pass
            time.sleep(1.0 / CPU_SAMPLE_HZ)

    def start(self):
        self._planner    = []
        self._controller = []
        self._running    = True
        self._thread     = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def results(self):
        with self._lock:
            p = list(self._planner)
            c = list(self._controller)

        def stats(lst):
            if not lst:
                return None, None
            return round(sum(lst) / len(lst), 2), round(max(lst), 2)

        p_avg,  p_peak = stats(p)
        c_avg,  c_peak = stats(c)
        return p_avg, p_peak, c_avg, c_peak


# ── helpers ───────────────────────────────────────────────────────────────────

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


# ── main node ─────────────────────────────────────────────────────────────────

class BenchmarkNode(Node):

    def __init__(self, planner: str, output_dir: str):
        super().__init__("nav_benchmark")
        self.planner    = planner
        self.output_dir = output_dir

        self._client    = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listen = tf2_ros.TransformListener(self._tf_buffer, self)
        self._cpu       = CPUMonitor()
        self._results   = []

    def send_goal(self, goal_index: int, x: float, y: float, oz: float = 0.0, ow: float = 1.0):
        self.get_logger().info(f"Goal {goal_index+1}/{len(GOALS)}: ({x}, {y})")

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            return

        goal_msg            = NavigateToPose.Goal()
        goal_msg.pose       = make_goal_pose(x, y, oz, ow)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # start CPU monitoring for this goal
        self._cpu.start()
        send_time = time.time()

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            self._cpu.stop()
            self._record(goal_index, x, y, oz, ow, None, None, None, None,
                     "REJECTED", None, None, None, None)
            return

        plan_time = time.time() - send_time

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        total_time = time.time() - send_time

        # stop CPU monitoring
        self._cpu.stop()
        p_avg, p_peak, c_avg, c_peak = self._cpu.results()

        status = result_future.result().status
        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED:   "ABORTED",
            GoalStatus.STATUS_CANCELED:  "CANCELED",
        }.get(status, f"STATUS_{status}")

        final_x, final_y = get_robot_pose(self._tf_buffer)
        error = euclidean(x, y, final_x, final_y)

        self.get_logger().info(
            f"  → {status_str} | plan≈{plan_time:.2f}s | total={total_time:.2f}s "
            f"| final=({final_x:.3f},{final_y:.3f}) | err={error}m "
            f"| planner CPU avg={p_avg}% peak={p_peak}% "
            f"| controller CPU avg={c_avg}% peak={c_peak}%"
        )
        self._record(goal_index, x, y, oz, ow, final_x, final_y,
                 round(plan_time, 3), round(total_time, 3),
                 status_str, p_avg, p_peak, c_avg, c_peak)

    def _record(self, idx, gx, gy, goz, gow, fx, fy,
                plan_t, total_t, status,
                p_avg, p_peak, c_avg, c_peak):
        self._results.append({
            "goal_num":              idx + 1,
            "given_x":               gx,
            "given_y":               gy,
            "given_oz":              goz,
            "given_ow":              gow,
            "final_x":               fx,
            "final_y":               fy,
            "position_error_m":      euclidean(gx, gy, fx, fy),
            "plan_time_s":           plan_t,
            "total_time_s":          total_t,
            "status":                status,
            "planner_cpu_avg_%":     p_avg,
            "planner_cpu_peak_%":    p_peak,
            "controller_cpu_avg_%":  c_avg,
            "controller_cpu_peak_%": c_peak,
            "planner_set":           self.planner,
        })

    def run(self):
        self.get_logger().info(
            f"Starting benchmark | planner='{self.planner}'"
        )
        time.sleep(2.0)

        for i, (gx, gy, goz, gow) in enumerate(GOALS):
            self.send_goal(i, gx, gy, goz, gow)
            time.sleep(0.5)

        self._save_results()

    def _save_results(self):
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"nav_benchmark_{self.planner}_{ts}.csv"
        filepath = os.path.join(self.output_dir, filename)

        fields = [
            "goal_num", "given_x", "given_y", "given_oz", "given_ow",
            "final_x", "final_y", "position_error_m",
            "plan_time_s", "total_time_s", "status",
            "planner_cpu_avg_%", "planner_cpu_peak_%",
            "controller_cpu_avg_%", "controller_cpu_peak_%",
            "planner_set",
        ]

        os.makedirs(self.output_dir, exist_ok=True)
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self._results)

        print(f"\n[benchmark] Results saved → {filepath}\n")
        print("-" * 90)
        print(f"{'#':<4} {'Goal':<14} {'Final':<18} {'Err':>6} {'Plan':>7} "
              f"{'Total':>7} {'P-avg':>7} {'P-peak':>8} {'C-avg':>7} {'C-peak':>8}  Status")
        print("-" * 90)
        for r in self._results:
            fx  = f"{r['final_x']:.3f}"           if r['final_x']               is not None else "N/A"
            fy  = f"{r['final_y']:.3f}"           if r['final_y']               is not None else "N/A"
            err = f"{r['position_error_m']:.3f}m" if r['position_error_m']      is not None else "N/A"
            pt  = f"{r['plan_time_s']}s"          if r['plan_time_s']           is not None else "N/A"
            tt  = f"{r['total_time_s']}s"         if r['total_time_s']          is not None else "N/A"
            pa  = f"{r['planner_cpu_avg_%']}%"    if r['planner_cpu_avg_%']     is not None else "N/A"
            pp  = f"{r['planner_cpu_peak_%']}%"   if r['planner_cpu_peak_%']    is not None else "N/A"
            ca  = f"{r['controller_cpu_avg_%']}%" if r['controller_cpu_avg_%']  is not None else "N/A"
            cp  = f"{r['controller_cpu_peak_%']}%"if r['controller_cpu_peak_%'] is not None else "N/A"
            print(
                f"{r['goal_num']:<4} "
                f"({r['given_x']:.2f},{r['given_y']:.2f}){'':>2} "
                f"({fx},{fy}) "
                f"{err:>8} {pt:>8} {tt:>8} "
                f"{pa:>8} {pp:>9} {ca:>8} {cp:>9}  {r['status']}"
            )
        print("-" * 90)


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Nav2 live robot benchmark runner")
    parser.add_argument("--planner",    default="dstar_apf",
                        help="Planner set name e.g. dstar_apf, navfn_rpp")
    parser.add_argument("--output-dir", default=os.path.expanduser("~/benchmark_results"),
                        help="Directory to save CSV results")
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = BenchmarkNode(planner=args.planner,
                         output_dir=args.output_dir)
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