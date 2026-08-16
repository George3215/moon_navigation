"""Automated evaluation of a single traversal run.

While the navigation stack is running, this node subscribes to the rover pose,
the active navigation mode, and the velocity command, and computes the paper's
metrics:

- traversal time (first pose -> goal arrival or timeout)
- traversal distance (integrated Euclidean pose deltas)
- success (reached the goal within ``arrival_dist``)
- per-mode ``{time_s, distance_m, fraction}`` breakdown

On completion it writes ``metrics.json`` and ``trajectory.csv`` into
``out_dir`` and logs a one-line summary.  Pair it with the launch driver
``scripts/run_experiment.sh`` for the single-mode / multi-mode sweeps.

Usage (stack already running):

    ros2 run mars_navigation_ros2 evaluate --ros-args \
        -p goal_x:=26.5 -p goal_y:=12.5 -p arrival_dist:=2.0 \
        -p timeout_s:=300 -p out_dir:=/tmp/exp/mode1
"""

import csv
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool, String


class Evaluator(Node):
    def __init__(self):
        super().__init__("evaluate")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("mode_topic", "/navigation_mode")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("rollover_topic", "/simulation/rollover")
        self.declare_parameter("goal_x", 20.0)
        self.declare_parameter("goal_y", 10.0)
        self.declare_parameter("arrival_dist", 2.0)
        self.declare_parameter("timeout_s", 300.0)
        self.declare_parameter("out_dir", "/tmp/exp")

        self.goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        self.arrival_dist = float(self.get_parameter("arrival_dist").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.out_dir = self.get_parameter("out_dir").value

        self.start_time = None
        self.last_time = None
        self.last_pose = None
        self.current_mode = "unknown"
        self.mode_start_time = None
        self.total_distance = 0.0
        self.per_mode = {}  # mode -> {"time": float, "distance": float}
        self.trajectory = []  # (t, x, y, mode)
        self.finished = False
        self._done_future = Future()

        self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._pose_cb, 10
        )
        self.create_subscription(String, self.get_parameter("mode_topic").value, self._mode_cb, 10)
        self.create_subscription(Twist, self.get_parameter("cmd_topic").value, self._cmd_cb, 10)
        self.create_subscription(Bool, self.get_parameter("rollover_topic").value, self._rollover_cb, 10)
        self.create_timer(1.0, self._check_timeout)
        self.get_logger().info(
            f"evaluator armed (goal={self.goal}, arrival_dist={self.arrival_dist}, "
            f"timeout={self.timeout_s}s, out={self.out_dir})"
        )

    def _mode_cb(self, msg):
        self._switch_mode(msg.data)

    def _cmd_cb(self, _msg):
        pass

    def _rollover_cb(self, msg):
        if msg.data and not self.finished:
            self.get_logger().warn("rollover detected (terrain slope exceeded mode limit)")
            self._finish(False, "rollover")

    def _switch_mode(self, mode):
        if mode == self.current_mode:
            return
        self._flush_mode(mode)
        self.current_mode = mode
        self.mode_start_time = self.last_time if self.last_time is not None else time.monotonic()

    def _flush_mode(self, next_mode):
        """Close out the elapsed time/distance attributed to ``current_mode``."""
        if self.mode_start_time is None:
            return
        now = time.monotonic()
        entry = self.per_mode.setdefault(self.current_mode, {"time": 0.0, "distance": 0.0})
        entry["time"] += now - self.mode_start_time
        self.mode_start_time = now

    def _pose_cb(self, msg):
        if self.finished:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = time.monotonic()

        if self.start_time is None:
            self.start_time = now
            self.last_time = now
            self.last_pose = (x, y)
            self.mode_start_time = now
            self.trajectory.append((0.0, x, y, self.current_mode))
            self.get_logger().info(f"started at ({x:.2f}, {y:.2f})")
            return

        dx = x - self.last_pose[0]
        dy = y - self.last_pose[1]
        step = math.hypot(dx, dy)
        self.total_distance += step
        entry = self.per_mode.setdefault(self.current_mode, {"time": 0.0, "distance": 0.0})
        entry["distance"] += step
        self.last_pose = (x, y)
        self.last_time = now
        self.trajectory.append((now - self.start_time, x, y, self.current_mode))

        if math.hypot(x - self.goal[0], y - self.goal[1]) <= self.arrival_dist:
            self._finish(True, "goal reached")

    def _check_timeout(self):
        if self.finished or self.start_time is None:
            return
        if time.monotonic() - self.start_time >= self.timeout_s:
            self._finish(False, "timeout")

    def _finish(self, success, reason):
        if self.finished:
            return
        self.finished = True
        end_time = time.monotonic()
        traversal_time = end_time - self.start_time if self.start_time is not None else 0.0
        # Finalize the current mode's elapsed time.
        if self.mode_start_time is not None:
            self.per_mode.setdefault(self.current_mode, {"time": 0.0, "distance": 0.0})
            self.per_mode[self.current_mode]["time"] += end_time - self.mode_start_time

        for mode, entry in self.per_mode.items():
            entry["fraction_time"] = entry["time"] / traversal_time if traversal_time > 0 else 0.0
            entry["fraction_distance"] = entry["distance"] / self.total_distance if self.total_distance > 0 else 0.0

        avg_speed = self.total_distance / traversal_time if traversal_time > 0 else 0.0
        metrics = {
            "success": success,
            "reason": reason,
            "traversal_time_s": round(traversal_time, 3),
            "traversal_distance_m": round(self.total_distance, 3),
            "average_speed_mps": round(avg_speed, 4),
            "goal": list(self.goal),
            "arrival_dist_m": self.arrival_dist,
            "per_mode": self.per_mode,
        }

        os.makedirs(self.out_dir, exist_ok=True)
        with open(os.path.join(self.out_dir, "metrics.json"), "w") as fh:
            json.dump(metrics, fh, indent=2, sort_keys=True)
        with open(os.path.join(self.out_dir, "trajectory.csv"), "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["t", "x", "y", "mode"])
            writer.writerows(self.trajectory)

        modes = ", ".join(
            f"{m}:{v['time']:.1f}s/{v['distance']:.1f}m" for m, v in sorted(self.per_mode.items())
        )
        self.get_logger().info(
            f"DONE {reason} | success={success} | time={traversal_time:.1f}s | "
            f"dist={self.total_distance:.1f}m | speed={avg_speed:.3f}m/s | [{modes}]"
        )
        self.get_logger().info(f"wrote {self.out_dir}/metrics.json + trajectory.csv")
        if not self._done_future.done():
            self._done_future.set_result(True)


def main():
    rclpy.init()
    node = Evaluator()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin_until_future_complete(node._done_future)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
