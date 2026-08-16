import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .ros_utils import make_pose_with_covariance

# Terrain constants (must match generate_mixed_terrain.py / image_to_map.py).
REAL_M = 54.0
RES = 0.2
OX = -27.0
OY = -27.0
HPG = 4.820803273566 / 255.0


class PoseSimulator(Node):
    """Kinematic rover simulator with a slope-based rollover failure model.

    Integrates ``/cmd_vel`` into a pose and publishes ``/pose_with_covariance``.
    In addition, it samples the terrain slope under the rover from the heightmap
    and, if that slope exceeds the active navigation mode's limit, declares a
    rollover: it freezes the pose and publishes ``/simulation/rollover`` so the
    evaluator can mark the run failed.

    This reproduces the paper's success-rate differentiation: Mode 1 (flat,
    2.0 m/s) rolls over on slopes/rocks, Mode 2 (rocky, 0.8 m/s) rolls over on
    the steeper ridge but survives scattered rocks by routing around them, and
    Mode 3 (challenging, 0.5 m/s) survives the ridge because it is slope-aware
    and slow.
    """

    def __init__(self):
        super().__init__("pose_simulator")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("mode_topic", "/navigation_mode")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("x", -22.0)
        self.declare_parameter("y", -22.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("rate", 20.0)

        self.declare_parameter("rollover_topic", "/simulation/rollover")
        self.declare_parameter("enable_rollover", True)
        self.declare_parameter("mode1_slope_limit_deg", 8.0)
        self.declare_parameter("mode2_slope_limit_deg", 12.0)
        self.declare_parameter("mode3_slope_limit_deg", 40.0)
        self.declare_parameter("image_path", "")
        self.declare_parameter("real_meter", REAL_M)
        self.declare_parameter("resolution", RES)
        self.declare_parameter("height_per_gray", HPG)

        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.last = self.get_clock().now()
        self.cmd = Twist()
        self.mode = "Mode 1"
        self.rolled_over = False

        self.slope_deg = self._load_slope_map()
        self.slope_limit = self._slope_limit()

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, 10
        )
        self.rollover_pub = self.create_publisher(Bool, self.get_parameter("rollover_topic").value, 1)
        self.create_subscription(Twist, self.get_parameter("cmd_topic").value, self._cmd, 10)
        self.create_subscription(String, self.get_parameter("mode_topic").value, self._mode_cb, 10)
        self.timer = self.create_timer(1.0 / float(self.get_parameter("rate").value), self._tick)
        self.get_logger().info(
            f"pose_simulator ready (rollover limits M1={self.get_parameter('mode1_slope_limit_deg').value}deg "
            f"M2={self.get_parameter('mode2_slope_limit_deg').value}deg "
            f"M3={self.get_parameter('mode3_slope_limit_deg').value}deg)"
        )

    def _load_slope_map(self):
        path = self.get_parameter("image_path").value
        if not path:
            self.get_logger().warn("no image_path set; rollover model disabled")
            return None
        from .elevation_synthesis import load_height

        height = load_height(
            path,
            float(self.get_parameter("real_meter").value),
            float(self.get_parameter("resolution").value),
            float(self.get_parameter("height_per_gray").value),
        )
        res = float(self.get_parameter("resolution").value)
        gy, gx = np.gradient(height, res)
        return np.degrees(np.arctan(np.sqrt(gx * gx + gy * gy)))

    def _slope_limit(self):
        limits = {
            "Mode 1": float(self.get_parameter("mode1_slope_limit_deg").value),
            "Mode 2": float(self.get_parameter("mode2_slope_limit_deg").value),
            "Mode 3": float(self.get_parameter("mode3_slope_limit_deg").value),
        }
        return limits.get(self.mode, limits["Mode 1"])

    def _cmd(self, msg):
        self.cmd = msg

    def _mode_cb(self, msg):
        self.mode = msg.data
        self.slope_limit = self._slope_limit()

    def _terrain_slope(self):
        if self.slope_deg is None:
            return 0.0
        res = float(self.get_parameter("resolution").value)
        col = int(round((self.x - OX) / res))
        row = int(round((self.y - OY) / res))
        h, w = self.slope_deg.shape
        if not (0 <= row < h and 0 <= col < w):
            return 0.0
        return float(self.slope_deg[row, col])

    def _tick(self):
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last).nanoseconds * 1e-9)
        self.last = now

        if not self.rolled_over:
            slope = self._terrain_slope()
            if self.get_parameter("enable_rollover").value and slope > self.slope_limit:
                self.rolled_over = True
                self.get_logger().warn(
                    f"ROLLOVER: terrain slope {slope:.1f} deg exceeds "
                    f"{self.mode} limit {self.slope_limit:.1f} deg at "
                    f"({self.x:.2f}, {self.y:.2f})"
                )
                msg = Bool()
                msg.data = True
                self.rollover_pub.publish(msg)
                # Freeze: a rolled-over rover no longer moves.
                self.cmd = Twist()

        self.x += self.cmd.linear.x * math.cos(self.yaw) * dt
        self.y += self.cmd.linear.x * math.sin(self.yaw) * dt
        self.yaw += self.cmd.angular.z * dt
        msg = make_pose_with_covariance(
            self.get_parameter("frame_id").value, now.to_msg(), self.x, self.y, self.yaw
        )
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PoseSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
