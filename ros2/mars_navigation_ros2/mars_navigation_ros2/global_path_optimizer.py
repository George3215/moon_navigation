"""Global path smoother (ROS2 port of planning/global_path_optimizer/scripts/traj_gen.py).

For the efficient mode (Mode 1), the rover follows a smooth curve instead of the
raw A* polyline.  This node subscribes to the raw global path
(``/map_server/global_path_updated``) and, while in Mode 1, continuously
re-publishes a Catmull-Rom smoothed trajectory on
``/trajectory_ctrl/global_path_updated``.

The trajectory spans the *full* remaining path: the rover's current pose, then
every not-yet-passed waypoint up to the goal, resampled to a dense curve.  The
original node generated a single Hermite segment and then went silent, so the
rover traversed only the first ~5 m and stopped; re-publishing on a timer keeps
the rover tracking the whole route.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from .ros_utils import make_pose_stamped


def catmull_rom(points, samples_per_segment=8):
    """Resample a (N, 2) polyline with a Catmull-Rom spline (C1 smooth)."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return pts
    p = np.vstack([pts[0], pts, pts[-1]])  # padded tangents
    out = [pts[0]]
    for i in range(len(pts) - 1):
        p0, p1, p2, p3 = p[i], p[i + 1], p[i + 2], p[i + 3]
        for t in np.linspace(0.0, 1.0, samples_per_segment, endpoint=False):
            t2, t3 = t * t, t * t * t
            pt = 0.5 * (
                (2.0 * p1)
                + (-p0 + p2) * t
                + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2
                + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3
            )
            out.append(pt)
    out.append(pts[-1])
    return np.array(out)


class GlobalPathOptimizer(Node):
    def __init__(self):
        super().__init__("global_path_optimizer")
        self.declare_parameter("global_path_input_topic", "/map_server/global_path_updated")
        self.declare_parameter("global_trajectory_output_topic", "/trajectory_ctrl/global_path_updated")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("mode_topic", "/navigation_mode")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("resample_spacing_m", 0.5)
        self.declare_parameter("publish_period", 1.0)

        self.pose = None
        self.path = None  # list of (x, y)
        self.mode = "Mode 1"

        self.traj_pub = self.create_publisher(
            Path, self.get_parameter("global_trajectory_output_topic").value, 1
        )
        self.create_subscription(Path, self.get_parameter("global_path_input_topic").value, self._path_cb, 1)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._pose_cb, 10)
        self.create_subscription(String, self.get_parameter("mode_topic").value, self._mode_cb, 10)
        self.create_timer(float(self.get_parameter("publish_period").value), self._tick)
        self.get_logger().info("global_path_optimizer ready")

    def _pose_cb(self, msg):
        self.pose = msg

    def _mode_cb(self, msg):
        self.mode = msg.data

    def _path_cb(self, msg):
        if not msg.poses:
            return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _tick(self):
        if self.mode != "Mode 1" or self.pose is None or self.path is None or len(self.path) < 2:
            return
        px = self.pose.pose.pose.position.x
        py = self.pose.pose.pose.position.y

        # Drop every waypoint up to (and including) the one nearest the rover,
        # then always keep the final goal so the curve still terminates there.
        nearest = min(range(len(self.path)), key=lambda i: np.hypot(self.path[i][0] - px, self.path[i][1] - py))
        tail = self.path[nearest + 1:]
        if not tail:
            tail = [self.path[-1]]
        elif tail[-1] != self.path[-1]:
            tail.append(self.path[-1])

        points = [(px, py)] + list(tail)
        if len(points) < 2:
            return

        # Resample each segment so the curve is roughly ``resample_spacing_m``.
        pts = np.asarray(points, dtype=float)
        seg_lens = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
        samples = max(2, int(round(float(np.max(seg_lens)) / float(self.get_parameter("resample_spacing_m").value))))
        curve = catmull_rom(points, samples_per_segment=samples)

        out = Path()
        out.header.frame_id = self.get_parameter("frame_id").value
        out.header.stamp = self.get_clock().now().to_msg()
        for x, y in curve:
            out.poses.append(make_pose_stamped(out.header.frame_id, out.header.stamp, x, y))
        self.traj_pub.publish(out)


def main():
    rclpy.init()
    node = GlobalPathOptimizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
