"""Local A* planner (ROS2 port of planning/local_planner/scripts/local_planner.py).

Consumes the local costmap and local goal produced by the map server and
publishes a collision-free local path on ``/local_planner/local_path``. This is
the planner used by Mode 2 (rocky) and Mode 3 (challenging).
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String

from .astar import AStar
from .ros_utils import make_pose_stamped


class LocalPlanner(Node):
    def __init__(self):
        super().__init__("local_planner")
        self.declare_parameter("local_map_topic", "/local_planner/local_map")
        self.declare_parameter("local_goal_topic", "/local_planner/local_goal")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("replan_topic", "/map_server/replan_signal")
        self.declare_parameter("path_topic", "/local_planner/local_path")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("plan_period", 1.0)

        self.declare_parameter("search_resolution", 0.5)
        self.declare_parameter("diagonal_movement", True)
        self.declare_parameter("heuristic_weight", 0.85)
        self.declare_parameter("dilation_radius_meters", 0.5)
        self.declare_parameter("hard_obstacle_threshold", 88)
        self.declare_parameter("terrain_cost_weight", 3.0)

        self.map_msg = None
        self.goal = None
        self.pose = None
        self.last_goal = None
        self.last_start = None

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 1)
        self.create_subscription(OccupancyGrid, self.get_parameter("local_map_topic").value, self._map_cb, 1)
        self.create_subscription(PoseStamped, self.get_parameter("local_goal_topic").value, self._goal_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._pose_cb, 10)
        self.create_subscription(String, self.get_parameter("replan_topic").value, self._replan_cb, 10)

        self.create_timer(float(self.get_parameter("plan_period").value), self._tick)
        self.get_logger().info("local_planner ready")

    def _map_cb(self, msg):
        self.map_msg = msg

    def _goal_cb(self, msg):
        self.goal = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _replan_cb(self, msg):
        if msg.data in ("True", "true", "1"):
            self.last_goal = None  # force a replan

    def _tick(self):
        if self.map_msg is None or self.goal is None or self.pose is None:
            self.get_logger().info("waiting for local map/goal/pose...", throttle_duration_sec=5.0)
            return

        goal_pt = (self.goal.pose.position.x, self.goal.pose.position.y)
        start_pt = (self.pose.pose.pose.position.x, self.pose.pose.pose.position.y)

        # Replan on every tick: the local map is re-centred on the rover each
        # tick, so a path cached from an earlier pose would otherwise drift out
        # of the window and stall the follower just short of the goal. The A*
        # search on the 20 m window is cheap, so replanning is not a burden.
        if (
            self.last_goal is not None
            and goal_pt == self.last_goal
            and self.last_start is not None
            and math.hypot(start_pt[0] - self.last_start[0], start_pt[1] - self.last_start[1]) < 0.5
        ):
            return  # stationary: no replan needed this tick
        self.last_goal = goal_pt
        self.last_start = start_pt

        params = {k: self.get_parameter(k).value for k in (
            "search_resolution", "diagonal_movement", "heuristic_weight",
            "dilation_radius_meters", "hard_obstacle_threshold", "terrain_cost_weight",
        )}

        try:
            planner = AStar(self.map_msg, start_pt, goal_pt, **params)
        except Exception as exc:  # noqa: BLE001 - defensive: bad map/start/goal
            self.get_logger().warn(f"A* init failed: {exc}")
            return

        path = planner.search()
        if path is None:
            self.get_logger().warn("no local path found")
            return

        out = Path()
        out.header.frame_id = self.get_parameter("frame_id").value
        out.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            out.poses.append(make_pose_stamped(out.header.frame_id, out.header.stamp, x, y))
        self.path_pub.publish(out)
        self.get_logger().info(f"published local path with {len(out.poses)} points")


def main():
    rclpy.init()
    node = LocalPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
