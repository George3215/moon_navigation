"""Pure-pursuit path follower (ROS2 port of ctrl/c_pursuit_ctrl/src/c_pursuit.cpp).

Selects which path to track based on the active navigation mode:

- Mode 1 (efficient): follows the smoothed global path
  ``/trajectory_ctrl/global_path_updated`` at high speed.
- Mode 2 (safe) / Mode 3 (conservative): follows ``/local_planner/local_path``
  at reduced speed.

Publishes a ``geometry_msgs/Twist`` on ``/cmd_vel`` consumed by the rover (or,
in this reproduction, by the kinematic ``pose_simulator``).
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from .ros_utils import quaternion_to_yaw


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("global_path_topic", "/trajectory_ctrl/global_path_updated")
        self.declare_parameter("local_path_topic", "/local_planner/local_path")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("mode_topic", "/navigation_mode")

        self.declare_parameter("mode1_speed", 2.0)
        self.declare_parameter("mode2_speed", 0.8)
        self.declare_parameter("mode3_speed", 0.5)
        self.declare_parameter("lookahead_distance", 1.0)
        self.declare_parameter("safety_corridor", 1.0)
        self.declare_parameter("max_yaw_rate", 2.5)
        self.declare_parameter("steering_gain", 1.2)
        self.declare_parameter("heading_kp", 1.2)
        self.declare_parameter("turn_speed_gain", 0.18)
        self.declare_parameter("min_tracking_speed", 0.08)
        self.declare_parameter("rotate_in_place_heading_thresh", 0.9)
        self.declare_parameter("control_rate", 10.0)

        self.mode = "Mode 1"
        self.speed = float(self.get_parameter("mode1_speed").value)
        self.pose = None
        self.global_path = None
        self.local_path = None

        self.cmd_pub = self.create_publisher(Twist, self.get_parameter("cmd_topic").value, 1)
        self.create_subscription(String, self.get_parameter("mode_topic").value, self._mode_cb, 10)
        self.create_subscription(Path, self.get_parameter("global_path_topic").value, self._global_path_cb, 1)
        self.create_subscription(Path, self.get_parameter("local_path_topic").value, self._local_path_cb, 1)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._pose_cb, 10)

        self.create_timer(1.0 / float(self.get_parameter("control_rate").value), self._control)
        self.get_logger().info("path_follower ready")

    def _mode_cb(self, msg):
        self.mode = msg.data
        if self.mode == "Mode 1":
            self.speed = float(self.get_parameter("mode1_speed").value)
        elif self.mode == "Mode 2":
            self.speed = float(self.get_parameter("mode2_speed").value)
        elif self.mode == "Mode 3":
            self.speed = float(self.get_parameter("mode3_speed").value)
        else:
            self.speed = 0.0
        self.get_logger().info(f"mode -> {self.mode} (speed {self.speed:.2f} m/s)")

    def _global_path_cb(self, msg):
        self.global_path = msg

    def _local_path_cb(self, msg):
        self.local_path = msg

    def _pose_cb(self, msg):
        self.pose = msg

    def _active_path(self):
        if self.mode == "Mode 1":
            return self.global_path
        return self.local_path

    def _to_body_frame(self, path):
        """Transform a world-frame path into rover body frame using pose yaw."""
        if self.pose is None:
            return []
        yaw = quaternion_to_yaw(self.pose.pose.pose.orientation)
        px = self.pose.pose.pose.position.x
        py = self.pose.pose.pose.position.y
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        body = []
        for pose in path.poses:
            dx = pose.pose.position.x - px
            dy = pose.pose.position.y - py
            body.append((dx * cos_y + dy * sin_y, -dx * sin_y + dy * cos_y))
        return body

    def _control(self):
        cmd = Twist()
        path = self._active_path()
        if self.pose is None or path is None or not path.poses:
            self.cmd_pub.publish(cmd)
            return

        body = self._to_body_frame(path)
        if not body:
            self.cmd_pub.publish(cmd)
            return

        lookahead = float(self.get_parameter("lookahead_distance").value)
        target = body[-1]
        for x, y in body:
            if x > 0 and math.hypot(x, y) >= lookahead:
                target = (x, y)
                break

        tx, ty = target
        if tx <= 0:
            # Look-ahead point is behind the rover: rotate in place.
            cmd.linear.x = 0.0
            cmd.angular.z = float(self.get_parameter("max_yaw_rate").value) * (1.0 if ty > 0 else -1.0)
            self.cmd_pub.publish(cmd)
            return

        L = math.hypot(tx, ty)
        alpha = math.atan2(ty, tx)

        if abs(alpha) > float(self.get_parameter("rotate_in_place_heading_thresh").value):
            cmd.linear.x = 0.0
            cmd.angular.z = float(self.get_parameter("max_yaw_rate").value) * (1.0 if alpha > 0 else -1.0)
            self.cmd_pub.publish(cmd)
            return

        steering_gain = float(self.get_parameter("steering_gain").value)
        heading_kp = float(self.get_parameter("heading_kp").value)
        max_yaw = float(self.get_parameter("max_yaw_rate").value)
        curvature_omega = steering_gain * (2.0 * self.speed * math.sin(alpha)) / max(0.20, L)
        heading_omega = heading_kp * alpha
        omega = max(-max_yaw, min(max_yaw, curvature_omega + heading_omega))

        turn_speed_gain = float(self.get_parameter("turn_speed_gain").value)
        normalized_turn = abs(omega) / max(0.05, max_yaw)
        speed_scale = max(0.15, 1.0 - turn_speed_gain * normalized_turn)
        speed = max(float(self.get_parameter("min_tracking_speed").value), self.speed * speed_scale)

        # Stop near the end of the path.
        goal_dist = math.hypot(body[-1][0], body[-1][1])
        if goal_dist < float(self.get_parameter("safety_corridor").value):
            speed = 0.0
            omega = 0.0

        cmd.linear.x = speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
