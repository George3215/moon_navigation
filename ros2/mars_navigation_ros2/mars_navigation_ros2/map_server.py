import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mars_navigation_interfaces.srv import GetGlobalMap, InitializeMap, InitializePath, UpdateMap
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import String

from .ros_utils import make_pose_stamped


class MapServer(Node):
    def __init__(self):
        super().__init__("map_server")
        self.declare_parameter("globalmap_resolution", 1.0)
        self.declare_parameter("localmap_resolution", 0.2)
        self.declare_parameter("local_map_size", 20.0)
        self.declare_parameter("local_goal_lookahead", 6.0)
        self.declare_parameter("collision_check_period", 1.0)
        self.declare_parameter("obstacle_cost_threshold", 88)
        self.declare_parameter("mode1_map_topic", "/mode1/occupancy_grid")
        self.declare_parameter("mode2_map_topic", "/mode2/occupancy_grid")
        self.declare_parameter("mode3_map_topic", "/mode3/occupancy_grid")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("mode_topic", "/navigation_mode")

        self.global_map = None
        self.mode_map = None
        self.global_path = None
        self.pose = None
        self.goal = None
        self.mode = "Mode 1"
        self.last_tracked_index = 0
        self.lock = threading.Lock()

        self.global_map_pub = self.create_publisher(OccupancyGrid, "/map_server/global_map", 1)
        self.mode_map_pub = self.create_publisher(OccupancyGrid, "/map_server/mode_map", 1)
        self.global_path_pub = self.create_publisher(Path, "/map_server/global_path_updated", 1)
        self.local_map_pub = self.create_publisher(OccupancyGrid, "/local_planner/local_map", 1)
        self.local_goal_pub = self.create_publisher(PoseStamped, "/local_planner/local_goal", 1)
        self.replan_pub = self.create_publisher(String, "/map_server/replan_signal", 1)

        self.create_subscription(OccupancyGrid, self.get_parameter("mode1_map_topic").value, self._mode1_map, 1)
        self.create_subscription(OccupancyGrid, self.get_parameter("mode2_map_topic").value, lambda m: self._update_map(m, 2), 1)
        self.create_subscription(OccupancyGrid, self.get_parameter("mode3_map_topic").value, lambda m: self._update_map(m, 3), 1)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._pose, 10)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self._goal, 10)
        self.create_subscription(String, self.get_parameter("mode_topic").value, self._mode, 10)
        self.create_subscription(Path, "/global_planner/global_path", self._global_path, 1)
        self.create_subscription(Path, "/trajectory_ctrl/global_path_updated", self._path_for_collision, 1)
        self.create_subscription(Path, "/local_planner/local_path", self._path_for_collision, 1)

        self.create_service(InitializeMap, "Initialize_Map", self._initialize_map_srv)
        self.create_service(UpdateMap, "Update_Map", self._update_map_srv)
        self.create_service(GetGlobalMap, "Get_Globalmap", self._get_globalmap_srv)
        self.create_service(GetGlobalMap, "Get_Localmap", self._get_localmap_srv)
        self.create_service(InitializePath, "Initialize_Path", self._initialize_path_srv)

        self.create_timer(float(self.get_parameter("collision_check_period").value), self._tick)

    def _mode1_map(self, msg):
        with self.lock:
            if self.global_map is None:
                self._initialize_global_map(msg)
                self.get_logger().info(
                    f"global map initialized from {self.get_parameter('mode1_map_topic').value}: "
                    f"{msg.info.width}x{msg.info.height}, res={msg.info.resolution}"
                )

    def _initialize_global_map(self, msg):
        self.global_map = msg
        self.mode_map = OccupancyGrid()
        self.mode_map.header = msg.header
        self.mode_map.info = msg.info
        self.mode_map.data = [1] * (msg.info.width * msg.info.height)
        self._publish_maps()

    def _update_map(self, msg, mode):
        with self.lock:
            if self.global_map is None:
                return
            self._merge_local_map(msg, mode)
            self._publish_maps()

    def _merge_local_map(self, update_map, mode):
        gm = self.global_map
        dx = round((update_map.info.origin.position.x - gm.info.origin.position.x) / gm.info.resolution)
        dy = round((update_map.info.origin.position.y - gm.info.origin.position.y) / gm.info.resolution)
        data = list(gm.data)
        modes = list(self.mode_map.data)
        for y in range(update_map.info.height):
            for x in range(update_map.info.width):
                value = update_map.data[y * update_map.info.width + x]
                if value < 0:
                    continue
                gx = x + dx
                gy = y + dy
                if 0 <= gx < gm.info.width and 0 <= gy < gm.info.height:
                    idx = gy * gm.info.width + gx
                    if mode >= modes[idx]:
                        data[idx] = int(value)
                        modes[idx] = int(mode)
        gm.data = data
        self.mode_map.data = modes

    def _publish_maps(self):
        now = self.get_clock().now().to_msg()
        self.global_map.header.stamp = now
        self.mode_map.header.stamp = now
        self.global_map_pub.publish(self.global_map)
        self.mode_map_pub.publish(self.mode_map)

    def _pose(self, msg):
        self.pose = msg

    def _goal(self, msg):
        self.goal = msg

    def _mode(self, msg):
        self.mode = msg.data

    def _global_path(self, msg):
        with self.lock:
            self.global_path = msg
            self.last_tracked_index = 0
            self.global_path_pub.publish(msg)

    def _path_for_collision(self, _msg):
        pass

    def _initialize_map_srv(self, request, response):
        with self.lock:
            self._initialize_global_map(request.global_map)
            response.initialized = True
        return response

    def _update_map_srv(self, request, response):
        with self.lock:
            if self.global_map is None:
                response.updated = False
            else:
                self._merge_local_map(request.update_map, request.mode)
                self._publish_maps()
                response.updated = True
        return response

    def _get_globalmap_srv(self, _request, response):
        if self.global_map is not None:
            self.global_map_pub.publish(self.global_map)
            response.success = True
        else:
            response.success = False
        return response

    def _get_localmap_srv(self, _request, response):
        local = self._make_local_map()
        if local is not None:
            self.local_map_pub.publish(local)
            response.success = True
        else:
            response.success = False
        return response

    def _initialize_path_srv(self, request, response):
        with self.lock:
            self.global_path = request.path
            self.global_path_pub.publish(request.path)
            response.success = True
        return response

    def _tick(self):
        with self.lock:
            if self.global_map is not None:
                self._publish_maps()
            if self.global_path is not None:
                self.global_path_pub.publish(self.global_path)
                local_goal = self._local_goal()
                if local_goal is not None:
                    self.local_goal_pub.publish(local_goal)
            local = self._make_local_map()
            if local is not None:
                self.local_map_pub.publish(local)

    def _local_goal(self):
        if self.pose is None or self.global_path is None or not self.global_path.poses:
            return None
        px = self.pose.pose.pose.position.x
        py = self.pose.pose.pose.position.y
        lookahead = float(self.get_parameter("local_goal_lookahead").value)
        poses = self.global_path.poses
        start_idx = min(self.last_tracked_index, len(poses) - 1)
        nearest = start_idx
        nearest_dist = float("inf")
        for i in range(start_idx, len(poses)):
            dx = poses[i].pose.position.x - px
            dy = poses[i].pose.position.y - py
            dist = dx * dx + dy * dy
            if dist < nearest_dist:
                nearest_dist = dist
                nearest = i
        self.last_tracked_index = nearest
        accum = 0.0
        target = poses[-1]
        for i in range(nearest, len(poses) - 1):
            x0, y0 = poses[i].pose.position.x, poses[i].pose.position.y
            x1, y1 = poses[i + 1].pose.position.x, poses[i + 1].pose.position.y
            accum += math.hypot(x1 - x0, y1 - y0)
            if accum >= lookahead:
                target = poses[i + 1]
                break
        return target

    def _make_local_map(self):
        if self.global_map is None or self.pose is None:
            return None
        gm = self.global_map
        size_m = float(self.get_parameter("local_map_size").value)
        cells = max(1, int(round(size_m / gm.info.resolution)))
        px = self.pose.pose.pose.position.x
        py = self.pose.pose.pose.position.y
        center_x = int((px - gm.info.origin.position.x) / gm.info.resolution)
        center_y = int((py - gm.info.origin.position.y) / gm.info.resolution)
        half = cells // 2
        local = OccupancyGrid()
        local.header = gm.header
        local.info.resolution = gm.info.resolution
        local.info.width = cells
        local.info.height = cells
        local.info.origin.position.x = gm.info.origin.position.x + (center_x - half) * gm.info.resolution
        local.info.origin.position.y = gm.info.origin.position.y + (center_y - half) * gm.info.resolution
        local.info.origin.orientation.w = 1.0
        out = []
        for y in range(center_y - half, center_y - half + cells):
            for x in range(center_x - half, center_x - half + cells):
                if 0 <= x < gm.info.width and 0 <= y < gm.info.height:
                    out.append(int(gm.data[y * gm.info.width + x]))
                else:
                    out.append(-1)
        local.data = out
        return local


def main():
    rclpy.init()
    node = MapServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
