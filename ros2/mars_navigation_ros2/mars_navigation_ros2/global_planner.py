"""Global path planner (ROS2 port of nav_framework/global_initializer/scripts/image_planning_server.py).

Subscribes to the global costmap published by the map server, runs a coarse A*
search from the rover pose to a goal, downsamples the result into sparse
waypoints, and publishes a nav_msgs/Path on ``/global_planner/global_path``.

The goal can be supplied either from parameters (``use_manual_goal``, useful
for headless reproduction) or from the RViz "2D Nav Goal" topic
``/move_base_simple/goal``.
"""

import heapq
from collections import deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node

from .ros_utils import make_pose_stamped


def _occupancy_grid_to_numpy(msg):
    return np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)


def _world_to_grid(wx, wy, origin_x, origin_y, resolution):
    col = int(round((wx - origin_x) / resolution))
    row = int(round((wy - origin_y) / resolution))
    return row, col


def _grid_to_world(row, col, origin_x, origin_y, resolution):
    wx = col * resolution + origin_x + resolution / 2.0
    wy = row * resolution + origin_y + resolution / 2.0
    return wx, wy


def _downsample_path_by_step(path_world, step_size):
    if len(path_world) <= 2:
        return path_world
    result = [path_world[0]]
    accum = 0.0
    for i in range(1, len(path_world)):
        dx = path_world[i][0] - path_world[i - 1][0]
        dy = path_world[i][1] - path_world[i - 1][1]
        accum += float(np.sqrt(dx * dx + dy * dy))
        if accum >= step_size:
            result.append(path_world[i])
            accum = 0.0
    if result[-1] != path_world[-1]:
        result.append(path_world[-1])
    return result


def _aggregate_cost_map(cost_map, obstacle_mask, scale_factor):
    h, w = cost_map.shape
    new_h, new_w = h // scale_factor, w // scale_factor
    new_cost = np.zeros((new_h, new_w), dtype=np.float32)
    new_obs = np.zeros((new_h, new_w), dtype=bool)
    for i in range(new_h):
        for j in range(new_w):
            r0, r1 = i * scale_factor, (i + 1) * scale_factor
            c0, c1 = j * scale_factor, (j + 1) * scale_factor
            new_obs[i, j] = np.any(obstacle_mask[r0:r1, c0:c1])
            new_cost[i, j] = np.mean(cost_map[r0:r1, c0:c1])
    return new_cost, new_obs


def _nearest_free_cell(mask, start):
    """Return the nearest non-obstacle cell to ``start`` (8-connected BFS).

    The rover drives within one grid cell of the inflated boulder / crater-rim
    obstacles, so its own cell is sometimes flagged obstructed; A* would then
    refuse the start and emit ``global A* failed`` every tick, freezing the
    follower on a stale path.  Snapping the start to the closest free cell keeps
    replanning alive without changing the first waypoint (``_plan`` still pins
    ``path_world[0]`` to the true rover pose).
    """
    h, w = mask.shape
    sr, sc = start
    if 0 <= sr < h and 0 <= sc < w and not mask[sr, sc]:
        return start
    q = deque([(sr, sc)])
    seen = {(sr, sc)}
    while q:
        r, c = q.popleft()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if (nr, nc) in seen:
                    continue
                seen.add((nr, nc))
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if not mask[nr, nc]:
                    return (nr, nc)
                q.append((nr, nc))
    return start


def _a_star_search(cost_map, obstacle_mask, start, goal):
    height, width = cost_map.shape
    if obstacle_mask[start[0], start[1]] or obstacle_mask[goal[0], goal[1]]:
        return None

    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    open_list = [(0.0, 0.0, start)]
    came_from = {}
    g_score = {start: 0.0}

    def heuristic(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    while open_list:
        f, g, current = heapq.heappop(open_list)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        if g > g_score.get(current, float("inf")):
            continue
        for dx, dy in neighbors:
            nr, nc = current[0] + dx, current[1] + dy
            neighbor = (nr, nc)
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if obstacle_mask[nr, nc]:
                continue
            move_dist = float(np.sqrt(dx * dx + dy * dy))
            cell_cost = float(cost_map[nr, nc])
            tentative_g = g + move_dist * (1.0 + cell_cost / 100.0)
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score, tentative_g, neighbor))
    return None


def _visualize(grid, obstacle_mask, obstacle_mask_inflated, path_grid, output_path):
    cost_float = np.clip(grid.astype(np.float32), 0, 100)
    v = (255 - cost_float * 2.0).clip(0, 255).astype(np.uint8)
    height, width = grid.shape
    vis = np.zeros((height, width, 3), dtype=np.uint8)
    free = ~obstacle_mask
    vis[free] = np.stack([v[free], v[free], v[free]], axis=-1)
    vis[obstacle_mask] = (0, 0, 0)
    inflated_only = obstacle_mask_inflated & ~obstacle_mask
    vis[inflated_only] = (40, 40, 40)
    for i in range(len(path_grid) - 1):
        cv2.line(vis, (path_grid[i][1], path_grid[i][0]), (path_grid[i + 1][1], path_grid[i + 1][0]), (0, 255, 0), 1)
    cv2.imwrite(output_path, vis)


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__("global_planner")
        self.declare_parameter("global_map_topic", "/map_server/global_map")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("path_topic", "/global_planner/global_path")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("use_manual_goal", True)
        # Scalar start/goal params (arrays of launch substitutions would be
        # flattened to a single string, so keep these as individual floats).
        self.declare_parameter("start_x", -10.0)
        self.declare_parameter("start_y", -10.0)
        self.declare_parameter("goal_x", 20.0)
        self.declare_parameter("goal_y", 10.0)
        self.declare_parameter("a_star_search_resolution", 1.0)
        self.declare_parameter("global_path_step_size", 5.0)
        self.declare_parameter("obstacle_threshold", 50)
        self.declare_parameter("replan_goal_distance", 2.0)
        self.declare_parameter("visualize", False)

        self.map_msg = None
        self.pose = None
        self.goal = None
        self.last_goal = None

        self.path_pub = self.create_publisher(Path, self.get_parameter("path_topic").value, 1)
        self.create_subscription(OccupancyGrid, self.get_parameter("global_map_topic").value, self._map_cb, 1)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._pose_cb, 10)
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self._goal_cb, 10)

        self.create_timer(1.0, self._tick)
        self.get_logger().info("global_planner ready")

    def _map_cb(self, msg):
        self.map_msg = msg
        # The map server re-publishes the global map every tick as Mode 2/3
        # hazard windows merge in.  Clear the cached goal so the next tick
        # re-plans: otherwise the first straight-line plan (from the fully
        # accessible base map at max_slope_angle=60 deg) is frozen forever and
        # Mode 3 never sees the crater rim as an obstacle to route around.
        self.last_goal = None

    def _pose_cb(self, msg):
        self.pose = msg

    def _goal_cb(self, msg):
        self.goal = msg
        self.last_goal = None  # force replan on a new RViz goal

    def _current_start_goal(self):
        if self.get_parameter("use_manual_goal").value:
            if self.pose is not None:
                start = (self.pose.pose.pose.position.x, self.pose.pose.pose.position.y)
            else:
                start = (float(self.get_parameter("start_x").value),
                         float(self.get_parameter("start_y").value))
            goal = (float(self.get_parameter("goal_x").value),
                    float(self.get_parameter("goal_y").value))
        else:
            if self.pose is None or self.goal is None:
                return None, None
            start = (self.pose.pose.pose.position.x, self.pose.pose.pose.position.y)
            goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        return start, goal

    def _tick(self):
        if self.map_msg is None:
            self.get_logger().info("waiting for global map...", throttle_duration_sec=5.0)
            return
        start, goal = self._current_start_goal()
        if start is None or goal is None:
            self.get_logger().info("waiting for start/goal...", throttle_duration_sec=5.0)
            return
        if self.goal is not None:
            # A real RViz goal overrides the manual one; only replan if it moved.
            goal = (self.goal.pose.position.x, self.goal.pose.position.y)
        if self.last_goal is not None:
            d = float(np.hypot(goal[0] - self.last_goal[0], goal[1] - self.last_goal[1]))
            if d < self.get_parameter("replan_goal_distance").value:
                return
        path = self._plan(self.map_msg, start, goal)
        if path is None:
            self.get_logger().error("global A* failed to find a path")
            return
        self.last_goal = goal
        msg = Path()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            msg.poses.append(make_pose_stamped(msg.header.frame_id, msg.header.stamp, x, y))
        self.path_pub.publish(msg)
        self.get_logger().info(f"published global path with {len(msg.poses)} waypoints")

    def _plan(self, map_msg, start_world, goal_world):
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        grid = _occupancy_grid_to_numpy(map_msg)
        obstacle_threshold = self.get_parameter("obstacle_threshold").value
        obstacle_mask = (grid >= obstacle_threshold) | (grid < 0)
        cost_map = np.clip(grid, 0, 100).astype(np.float32)

        kernel = np.ones((3, 3), np.uint8)
        obstacle_mask_inflated = cv2.dilate(obstacle_mask.astype(np.uint8), kernel, iterations=1).astype(bool)

        factor = max(1, int(round(self.get_parameter("a_star_search_resolution").value / resolution)))
        if factor > 1:
            cost_search, obs_search = _aggregate_cost_map(cost_map, obstacle_mask_inflated, factor)
        else:
            cost_search, obs_search = cost_map, obstacle_mask_inflated

        start_grid = _world_to_grid(start_world[0], start_world[1], origin_x, origin_y, resolution)
        goal_grid = _world_to_grid(goal_world[0], goal_world[1], origin_x, origin_y, resolution)
        scaled_start = (start_grid[0] // factor, start_grid[1] // factor)
        scaled_goal = (goal_grid[0] // factor, goal_grid[1] // factor)

        search_h, search_w = cost_search.shape
        if not (0 <= scaled_start[0] < search_h and 0 <= scaled_start[1] < search_w):
            self.get_logger().error(f"start {scaled_start} outside map {search_h}x{search_w}")
            return None
        if not (0 <= scaled_goal[0] < search_h and 0 <= scaled_goal[1] < search_w):
            self.get_logger().error(f"goal {scaled_goal} outside map {search_h}x{search_w}")
            return None

        # The rover's own cell can land inside the *inflated* obstacle mask when
        # it drives close to a boulder / the rim; snap both endpoints to the
        # nearest free cell so a transient inflation overlap never kills the
        # replan (see _nearest_free_cell).
        scaled_start = _nearest_free_cell(obs_search, scaled_start)
        scaled_goal = _nearest_free_cell(obs_search, scaled_goal)

        path_grid = _a_star_search(cost_search, obs_search, scaled_start, scaled_goal)
        if path_grid is None:
            return None

        if factor > 1:
            path_grid_full = [(p[0] * factor + factor // 2, p[1] * factor + factor // 2) for p in path_grid]
        else:
            path_grid_full = path_grid

        path_world = [_grid_to_world(r, c, origin_x, origin_y, resolution) for (r, c) in path_grid_full]
        path_world[0] = start_world

        if self.get_parameter("visualize").value:
            _visualize(grid, obstacle_mask, obstacle_mask_inflated, path_grid_full, "path.png")

        return _downsample_path_by_step(path_world, float(self.get_parameter("global_path_step_size").value))


def main():
    rclpy.init()
    node = GlobalPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
