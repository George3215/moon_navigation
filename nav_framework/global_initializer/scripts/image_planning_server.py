#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
全局路径规划器：从 map_server 读取 OccupancyGrid，用 A* 做大尺度规划。
输出规划路径 (nav_msgs/Path) 和可视化图片 path.png。
"""
import rospy
import config
import cv2
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Path


# ==================== 工具函数 ====================

def occupancy_grid_to_numpy(msg):
    """将 OccupancyGrid 消息转为 numpy 数组 (height, width)，值域 -1~100"""
    data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return data


def world_to_grid(wx, wy, origin_x, origin_y, resolution):
    """世界坐标 → 栅格索引 (row, col)"""
    col = int(round((wx - origin_x) / resolution))
    row = int(round((wy - origin_y) / resolution))
    return (row, col)


def grid_to_world(row, col, origin_x, origin_y, resolution):
    """栅格索引 → 世界坐标（返回栅格中心）"""
    wx = col * resolution + origin_x + resolution / 2.0
    wy = row * resolution + origin_y + resolution / 2.0
    return (wx, wy)


def downsample_path_by_step(path_world, step_size):
    """按固定步长对路径下采样，保留起点和终点。
    沿路径累计距离，每隔 step_size 米取一个路点。"""
    if len(path_world) <= 2:
        return path_world
    result = [path_world[0]]
    accum = 0.0
    for i in range(1, len(path_world)):
        dx = path_world[i][0] - path_world[i - 1][0]
        dy = path_world[i][1] - path_world[i - 1][1]
        seg_len = np.sqrt(dx * dx + dy * dy)
        accum += seg_len
        if accum >= step_size:
            result.append(path_world[i])
            accum = 0.0
    # 始终保留终点
    if result[-1] != path_world[-1]:
        result.append(path_world[-1])
    return result


def aggregate_cost_map(cost_map, obstacle_mask, scale_factor):
    """将代价图和障碍掩膜降采样到更粗分辨率，块内有任何障碍即为障碍"""
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


# ==================== A* 搜索 ====================

def a_star_search(cost_map, obstacle_mask, start, goal):
    """
    A* 搜索：cost_map 值越大代价越高，obstacle_mask=True 为不可通行。
    start/goal: (row, col)。返回 [(row,col), ...] 或 None。
    """
    height, width = cost_map.shape

    if obstacle_mask[start[0], start[1]]:
        rospy.logwarn("起点位于障碍区域 %s", start)
        return None
    if obstacle_mask[goal[0], goal[1]]:
        rospy.logwarn("终点位于障碍区域 %s", goal)
        return None

    neighbors = [(-1, -1), (-1, 0), (-1, 1),
                 (0, -1),           (0, 1),
                 (1, -1),  (1, 0),  (1, 1)]

    open_list = []
    heapq.heappush(open_list, (0.0, 0.0, start))
    came_from = {}
    g_score = {start: 0.0}

    def heuristic(a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    while open_list:
        f, g, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        if g > g_score.get(current, float('inf')):
            continue

        for dx, dy in neighbors:
            nr, nc = current[0] + dx, current[1] + dy
            neighbor = (nr, nc)
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if obstacle_mask[nr, nc]:
                continue

            move_dist = np.sqrt(dx * dx + dy * dy)
            cell_cost = float(cost_map[nr, nc])
            tentative_g = g + move_dist * (1.0 + cell_cost / 100.0)

            if tentative_g < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_score, tentative_g, neighbor))

    return None


# ==================== 可视化 ====================

def visualize(grid, obstacle_mask, obstacle_mask_inflated, path_grid, height, width):
    """生成并保存可视化图片 path.png"""
    # 底图：可通行区域按代价着灰，障碍涂黑
    cost_float = np.clip(grid.astype(np.float32), 0, 100)
    v = (255 - cost_float * 2.0).clip(0, 255).astype(np.uint8)
    vis = np.zeros((height, width, 3), dtype=np.uint8)
    free = ~obstacle_mask
    vis[free] = np.stack([v[free], v[free], v[free]], axis=-1)
    vis[obstacle_mask] = (0, 0, 0)
    inflated_only = obstacle_mask_inflated & ~obstacle_mask
    vis[inflated_only] = (40, 40, 40)

    # 放大方便查看
    scale = max(1, 500 // max(height, width))
    if scale > 1:
        vis = cv2.resize(vis, (width * scale, height * scale), interpolation=cv2.INTER_NEAREST)
        path_px = [(c * scale + scale // 2, r * scale + scale // 2) for (r, c) in path_grid]
    else:
        path_px = [(c, r) for (r, c) in path_grid]

    # 画路径
    for i in range(len(path_px) - 1):
        cv2.arrowedLine(vis, path_px[i], path_px[i + 1], (0, 255, 0), max(1, scale // 2))
    for p in path_px:
        cv2.circle(vis, p, max(2, scale), (0, 0, 255), -1)
    if path_px:
        cv2.circle(vis, path_px[0], max(3, scale + 1), (255, 0, 0), -1)   # 起点蓝
        cv2.circle(vis, path_px[-1], max(3, scale + 1), (0, 200, 255), -1) # 终点黄

    if config.visual:
        cv2.imshow("Global Path Planning", vis)
        cv2.waitKey(0)
    cv2.imwrite("path.png", vis)
    rospy.loginfo("可视化已保存到 path.png")


# ==================== 主规划流程 ====================

def plan_path(map_msg, start_world, goal_world):
    """从 OccupancyGrid 做 A* 规划，返回世界坐标路径列表"""
    origin_x = map_msg.info.origin.position.x
    origin_y = map_msg.info.origin.position.y
    resolution = map_msg.info.resolution
    height = map_msg.info.height
    width = map_msg.info.width

    # 1. OccupancyGrid → numpy（值域 -1~100，0=free, 100=占据, -1=未知）
    grid = occupancy_grid_to_numpy(map_msg)

    # 2. 障碍掩膜：>= threshold 或未知(-1) 均为障碍
    obstacle_mask = (grid >= config.obstacle_threshold) | (grid < 0)
    cost_map = np.clip(grid, 0, 100).astype(np.float32)

    # 3. 障碍膨胀（安全边距 1 格）
    kernel = np.ones((3, 3), np.uint8)
    obstacle_mask_inflated = cv2.dilate(
        obstacle_mask.astype(np.uint8), kernel, iterations=1
    ).astype(bool)

    # 4. 降采样（如果 a_star_search_resolution > 地图分辨率）
    factor = max(1, int(config.a_star_search_resolution / resolution))
    if factor > 1:
        cost_search, obs_search = aggregate_cost_map(cost_map, obstacle_mask_inflated, factor)
    else:
        cost_search = cost_map
        obs_search = obstacle_mask_inflated

    # 5. 世界坐标 → 栅格索引
    start_grid = world_to_grid(start_world[0], start_world[1], origin_x, origin_y, resolution)
    goal_grid = world_to_grid(goal_world[0], goal_world[1], origin_x, origin_y, resolution)
    scaled_start = (start_grid[0] // factor, start_grid[1] // factor)
    scaled_goal = (goal_grid[0] // factor, goal_grid[1] // factor)
    search_h, search_w = cost_search.shape

    rospy.loginfo("规划: %dx%d 栅格 (factor=%d), start=%s, goal=%s",
                  search_h, search_w, factor, scaled_start, scaled_goal)

    # 边界检查
    if not (0 <= scaled_start[0] < search_h and 0 <= scaled_start[1] < search_w):
        rospy.logerr("起点超出地图范围: %s (map %dx%d)", scaled_start, search_h, search_w)
        return None
    if not (0 <= scaled_goal[0] < search_h and 0 <= scaled_goal[1] < search_w):
        rospy.logerr("终点超出地图范围: %s (map %dx%d)", scaled_goal, search_h, search_w)
        return None

    # 6. A* 搜索
    path_grid = a_star_search(cost_search, obs_search, scaled_start, scaled_goal)
    if path_grid is None:
        rospy.logwarn("A* 未找到路径")
        return None
    rospy.loginfo("路径包含 %d 个路点", len(path_grid))

    # 7. 映射回原始分辨率栅格
    if factor > 1:
        path_grid_full = [(p[0] * factor + factor // 2, p[1] * factor + factor // 2) for p in path_grid]
    else:
        path_grid_full = path_grid

    # 8. 转为世界坐标
    path_world = []
    for (r, c) in path_grid_full:
        wx, wy = grid_to_world(r, c, origin_x, origin_y, resolution)
        path_world.append((wx, wy))

    # 9. 可视化
    visualize(grid, obstacle_mask, obstacle_mask_inflated, path_grid_full, height, width)

    return path_world


# ==================== ROS 回调 ====================

global_map_msg = None
start_ok = False
goal_ok = False
start_pose = None
goal_pose = None


def map_callback(msg):
    global global_map_msg
    global_map_msg = msg
    rospy.loginfo("收到全局地图: %dx%d, resolution=%.2f",
                  msg.info.width, msg.info.height, msg.info.resolution)


def pose_callback(msg):
    global start_pose, start_ok
    start_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    if not start_ok:
        rospy.loginfo("收到起点: (%.2f, %.2f)", start_pose[0], start_pose[1])
    start_ok = True
    


def goal_callback(msg):
    global goal_pose, goal_ok
    goal_pose = (msg.pose.position.x, msg.pose.position.y)
    goal_ok = True
    rospy.loginfo("收到终点: (%.2f, %.2f)", goal_pose[0], goal_pose[1])


# ==================== 入口 ====================

if __name__ == "__main__":
    rospy.init_node("global_planning_node")

    # 订阅全局地图
    globalmap_topic = getattr(config, 'globalmap_topic', '/global_planner/global_map')
    rospy.Subscriber(globalmap_topic, OccupancyGrid, map_callback)
    rospy.loginfo("等待全局地图: %s", globalmap_topic)

    # 路径发布（保持原有话题）
    path_publisher = rospy.Publisher(config.path_topic, Path, queue_size=1)

    if config.manual_start_goal:
        # 手动模式：等地图到达后, 将 config 中的栅格索引转为世界坐标
        while global_map_msg is None and not rospy.is_shutdown():
            rospy.sleep(0.5)
        if rospy.is_shutdown():
            exit(0)
        oi = global_map_msg.info
        start_pose = grid_to_world(config.start[0], config.start[1],
                                   oi.origin.position.x, oi.origin.position.y, oi.resolution)
        goal_pose = grid_to_world(config.goal[0], config.goal[1],
                                  oi.origin.position.x, oi.origin.position.y, oi.resolution)
        start_ok = True
        goal_ok = True
    else:
        rospy.Subscriber(config.pose_topic, PoseWithCovarianceStamped, pose_callback)
        rospy.Subscriber(config.goal_topic, PoseStamped, goal_callback)

    # 等待所有输入就绪
    while not rospy.is_shutdown():
        if global_map_msg is None:
            rospy.loginfo_throttle(5, "等待全局地图...")
        elif not start_ok:
            rospy.loginfo_throttle(5, "等待起点...")
        elif not goal_ok:
            rospy.loginfo_throttle(5, "等待终点...")
        else:
            break
        rospy.sleep(0.5)

    if rospy.is_shutdown():
        exit(0)

    rospy.loginfo("起点: (%.2f, %.2f), 终点: (%.2f, %.2f)",
                  start_pose[0], start_pose[1], goal_pose[0], goal_pose[1])

    # 执行规划
    path_world = plan_path(global_map_msg, start_pose, goal_pose)

    if path_world is None:
        rospy.logerr("规划失败")
        exit(1)

    # 用精确起点替换第一个路点
    path_world[0] = start_pose

    # 按全局路径步长下采样
    step_size = getattr(config, 'global_path_step_size', 5)
    path_world = downsample_path_by_step(path_world, step_size)
    rospy.loginfo("路径下采样 (步长=%.1fm): %d 个路点", step_size, len(path_world))

    # 构建并发布 Path 消息
    path_msg = Path()
    path_msg.header.frame_id = config.output_frame_id
    path_msg.header.stamp = rospy.Time.now()
    for p in path_world:
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        path_msg.poses.append(pose)

    rospy.loginfo("发布路径，共 %d 个路点", len(path_msg.poses))
    for _ in range(3):
        path_publisher.publish(path_msg)
        rospy.sleep(0.5)

    rospy.loginfo("全局路径规划完成")
    