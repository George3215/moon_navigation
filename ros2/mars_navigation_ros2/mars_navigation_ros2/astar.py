import heapq
import math

import numpy as np
from scipy.ndimage import maximum_filter, zoom


class AStar:
    def __init__(
        self,
        map_msg,
        start,
        goal,
        search_resolution=1.0,
        diagonal_movement=True,
        heuristic_weight=0.85,
        dilation_radius_meters=0.5,
        traverse_threshold=68,
        hard_obstacle_threshold=88,
        dilation_cost_threshold=75,
        unknown_cost=88,
        terrain_cost_weight=3.0,
    ):
        self.search_resolution = float(search_resolution)
        self.diagonal_movement = bool(diagonal_movement)
        self.heuristic_weight = float(heuristic_weight)
        self.dilation_radius_meters = float(dilation_radius_meters)
        self.traverse_threshold = float(traverse_threshold)
        self.hard_obstacle_threshold = float(hard_obstacle_threshold)
        self.dilation_cost_threshold = float(dilation_cost_threshold)
        self.unknown_cost = float(unknown_cost)
        self.terrain_cost_weight = float(terrain_cost_weight)
        self._set_map_and_endpoints(map_msg, start, goal)

    def _set_map_and_endpoints(self, map_msg, start, goal):
        self.original_resolution = float(map_msg.info.resolution)
        self.origin_x = float(map_msg.info.origin.position.x)
        self.origin_y = float(map_msg.info.origin.position.y)
        original = np.array(map_msg.data, dtype=np.float32).reshape(
            map_msg.info.height, map_msg.info.width
        )
        self.map = self._resample(original)
        self.height, self.width = self.map.shape
        self.start = self._nearest_valid(self.world_to_grid(*start))
        self.goal = self._nearest_valid(self.world_to_grid(*goal))

    def _resample(self, original):
        scale = self.original_resolution / self.search_resolution
        resampled = zoom(original, scale, order=0) if abs(scale - 1.0) > 1e-6 else original
        radius_px = max(0, int(self.dilation_radius_meters / max(self.search_resolution, 1e-6)))
        if radius_px > 0:
            high = np.where(resampled >= self.dilation_cost_threshold, resampled, -np.inf)
            dilated = maximum_filter(high, size=2 * radius_px + 1, mode="nearest")
            resampled = np.maximum(resampled, np.where(np.isfinite(dilated), dilated, resampled))
        resampled = resampled.astype(np.float32)
        resampled[resampled < 0] = self.unknown_cost
        return resampled

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.search_resolution)
        row = int((y - self.origin_y) / self.search_resolution)
        return row, col

    def grid_to_world(self, row, col):
        return (
            col * self.search_resolution + self.origin_x + self.search_resolution / 2.0,
            row * self.search_resolution + self.origin_y + self.search_resolution / 2.0,
        )

    def valid(self, row, col):
        return (
            0 <= row < self.height
            and 0 <= col < self.width
            and self.map[row, col] < self.hard_obstacle_threshold
        )

    def _nearest_valid(self, point, max_radius=30):
        if self.valid(*point):
            return point
        row, col = point
        for radius in range(1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue
                    candidate = (row + dr, col + dc)
                    if self.valid(*candidate):
                        return candidate
        return point

    def _neighbors(self, point):
        base = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.diagonal_movement:
            base += [(-1, -1), (1, -1), (-1, 1), (1, 1)]
        row, col = point
        for dr, dc in base:
            nxt = (row + dr, col + dc)
            if self.valid(*nxt):
                yield nxt, math.sqrt(2.0) if dr and dc else 1.0

    def _heuristic(self, a, b):
        return self.heuristic_weight * math.hypot(a[0] - b[0], a[1] - b[1])

    def _cost(self, point, step):
        terrain = float(self.map[point])
        ratio = min(1.0, max(0.0, terrain / 100.0))
        return step + self.terrain_cost_weight * ratio * ratio

    def search(self):
        start = self.start
        goal = self.goal
        open_heap = [(self._heuristic(start, goal), start)]
        came_from = {}
        g_score = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return [self.grid_to_world(r, c) for r, c in path]

            for neighbor, step in self._neighbors(current):
                tentative = g_score[current] + self._cost(neighbor, step)
                if tentative < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    heapq.heappush(open_heap, (tentative + self._heuristic(neighbor, goal), neighbor))
        return None
