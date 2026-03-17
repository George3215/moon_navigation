import heapq
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import maximum_filter, zoom


class AStar:
    def __init__(
        self,
        map_msg,
        start,
        goal,
        search_resolution=1.0,
        diagonal_movement=False,
        heuristic_weight=1.0,
        dilation_radius_meters=1.0,
        traverse_threshold=65,
        hard_obstacle_threshold=90,
        dilation_cost_threshold=75,
        unknown_cost=90,
        high_cost_penalty_gain=2.0,
        terrain_cost_weight=5.0,
        turn_penalty_gain=0.35,
        risk_threshold=50,
        risk_penalty_gain=1.8,
    ):
        self._configure(
            search_resolution,
            diagonal_movement,
            heuristic_weight,
            dilation_radius_meters,
            traverse_threshold,
            hard_obstacle_threshold,
            dilation_cost_threshold,
            unknown_cost,
            high_cost_penalty_gain,
            terrain_cost_weight,
            turn_penalty_gain,
            risk_threshold,
            risk_penalty_gain,
        )
        self._set_map_and_endpoints(map_msg, start, goal)

    def re_init_(
        self,
        map_msg,
        start,
        goal,
        search_resolution=1.0,
        diagonal_movement=False,
        heuristic_weight=1.0,
        dilation_radius_meters=1.0,
        traverse_threshold=65,
        hard_obstacle_threshold=90,
        dilation_cost_threshold=75,
        unknown_cost=90,
        high_cost_penalty_gain=2.0,
        terrain_cost_weight=6.0,
        turn_penalty_gain=0.35,
        risk_threshold=50,
        risk_penalty_gain=1.8,
    ):
        self._configure(
            search_resolution,
            diagonal_movement,
            heuristic_weight,
            dilation_radius_meters,
            traverse_threshold,
            hard_obstacle_threshold,
            dilation_cost_threshold,
            unknown_cost,
            high_cost_penalty_gain,
            terrain_cost_weight,
            turn_penalty_gain,
            risk_threshold,
            risk_penalty_gain,
        )
        self._set_map_and_endpoints(map_msg, start, goal)

    def _configure(
        self,
        search_resolution,
        diagonal_movement,
        heuristic_weight,
        dilation_radius_meters,
        traverse_threshold,
        hard_obstacle_threshold,
        dilation_cost_threshold,
        unknown_cost,
        high_cost_penalty_gain,
        terrain_cost_weight,
        turn_penalty_gain,
        risk_threshold,
        risk_penalty_gain,
    ):
        self.search_resolution = float(search_resolution)
        self.diagonal_movement = bool(diagonal_movement)
        self.heuristic_weight_base = float(heuristic_weight)

        self.dilation_radius_meters = float(dilation_radius_meters)
        self.traverse_threshold = float(traverse_threshold)
        self.hard_obstacle_threshold = float(max(hard_obstacle_threshold, traverse_threshold + 1))
        self.dilation_cost_threshold = float(dilation_cost_threshold)
        self.unknown_cost = float(unknown_cost)

        self.high_cost_penalty_gain = float(high_cost_penalty_gain)
        self.terrain_cost_weight = float(terrain_cost_weight)
        self.turn_penalty_gain = float(turn_penalty_gain)
        self.risk_threshold = float(min(risk_threshold, traverse_threshold))
        self.risk_penalty_gain = float(risk_penalty_gain)

        if self.diagonal_movement:
            self.neighbors = [
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (-1, -1),
                (1, -1),
                (-1, 1),
                (1, 1),
            ]
        else:
            self.neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def _set_map_and_endpoints(self, map_msg, start, goal):
        self.original_map = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        self.original_resolution = map_msg.info.resolution
        self.origin_x = map_msg.info.origin.position.x
        self.origin_y = map_msg.info.origin.position.y

        self.map = self.resample_map(
            self.original_map,
            self.original_resolution,
            self.search_resolution,
            self.dilation_radius_meters,
        )
        self.height, self.width = self.map.shape

        self.start = self._find_nearest_valid(self.world_to_grid(start[0], start[1]))
        self.goal = self._find_nearest_valid(self.world_to_grid(goal[0], goal[1]))

        traversable = self.map[self.map < self.traverse_threshold]
        mean_cost_scale = 1.0 + (np.mean(traversable) / 100.0 if traversable.size else 0.0)
        self.heuristic_weight = self.heuristic_weight_base * mean_cost_scale

    def _find_nearest_valid(self, point, max_radius=20):
        row, col = point
        if self.valid(row, col):
            return point
        for r in range(1, max_radius + 1):
            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    if abs(dr) != r and abs(dc) != r:
                        continue
                    nr, nc = row + dr, col + dc
                    if self.valid(nr, nc):
                        return (nr, nc)
        return point

    def resample_map(self, original_map, original_res, search_res, dilation_radius_meters):
        scale = original_res / search_res
        resampled = zoom(original_map, scale, order=0)
        resampled = self.map_dilate(
            resampled,
            cost_threshold=self.dilation_cost_threshold,
            dilation_radius_meters=dilation_radius_meters,
        )
        resampled = resampled.astype(np.float32)
        resampled[resampled == -1] = self.unknown_cost
        return resampled

    def map_dilate(self, map_data, cost_threshold=80, dilation_radius_meters=1.0):
        radius_px = max(0, int(dilation_radius_meters / max(self.search_resolution, 1e-6)))
        if radius_px == 0:
            return np.array(map_data, copy=True)

        arr = np.array(map_data, dtype=np.float32, copy=True)
        high_cost = np.where(arr >= cost_threshold, arr, -np.inf)
        kernel_size = 2 * radius_px + 1
        dilated = maximum_filter(high_cost, size=kernel_size, mode="nearest")
        return np.maximum(arr, np.where(np.isfinite(dilated), dilated, arr))

    def world_to_grid(self, world_x, world_y):
        col = int((world_x - self.origin_x) / self.search_resolution)
        row = int((world_y - self.origin_y) / self.search_resolution)
        return row, col

    def grid_to_world(self, row, col):
        x = col * self.search_resolution + self.origin_x
        y = row * self.search_resolution + self.origin_y
        return x, y

    def heuristic(self, a, b):
        return self.heuristic_weight * np.linalg.norm(np.array(a) - np.array(b))

    def valid(self, row, col):
        return (
            0 <= row < self.height
            and 0 <= col < self.width
            and self.map[row, col] < self.hard_obstacle_threshold
        )

    def get_cost(self, row, col, parent_row, parent_col):
        step_cost = math.sqrt(2) if (row != parent_row and col != parent_col) else 1.0

        terrain = float(self.map[row, col])
        if terrain >= self.hard_obstacle_threshold:
            return float("inf")

        # 平滑的地形成本，使用线性+二次混合，避免过度惩罚
        terrain_ratio = np.clip(terrain / 100.0, 0.0, 1.0)
        terrain_cost = self.terrain_cost_weight * (0.5 * terrain_ratio + 0.5 * terrain_ratio ** 2)

        # 风险成本：可通行但危险的区域
        risk_cost = 0.0
        if terrain >= self.risk_threshold:
            span = max(1.0, self.traverse_threshold - self.risk_threshold)
            risk_ratio = np.clip((terrain - self.risk_threshold) / span, 0.0, 1.0)
            risk_cost = self.risk_penalty_gain * risk_ratio

        # 高成本惩罚：在遍历困难区域时加权
        high_cost = 0.0
        if terrain >= self.traverse_threshold:
            span = max(1.0, self.hard_obstacle_threshold - self.traverse_threshold)
            exceed_ratio = np.clip((terrain - self.traverse_threshold) / span, 0.0, 1.0)
            # 使用线性惩罚（而非二次），避免过度膨胀
            high_cost = self.high_cost_penalty_gain * exceed_ratio

        return step_cost + terrain_cost + risk_cost + high_cost

    def visualize_map(self, current_path=None):
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(self.map, cmap="gray", origin="lower")

        sr, sc = self.start
        gr, gc = self.goal
        ax.scatter(sc, sr, color="green", s=100, label="Start")
        ax.scatter(gc, gr, color="red", s=100, label="Goal")

        if current_path:
            px = [p[1] for p in current_path]
            py = [p[0] for p in current_path]
            ax.scatter(px, py, color="blue", s=20, label="Path Points")

        ax.legend()
        plt.title("A* Search Map Visualization")
        plt.xlabel("X (grid index)")
        plt.ylabel("Y (grid index)")
        plt.show()

    def a_star_search(self):
        start = (self.start[0], self.start[1])
        goal = (self.goal[0], self.goal[1])

        open_heap = [(self.heuristic(start, goal), start)]
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}
        closed = set()

        while open_heap:
            current_f, current = heapq.heappop(open_heap)
            if current_f > f_score.get(current, float("inf")):
                continue

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return [self.grid_to_world(r, c) for r, c in path]

            if current in closed:
                continue
            closed.add(current)

            for dr, dc in self.neighbors:
                nbr = (current[0] + dr, current[1] + dc)
                if nbr in closed or not self.valid(nbr[0], nbr[1]):
                    continue

                tentative_g = g_score[current] + self.get_cost(nbr[0], nbr[1], current[0], current[1])

                if current in came_from:
                    prev = came_from[current]
                    prev_vec = (current[0] - prev[0], current[1] - prev[1])
                    new_vec = (nbr[0] - current[0], nbr[1] - current[1])
                    if prev_vec != new_vec:
                        tentative_g += self.turn_penalty_gain

                if tentative_g >= g_score.get(nbr, float("inf")):
                    continue

                came_from[nbr] = current
                g_score[nbr] = tentative_g
                nbr_f = tentative_g + self.heuristic(nbr, goal)
                f_score[nbr] = nbr_f
                heapq.heappush(open_heap, (nbr_f, nbr))

        return None
