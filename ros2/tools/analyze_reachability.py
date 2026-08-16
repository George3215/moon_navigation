"""Offline reachability analysis: find start/goal pairs traversable in all 3 modes.

Replicates the exact map pipelines of image_to_map (Mode 1 base map) and
elevation_synthesis (Mode 2 rock map, Mode 3 cost map), then checks which cells
are reachable from a candidate start under each mode's obstacle threshold.

Usage:  python3 tools/analyze_reachability.py
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__) + "/../mars_navigation_ros2")
sys.path.insert(0, os.path.dirname(__file__) + "/../mars_navigation_ros2/mars_navigation_ros2")

from elevation_synthesis import compute_rock_map, compute_traversability_cost, load_height

REPO = "/home/lry/mars_navigation"
IMG = REPO + "/nav_framework/global_initializer/scripts/images/marsyard2022_terrain_hm.jpg"

REAL_M = 54.0
RES = 0.2
ORIGIN_X = -27.0
ORIGIN_Y = -27.0
HEIGHT_PER_GRAY = 4.820803273566 / 255.0
MAX_SLOPE_ANGLE = 2.0  # image_to_map default


def base_map(height):
    """Replicate image_to_map.py: accessibility -> cleanup -> 0..100."""
    h, w = height.shape
    acc = np.full((h, w), 255, dtype=np.uint8)
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    max_slope_ratio = np.tan(np.radians(MAX_SLOPE_ANGLE))
    for i in range(h):
        for j in range(w):
            cur = float(height[i, j])
            for dx, dy in neighbors:
                ni, nj = i + dx, j + dy
                if 0 <= ni < h and 0 <= nj < w:
                    horizontal = RES * (1 if dx == 0 or dy == 0 else np.sqrt(2))
                    if abs(cur - float(height[ni, nj])) / horizontal <= max_slope_ratio:
                        acc[i, j] = 0
                        break
    # filter small connected regions (min_size=2)
    num, labels = cv2.connectedComponents(acc)
    out = np.zeros_like(acc)
    for lbl in range(1, num):
        region = labels == lbl
        if int(np.sum(region)) >= 2:
            out[region] = 255
    # distance transform cleanup
    dist = cv2.distanceTransform(255 - out, cv2.DIST_L2, 5)
    connected = ((dist < 1).astype(np.uint8)) * 255
    # morphology close
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    morphed = cv2.erode(cv2.dilate(connected, kernel, iterations=1), kernel, iterations=1)
    normalized = cv2.normalize(morphed, None, 0, 100, cv2.NORM_MINMAX, cv2.CV_8U)
    return np.flip(normalized, axis=0).astype(np.float32)


def flood_reach(free, start_rc):
    """Return a boolean mask of cells reachable from start through free cells."""
    h, w = free.shape
    reach = np.zeros_like(free)
    stack = [start_rc]
    reach[start_rc] = True
    while stack:
        r, c = stack.pop()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and free[nr, nc] and not reach[nr, nc]:
                    reach[nr, nc] = True
                    stack.append((nr, nc))
    return reach


def main():
    height = load_height(IMG, REAL_M, RES, HEIGHT_PER_GRAY)
    print("height", height.shape, "range", height.min(), height.max())

    base = base_map(height)
    rock, obstacle = compute_rock_map(height)
    _, _, cost = compute_traversability_cost(height, RES)

    print("base:  free(<50) frac =", round(float((base < 50).mean()), 4),
          " obstacle(>=50) frac =", round(float((base >= 50).mean()), 4))
    print("rock:  obstacle frac =", round(float((obstacle >= 50).mean()), 4))
    print("cost:  <50 frac =", round(float((cost < 50).mean()), 4),
          " 50-88 frac =", round(float(((cost >= 50) & (cost < 88)).mean()), 4),
          " >=88 frac =", round(float((cost >= 88).mean()), 4))

    # Mode-specific free masks (global planner threshold=50).
    free1 = (base < 50)
    free2 = (base < 50) & (obstacle < 50)
    free3 = (base < 50) & (cost < 50)

    # Candidate start/goal grid: sweep world coords on a coarse grid.
    def rc_from_world(x, y):
        return int(round((y - ORIGIN_Y) / RES)), int(round((x - ORIGIN_X) / RES))

    # Find a start that is free in all modes (bottom-left region historically).
    candidates = []
    for x in np.arange(-22, 23, 2):
        for y in np.arange(-22, 23, 2):
            r, c = rc_from_world(x, y)
            if not (0 <= r < height.shape[0] and 0 <= c < height.shape[1]):
                continue
            if free1[r, c] and free2[r, c] and free3[r, c]:
                candidates.append((x, y, r, c))

    print("cells free in ALL modes (coarse 2m grid):", len(candidates))

    # For a few candidate starts, measure the reachable area in each mode.
    starts = [(-18.5, -22.5), (-16, -20), (-10, -10), (0, 0)]
    for sx, sy in starts:
        r, c = rc_from_world(sx, sy)
        if not (0 <= r < height.shape[0] and 0 <= c < height.shape[1]):
            print(f"start ({sx},{sy}) out of bounds")
            continue
        r1 = flood_reach(free1, (r, c)).sum()
        r2 = flood_reach(free2, (r, c)).sum()
        r3 = flood_reach(free3, (r, c)).sum()
        print(f"start ({sx:+.1f},{sy:+.1f}) rc=({r},{c}) "
              f"| base free={free1[r,c]} rock free={free2[r,c]} cost free={free3[r,c]} "
              f"cost={cost[r,c]:.0f} | reach cells: mode1={r1} mode2={r2} mode3={r3}")

    # Now: for the historical start, find goals reachable in ALL three modes.
    r0, c0 = rc_from_world(-18.5, -22.5)
    reach3 = flood_reach(free3, (r0, c0))
    # Among mode3-reachable cells, find ones far from start and also free in m1/m2.
    rows, cols = np.where(reach3 & free1 & free2)
    print("mode3-reachable & all-free cells:", rows.size)
    best = None
    for r, c in zip(rows, cols):
        x = c * RES + ORIGIN_X + RES / 2
        y = r * RES + ORIGIN_Y + RES / 2
        d = np.hypot(x - (-18.5), y - (-22.5))
        if d > 25 and cost[r, c] < 50 and obstacle[r, c] < 50 and base[r, c] < 50:
            if best is None or d > best[3]:
                best = (x, y, cost[r, c], d)
    if best:
        print(f"FARTHEST reachable goal from start (-18.5,-22.5): "
              f"({best[0]:+.1f},{best[1]:+.1f}) dist={best[3]:.1f}m cost={best[2]:.0f}")
    else:
        print("no goal >25m reachable in all modes from (-18.5,-22.5)")


if __name__ == "__main__":
    main()
