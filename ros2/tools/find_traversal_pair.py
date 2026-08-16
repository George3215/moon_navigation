"""Find a start/goal pair with a clean corridor in all three modes.

Uses the actual map pipelines (base / rock / cost), plus the global planner's
real reachability semantics (dilate + 5x5 aggregation + obstacle_threshold=50).
Prints candidate (start, goal) pairs and the per-mode straight-line corridor
cost.
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__) + "/../mars_navigation_ros2/mars_navigation_ros2")
from elevation_synthesis import compute_rock_map, compute_traversability_cost, load_height

REPO = "/home/lry/mars_navigation"
IMG = REPO + "/nav_framework/global_initializer/scripts/images/marsyard2022_terrain_hm.jpg"
REAL_M = 54.0
RES = 0.2
OX = -27.0
OY = -27.0
HPG = 4.820803273566 / 255.0
MAX_SLOPE = 2.0


def base_map(height):
    h, w = height.shape
    acc = np.full((h, w), 255, np.uint8)
    nb = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    ratio = np.tan(np.radians(MAX_SLOPE))
    for i in range(h):
        for j in range(w):
            cur = float(height[i, j])
            for dx, dy in nb:
                ni, nj = i + dx, j + dy
                if 0 <= ni < h and 0 <= nj < w:
                    horiz = RES * (1 if dx == 0 or dy == 0 else np.sqrt(2))
                    if abs(cur - float(height[ni, nj])) / horiz <= ratio:
                        acc[i, j] = 0
                        break
    num, labels = cv2.connectedComponents(acc)
    out = np.zeros_like(acc)
    for lbl in range(1, num):
        if int(np.sum(labels == lbl)) >= 2:
            out[labels == lbl] = 255
    dist = cv2.distanceTransform(255 - out, cv2.DIST_L2, 5)
    connected = ((dist < 1).astype(np.uint8)) * 255
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    morphed = cv2.erode(cv2.dilate(connected, k), k)
    return np.flip(cv2.normalize(morphed, None, 0, 100, cv2.NORM_MINMAX, cv2.CV_8U), 0).astype(np.float32)


def world_to_rc(x, y):
    return int(round((y - OY) / RES)), int(round((x - OX) / RES))


def rc_to_world(r, c):
    return c * RES + OX + RES / 2, r * RES + OY + RES / 2


def global_free(mask, x0, y0, x1, y1):
    """A* reachability using global planner semantics (dilate + 5x5 + thresh 50)."""
    obstacle = (mask >= 50)
    obstacle = cv2.dilate(obstacle.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    h, w = obstacle.shape
    f = 5
    nh, nw = h // f, w // f
    agg = np.zeros((nh, nw), bool)
    for i in range(nh):
        for j in range(nw):
            agg[i, j] = np.any(obstacle[i * f:(i + 1) * f, j * f:(j + 1) * f])
    sr, sc = world_to_rc(x0, y0)
    gr, gc = world_to_rc(x1, y1)
    s = (sr // f, sc // f)
    g = (gr // f, gc // f)
    if not (0 <= s[0] < nh and 0 <= s[1] < nw and 0 <= g[0] < nh and 0 <= g[1] < nw):
        return False
    if agg[s] or agg[g]:
        return False
    from heapq import heappush, heappop
    open_list = [(0, s)]
    gscore = {s: 0.0}
    came = {}
    while open_list:
        _, cur = heappop(open_list)
        if cur == g:
            return True
        r, c = cur
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if 0 <= nr < nh and 0 <= nc < nw and not agg[nr, nc]:
                    t = gscore[cur] + np.sqrt(dr * dr + dc * dc)
                    if t < gscore.get((nr, nc), float("inf")):
                        gscore[(nr, nc)] = t
                        came[(nr, nc)] = cur
                        heappush(open_list, (t + np.hypot(nr - g[0], nc - g[1]), (nr, nc)))
    return False


def main():
    height = load_height(IMG, REAL_M, RES, HPG)
    base = base_map(height)
    rock, obstacle = compute_rock_map(height)
    _, _, cost = compute_traversability_cost(height, RES)

    free1 = base < 50
    free2 = (base < 50) & (obstacle < 50)
    free3 = (base < 50) & (cost < 50)

    # Report cost at a few known points.
    for (x, y) in [(-18.5, -22.5), (-6, -10), (26.5, 12.5), (0, 0), (-15, 5), (15, -5)]:
        r, c = world_to_rc(x, y)
        if 0 <= r < 270 and 0 <= c < 270:
            print(f"({x:+.1f},{y:+.1f}) base={base[r,c]:.0f} rock={obstacle[r,c]:.0f} "
                  f"cost={cost[r,c]:.0f} | free1={free1[r,c]} free2={free2[r,c]} free3={free3[r,c]}")

    # Sweep candidate start/goal pairs in the flat region, require global A* in all 3.
    print("\nCandidate pairs (global A* reachable in mode1+mode2+mode3):")
    found = 0
    for sx, sy in [(-15, -15), (-10, -10), (-5, -15), (-15, -5), (-5, -5), (0, 0), (-10, 0)]:
        for gx, gy in [(10, 10), (15, 5), (5, 15), (20, 10), (15, 15), (10, 15), (20, 0), (0, 20)]:
            d = np.hypot(gx - sx, gy - sy)
            if d < 20:
                continue
            a1 = global_free(base, sx, sy, gx, gy)
            a2 = global_free(np.where(obstacle >= 50, 100, base * 0 + 0), sx, sy, gx, gy)
            # mode2 free mask combines base+rock
            m2mask = np.where((base >= 50) | (obstacle >= 50), 100, 0).astype(np.float32)
            a2 = global_free(m2mask, sx, sy, gx, gy)
            m3mask = np.where((base >= 50) | (cost >= 50), 100, 0).astype(np.float32)
            a3 = global_free(m3mask, sx, sy, gx, gy)
            if a1 and a2 and a3:
                gr, gc = world_to_rc(gx, gy)
                print(f"  start({sx:+.1f},{sy:+.1f}) -> goal({gx:+.1f},{gy:+.1f}) d={d:.0f}m "
                      f"cost@goal={cost[gr,gc]:.0f}")
                found += 1
    if not found:
        print("  (none)")

    # Largest connected low-cost blob in mode3, to guide manual choice.
    big = (free1 & free2 & free3).astype(np.uint8)
    num, labels = cv2.connectedComponents(big, connectivity=8)
    sizes = [(int(np.sum(labels == l)), l) for l in range(1, num)]
    sizes.sort(reverse=True)
    print("\nLargest all-mode-free connected components (cells):")
    for sz, l in sizes[:5]:
        ys, xs = np.where(labels == l)
        r0, r1 = ys.min(), ys.max()
        c0, c1 = xs.min(), xs.max()
        x0, y0 = rc_to_world(r0, c0)
        x1, y1 = rc_to_world(r1, c1)
        print(f"  component {l}: {sz} cells, bbox world x[{x0:+.1f},{x1:+.1f}] y[{y0:+.1f},{y1:+.1f}]")


if __name__ == "__main__":
    main()
