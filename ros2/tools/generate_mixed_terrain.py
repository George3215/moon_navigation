"""Synthesize a *mixed* terrain heightmap for a meaningful mode-differentiation run.

The real Mars Yard heightmap has no region where Mode 1's 2-degree base map
rejects terrain that Mode 3's cost map accepts (``base>=50 & cost<50`` is 0.0%,
and the cost map is *more* restrictive than the base map).  So every mode trivially
succeeds on the flat corridor, which robs the paper of its point.

This script synthesises a 54 m heightmap with clear, position-separated segments
along a ``start -> goal`` corridor at ``y = 0``:

    flat        x <  -6       Mode 1 efficient   (2.0 m/s)
    rocky       [-12, -6)     Mode 2 safe        (0.8 m/s, scattered rocks)
    challenging [-6,  +6)     Mode 3 conservative (0.5 m/s, smooth ~22 deg ridge)
    flat        x >= +6       Mode 1 efficient   (2.0 m/s)

The ridge is a full-width Gaussian wall (constant in ``y``): its ~22 deg slopes
are far above Mode 1's 2 deg flat-terrain assumption but below Mode 3's 30 deg
critical slope, so the cost map rates it ~50 (challenging, not impassable) while
it stays smooth enough to read as "challenging" to the VLM classifier.

The zone boundaries are mirrored by ``terrain_image_publisher`` so the VLM sees
the terrain that actually matches the rover's current position.

Output: ``docs/picture/terrain/mixed_terrain_hm.png`` (same grayscale->metre
convention as the original ``marsyard2022_terrain_hm.jpg``).

Usage:  python3 tools/generate_mixed_terrain.py
"""

import os

import cv2
import numpy as np

REPO = "/home/lry/mars_navigation"
OUT = REPO + "/docs/picture/terrain/mixed_terrain_hm.png"

REAL_M = 54.0
RES = 0.2
SIZE = int(round(REAL_M / RES))          # 270
OX = -27.0
OY = -27.0
HPG = 4.820803273566 / 255.0             # metres per gray level

# --- zone boundaries (world x, metres). Keep in sync with terrain_image_publisher.
ROCKY_LO = -12.0
ROCKY_HI = -6.0
RIDGE_LO = -6.0
RIDGE_HI = 6.0

# --- terrain geometry ------------------------------------------------------
BASE_H = 1.8                             # flat plateau height (m)
RIDGE_AMP = 1.17                         # ridge peak height above plateau (m)
RIDGE_SIGMA = 2.5                        # ridge half-width (m) -> ~15.8 deg max slope (cost ~73)
RIDGE_X0 = 0.0
ROCKY_Y = (-4.0, 4.0)                    # rock-field y extent (m) -- band around the corridor


def build_heightmap():
    """Return a world-oriented height map ``H[row, col]`` in metres.

    ``H[0]`` is world ``y = OY`` (south); ``H[row, col]`` is world
    ``(OX + col*RES, OY + row*RES)``.
    """
    xs = OX + (np.arange(SIZE) + 0.5) * RES
    ys = OY + (np.arange(SIZE) + 0.5) * RES
    X, Y = np.meshgrid(xs, ys)                   # X[col], Y[row] -> H[row, col]

    h = np.full((SIZE, SIZE), BASE_H, dtype=np.float64)

    # 1. smooth full-width ridge (a wall along y, Gaussian profile along x).
    h += RIDGE_AMP * np.exp(-((X - RIDGE_X0) ** 2) / (2.0 * RIDGE_SIGMA ** 2))

    # 2. scattered rocks.  They are deliberately *gentle* (whole-field max slope
    #    ~10 deg, kept below Mode 2's 12 deg rollover limit): the rock field is
    #    textured enough to read as "rocky", but Mode 2 crosses it safely and
    #    only fails on the ridge, which is the capability-differentiating
    #    terrain.  (Steeper/denser rocks would trip Mode 2's rollover limit on
    #    overlapping rock flanks before it ever reaches the ridge, blurring the
    #    Mode-2-vs-Mode-3 distinction.)
    rng = np.random.default_rng(7)
    n_rocks = 6
    rock_amp = 0.09
    rock_sigma = 0.5
    for _ in range(n_rocks):
        rx = rng.uniform(ROCKY_LO, ROCKY_HI)
        ry = rng.uniform(*ROCKY_Y)
        amp = rock_amp * rng.uniform(0.85, 1.15)
        sig = rock_sigma * rng.uniform(0.9, 1.1)
        h += amp * np.exp(-(((X - rx) ** 2 + (Y - ry) ** 2) / (2.0 * sig ** 2)))

    return h


def main():
    h = build_heightmap()

    # The ROS loader treats the image's top row as world north.  ``load_height``
    # returns ``flip(gray)``; we want that to equal the world-oriented ``h``, so
    # save the image as ``flip(h)``.
    gray = np.clip(np.rint(h / HPG), 0, 255).astype(np.uint8)
    image = np.flip(gray, axis=0)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    cv2.imwrite(OUT, image)
    print(f"wrote {OUT}  ({SIZE}x{SIZE}, gray range {gray.min()}..{gray.max()})")

    gy, gx = np.gradient(h, RES)
    slope_deg = np.degrees(np.arctan(np.sqrt(gx ** 2 + gy ** 2)))
    print(f"height range {h.min():.2f}..{h.max():.2f} m")
    print(f"slope: max {slope_deg.max():.1f} deg, "
          f"ridge p90 {np.percentile(slope_deg, 90):.1f} deg")


if __name__ == "__main__":
    main()
