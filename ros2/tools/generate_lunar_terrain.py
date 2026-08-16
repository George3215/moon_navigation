"""Synthesize a *lunar* (Moon) terrain heightmap for the Isaac Sim closed loop.

Replaces ``generate_mixed_terrain.py``'s Martian ridge with a Moon-like scene
built from a central impact crater (rim + bowl) plus regolith noise and a
scattered boulder field, while keeping the same grayscale->metre convention so
the existing ROS 2 navigation stack loads it unchanged.

The corridor at ``y = 0`` (``start (-20,0) -> goal (20,0)``) crosses, in order:

    flat        x <  -12     Mode 1 efficient    (lunar mare)
    rocky       [-12, -6)    Mode 2 safe         (near-vertical boulder field)
    challenging [-6,  +6)    Mode 3 conservative (steep crater rim ~53 deg)
    flat        x >= +6      Mode 1 efficient    (lunar mare)

Physical rollover (real, via Isaac Sim tilt) is the differentiator: the crater
rim face (~53 deg) and the near-vertical boulders (~60 deg) both exceed the Leo
rover's ~47 deg static tip angle, so Mode 1 (straight, no avoidance) and Mode 2
(rock-avoiding but slope-blind) physically tip over, while Mode 3 (slope +
roughness cost map, ``slope_critical_deg = 30``) routes around the crater rim
and survives. This replaces the kinematic version's slope-limit *lookup*
(8/12/40 deg) with genuine physics.

Output:
    docs/picture/terrain/lunar_terrain_hm.png   (8-bit gray, top row = world north)
    isaac/lunar_terrain_hm.npy                  (world-oriented H[row=y, col=x], metres)

The ``.npy`` keeps the exact world-oriented height field so the Isaac Sim
terrain mesh is pixel-identical to the maps the ROS 2 stack derives from the
PNG (feed it as ``H.T`` to ``convert_height_field_to_mesh``).

Usage:  python3 tools/generate_lunar_terrain.py
"""

import os

import cv2
import numpy as np

REPO = "/home/lry/mars_navigation"
OUT = REPO + "/docs/picture/terrain/lunar_terrain_hm.png"
OUT_NPY = REPO + "/isaac/lunar_terrain_hm.npy"

REAL_M = 54.0
RES = 0.2
SIZE = int(round(REAL_M / RES))          # 270
OX = -27.0
OY = -27.0
HPG = 4.820803273566 / 255.0             # metres per gray level (4.82 m full scale)

# --- zone boundaries (world x, metres). Keep in sync with terrain_image_publisher. ---
ROCKY_LO = -12.0
ROCKY_HI = -6.0
RIDGE_LO = -6.0
RIDGE_HI = 6.0

# --- terrain geometry ------------------------------------------------------
BASE_H = 1.5                             # lunar mare plateau height (m)

# Central impact crater (the "challenging" segment).  The STEEP rim ring is the
# capability-differentiating terrain: a Gaussian annulus of height ``H_RIM`` over
# width ``RIM_SIGMA`` whose max slope is ``0.607 * H_RIM / RIM_SIGMA`` (~53 deg
# here), well above the Leo rover's ~47 deg static tip angle.  A gentle central
# bowl keeps it *looking* like a crater without adding a second steep feature.
CRATER_X0, CRATER_Y0 = 0.0, 0.0
CRATER_R_PIT = 3.0                       # gentle inner bowl radius (m)
CRATER_H_PIT = 0.5                       # shallow bowl depth -> ~15 deg (visual only)
CRATER_R_RIM = 5.0                       # steep rim ring radius (m)
CRATER_H_RIM = 2.2                       # rim height above plateau (m) -> ~59 deg face
CRATER_RIM_SIGMA = 0.8                   # rim ring width (m)

# Regolith: low-amplitude, blurred noise -> visual texture only (slope < 2 deg).
REGOLITH_AMP = 0.03
REGOLITH_SIGMA_PX = 1.5

# Boulder field (the "rocky" segment).  Large boulders (0.7-1.2 m tall, sigma
# 0.4-0.5 m -> ~45-60 deg faces) tip Mode 1 on its straight path; Mode 2 detects
# them (rock_height_threshold 0.1 m) and routes around.  The large sigma keeps the
# boulders wider than the 0.2 m grid so the mesh actually resolves a steep face
# (0.15 m-sigma boulders aliased into single-cell spikes the rover just bumped over).
ROCKY_Y = (-4.0, 4.0)
N_BOULDERS = 5
BOULDER_AMP = (0.7, 1.1)
BOULDER_SIGMA = (0.4, 0.5)

# A couple of off-corridor craters for visual realism only (|y| > 10, no effect
# on the y = 0 corridor).
SIDE_CRATERS = [(0.0, -16.0, 3.0, 0.5), (0.0, 16.0, 2.5, 0.4)]


def _cosine_bowl(r, r_pit, h_pit):
    """Smooth bowl: 0 at r >= r_pit, -h_pit at the centre, cos-profile walls."""
    bowl = np.zeros_like(r)
    inside = r < r_pit
    bowl[inside] = -h_pit * 0.5 * (1.0 + np.cos(np.pi * r[inside] / r_pit))
    return bowl


def _rim_ring(r, r_rim, h_rim, sigma):
    return h_rim * np.exp(-((r - r_rim) ** 2) / (2.0 * sigma ** 2))


def build_heightmap():
    """Return a world-oriented height map ``H[row, col]`` in metres.

    ``H[0]`` is world ``y = OY`` (south); ``H[row, col]`` is world
    ``(OX + col*RES, OY + row*RES)``.
    """
    xs = OX + (np.arange(SIZE) + 0.5) * RES
    ys = OY + (np.arange(SIZE) + 0.5) * RES
    X, Y = np.meshgrid(xs, ys)                   # X[col], Y[row] -> H[row, col]

    h = np.full((SIZE, SIZE), BASE_H, dtype=np.float64)

    # 1. central impact crater: bowl + rim ring.
    r = np.hypot(X - CRATER_X0, Y - CRATER_Y0)
    h += _cosine_bowl(r, CRATER_R_PIT, CRATER_H_PIT)
    h += _rim_ring(r, CRATER_R_RIM, CRATER_H_RIM, CRATER_RIM_SIGMA)

    # 2. side craters (visual only, off the corridor).
    for cx, cy, rr, hh in SIDE_CRATERS:
        sr = np.hypot(X - cx, Y - cy)
        h += _cosine_bowl(sr, rr, hh)

    # 3. boulder field.
    rng = np.random.default_rng(7)
    # Force two near-vertical boulders onto the y = 0 line to tip Mode 1's
    # straight path (1.2 m tall, sigma 0.4 m -> ~59 deg mesh face).  Big enough
    # (taller than the rover, face steeper than its 56 deg pitch-tip angle, but
    # under the 63 deg wheel-slip limit at mu=2.0) to backflip a straight-line
    # rover rather than just bump over.
    forced = [(-9.0, 0.0), (-7.5, 0.0)]
    for rx, ry in forced:
        h += 1.2 * np.exp(
            -((X - rx) ** 2 + (Y - ry) ** 2) / (2.0 * (0.4 ** 2))
        )
    for _ in range(N_BOULDERS - len(forced)):
        rx = rng.uniform(ROCKY_LO, ROCKY_HI)
        # Keep random boulders well OFF the y = 0 corridor (|y| > 3 m) so Mode 1's
        # straight path only ever hits the two *controlled* forced boulders AND so
        # Mode 2 has a clear rock-free lane at y ~ [1.3, 3] m to route around the
        # forced boulders and reach the crater rim.  Landing a random boulder at
        # |y| < 3 m blocks that lane (Mode 2 got physically stuck on a boulder at
        # (-9.2, 2.2) in the first run) and a stray next to a forced one stacks
        # into a > 63 deg face that stalls instead of flipping.
        ry = rng.choice([-1.0, 1.0]) * rng.uniform(3.0, ROCKY_Y[1])
        amp = rng.uniform(*BOULDER_AMP)
        sig = rng.uniform(*BOULDER_SIGMA)
        h += amp * np.exp(-(((X - rx) ** 2 + (Y - ry) ** 2) / (2.0 * sig ** 2)))

    # 4. regolith noise (blurred white noise -> gentle lunar texture).
    noise = rng.normal(0.0, REGOLITH_AMP, size=(SIZE, SIZE))
    noise = cv2.GaussianBlur(noise, (0, 0), sigmaX=REGOLITH_SIGMA_PX)
    h += noise

    return h


def _corridor_profile(slope_deg, h):
    """Report the slope along the y = 0 corridor, split by zone."""
    xs = OX + (np.arange(SIZE) + 0.5) * RES
    row = int(round((0.0 - OY) / RES))            # y = 0 row
    profile = slope_deg[row, :]

    def seg(lo, hi):
        mask = (xs >= lo) & (xs < hi)
        return profile[mask]

    print("corridor (y=0) slope profile (deg):")
    print(f"  flat        x<-12 : max {seg(-27, ROCKY_LO).max():.1f}")
    print(f"  rocky      [-12,-6): max {seg(ROCKY_LO, ROCKY_HI).max():.1f}")
    print(f"  challenging[-6, 6): max {seg(RIDGE_LO, RIDGE_HI).max():.1f}")
    print(f"  flat        x>=6 : max {seg(RIDGE_HI, 27).max():.1f}")
    print(f"  corridor height range: {h[row, :].min():.2f} .. {h[row, :].max():.2f} m")


def main():
    h = build_heightmap()

    gy, gx = np.gradient(h, RES)
    slope_deg = np.degrees(np.arctan(np.sqrt(gx ** 2 + gy ** 2)))

    # The ROS loader treats the image's top row as world north (see load_height's
    # np.flip), so save the image as flip(h) so it round-trips to h.
    gray = np.clip(np.rint(h / HPG), 0, 255).astype(np.uint8)
    image = np.flip(gray, axis=0)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    cv2.imwrite(OUT, image)
    print(f"wrote {OUT}  ({SIZE}x{SIZE}, gray range {gray.min()}..{gray.max()})")

    os.makedirs(os.path.dirname(OUT_NPY), exist_ok=True)
    np.save(OUT_NPY, h)
    print(f"wrote {OUT_NPY}  (world-oriented H[row=y, col=x], metres)")

    print(f"height range {h.min():.2f} .. {h.max():.2f} m")
    print(f"slope: max {slope_deg.max():.1f} deg, "
          f"p90 {np.percentile(slope_deg, 90):.1f} deg")
    _corridor_profile(slope_deg, h)


if __name__ == "__main__":
    main()
