"""Pure-NumPy/cv2 elevation synthesis (faithful port of the paper's perception).

This module centralises the terrain reasoning that the original stack spreads
across two ROS 1 nodes, so it can be reused by ``hazard_mapper`` (the live
ROS 2 node) and by offline figure/artifact generation without duplicating the
math.

Mode 2 (rocky)   -- port of ``perception/hazard_detection``
------------------------------------------------------------
``hazard_detection`` RANSAC-removes the ground plane from a depth point cloud
and flags points 0.1-0.3 m *above* ground as obstacles (rocks).  We reproduce
that as a high-pass residual on the heightmap: a Gaussian low-pass gives the
local ground surface, and the residual ``height - ground`` above a threshold is
the rock/obstacle field.

Mode 3 (challenging) -- port of
``perception/elevation_mapping_cupy/.../plugins/custom_traversability_cost.py``
-------------------------------------------------------------------------------
The CuPy plugin fuses a Sobel slope term with a local-std-dev roughness term:

    cost = slope_weight * (slope / slope_critical)^slope_exp
         + roughness_weight * (roughness / roughness_critical)^rough_exp
         + danger_boost * max(slope_danger, roughness_danger)^2
    then Gaussian smooth, clip to [0,1], and zero anything below a cost
    threshold.

All default constants below match the plugin's constructor defaults.
"""

import cv2
import numpy as np


def load_height(image_path, real_meter, resolution, height_per_gray):
    """Load a grayscale terrain image and convert it to a metre-scaled heightmap.

    ``height`` is returned flipped along axis 0 so that row 0 == world y == the
    map origin, matching ``image_to_map``.
    """
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(image_path)
    size = int(round(real_meter / resolution))
    resized = cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY).astype(np.float32)
    height = gray * float(height_per_gray)
    return np.flip(height, axis=0)


def compute_rock_map(height, ground_sigma=2.0, rock_height_threshold=0.1):
    """Mode 2 rock/obstacle map.

    Returns ``(rock_residual, obstacle)`` where ``obstacle`` is a 0/100 binary
    map (100 == rock, 0 == traversable).  ``ground_sigma`` (pixels) controls how
    smooth the local ground plane is; ``rock_height_threshold`` (metres) is the
    residual height that counts as a rock.
    """
    ground = cv2.GaussianBlur(
        height, (0, 0), sigmaX=ground_sigma, borderType=cv2.BORDER_REPLICATE
    )
    rock = height - ground
    obstacle = np.where(rock > rock_height_threshold, 100.0, 0.0).astype(np.float32)
    return rock, obstacle


def compute_traversability_cost(
    height,
    resolution,
    slope_weight=0.7,
    roughness_weight=0.3,
    slope_critical_deg=30.0,
    roughness_critical=0.08,
    slope_exponent=1.6,
    roughness_exponent=1.4,
    danger_slope_deg=20.0,
    danger_roughness=0.05,
    danger_boost=0.45,
    cost_threshold=0.05,
    cost_sigma=2.0,
    pre_sobel_sigma=0.8,
    roughness_kernel=5,
):
    """Mode 3 traversability cost map (0..100), ported from the CuPy plugin.

    Returns ``(slope_deg, roughness, cost100)``.
    """
    h = height.astype(np.float32)

    # 1. slope (Sobel gradient -> physical slope angle in degrees).
    h_grad = (
        cv2.GaussianBlur(h, (0, 0), sigmaX=pre_sobel_sigma, borderType=cv2.BORDER_REPLICATE)
        if pre_sobel_sigma > 0
        else h
    )
    gx = cv2.Sobel(h_grad, cv2.CV_32F, 1, 0, ksize=3, borderType=cv2.BORDER_REPLICATE) / (
        8.0 * resolution
    )
    gy = cv2.Sobel(h_grad, cv2.CV_32F, 0, 1, ksize=3, borderType=cv2.BORDER_REPLICATE) / (
        8.0 * resolution
    )
    slope_deg = np.degrees(np.arctan(np.sqrt(gx * gx + gy * gy)))

    # 2. roughness (local std-dev of height over a kernel box).
    k = int(roughness_kernel)
    mean_h = cv2.boxFilter(h, cv2.CV_32F, (k, k), normalize=True, borderType=cv2.BORDER_REPLICATE)
    mean_h2 = cv2.boxFilter(h * h, cv2.CV_32F, (k, k), normalize=True, borderType=cv2.BORDER_REPLICATE)
    variance = np.maximum(mean_h2 - mean_h * mean_h, 0.0)
    roughness = np.sqrt(variance)

    # 3. weighted fusion (slope + roughness), normalised to [0, 1].
    slope_norm = np.clip(slope_deg / max(1e-3, slope_critical_deg), 0.0, 1.0)
    roughness_norm = np.clip(roughness / max(1e-6, roughness_critical), 0.0, 1.0)
    cost = (
        slope_weight * np.power(slope_norm, slope_exponent)
        + roughness_weight * np.power(roughness_norm, roughness_exponent)
    )

    # 3.1 danger boost: moderate slope/roughness is penalised early.
    slope_danger = np.clip(
        (slope_deg - danger_slope_deg) / max(1e-3, slope_critical_deg - danger_slope_deg),
        0.0,
        1.0,
    )
    rough_danger = np.clip(
        (roughness - danger_roughness) / max(1e-6, roughness_critical - danger_roughness),
        0.0,
        1.0,
    )
    danger = np.maximum(slope_danger, rough_danger)
    cost = np.clip(cost + danger_boost * np.power(danger, 2.0), 0.0, 1.0)

    # 4. Gaussian smooth then threshold the low-cost speckle away.
    if cost_sigma > 0:
        cost = cv2.GaussianBlur(cost, (0, 0), sigmaX=cost_sigma, borderType=cv2.BORDER_REPLICATE)
    cost = np.clip(cost, 0.0, 1.0)
    cost = np.where(cost < cost_threshold, 0.0, cost)

    return slope_deg, roughness, cost * 100.0
