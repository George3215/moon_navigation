#
# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for details.
#
import cupy as cp
from typing import List
from cupyx.scipy.ndimage import convolve, gaussian_filter

from .plugin_manager import PluginBase


class CustomTraversabilityCost(PluginBase):
    """
    综合地形通行代价插件，用于 mode3 保守导航。
    结合 Sobel 坡度 + 局部粗糙度 → 归一化代价 [0, 1]。
    0 = 可自由通行, 1 = 不可通行, NaN = 未知区域。
    NaN 邻域的有效栅格也标记为 NaN（与旧插件行为一致，避免黑色边框）。
    """

    def __init__(
        self,
        input_layer_name: str = "inpaint",
        slope_weight: float = 0.7,
        roughness_weight: float = 0.3,
        slope_critical_deg: float = 30.0,
        roughness_critical: float = 0.08,
        slope_exponent: float = 1.6,
        roughness_exponent: float = 1.4,
        danger_slope_deg: float = 20.0,
        danger_roughness: float = 0.05,
        danger_boost: float = 0.45,
        cost_threshold: float = 0.05,
        nan_dilate: int = 3,
        nan_cost: float = 0.55,
        nan_as_cost: bool = True,
        grid_resolution: float = 0.1,
        sigma: float = 2.0,
        pre_sobel_sigma: float = 0.8,
        roughness_kernel: int = 5,
        **kwargs,
    ):
        super().__init__()
        self.input_layer_name = input_layer_name
        self.slope_weight = float(slope_weight)
        self.roughness_weight = float(roughness_weight)
        self.slope_critical_deg = float(slope_critical_deg)
        self.roughness_critical = float(roughness_critical)
        self.slope_exponent = float(slope_exponent)
        self.roughness_exponent = float(roughness_exponent)
        self.danger_slope_deg = float(danger_slope_deg)
        self.danger_roughness = float(danger_roughness)
        self.danger_boost = float(danger_boost)
        self.cost_threshold = float(cost_threshold)
        self.nan_dilate = int(nan_dilate)
        self.nan_cost = float(nan_cost)
        self.nan_as_cost = bool(nan_as_cost)
        self.grid_resolution = float(grid_resolution)
        self.sigma = float(sigma)
        self.pre_sobel_sigma = float(pre_sobel_sigma)
        self.roughness_kernel = int(roughness_kernel)

    def __call__(
        self,
        elevation_map: cp.ndarray,
        layer_names: List[str],
        plugin_layers: cp.ndarray,
        plugin_layer_names: List[str],
        *args,
    ) -> cp.ndarray:
        # 获取输入高程图层
        if self.input_layer_name in layer_names:
            idx = layer_names.index(self.input_layer_name)
            h = elevation_map[idx].copy()
        elif self.input_layer_name in plugin_layer_names:
            idx = plugin_layer_names.index(self.input_layer_name)
            h = plugin_layers[idx].copy()
        else:
            # Fallback to elevation layer when configured source layer is unavailable.
            h = elevation_map[0].copy()

        nan_mask = cp.isnan(h)

        # 膨胀 NaN 掩码：NaN 邻域内的有效栅格也将被标记为 NaN（避免边界假梯度 → 黑色边框）
        # cval=0 表示地图外部不当作 NaN，只膨胀地图内部的 NaN 区域
        if self.nan_dilate > 0:
            k = 2 * self.nan_dilate + 1
            dilate_kernel = cp.ones((k, k), dtype=cp.float32)
            has_nan_nearby = convolve(nan_mask.astype(cp.float32), dilate_kernel, mode="constant", cval=0.0) > 0
        else:
            has_nan_nearby = nan_mask.copy()

        # 用局部均值填充 NaN（避免 NaN→0 在边界产生假梯度）
        valid_float = (~nan_mask).astype(cp.float32)
        h_safe = cp.where(nan_mask, 0.0, h).astype(cp.float32)
        # 5×5 均值填补
        fill_k = cp.ones((5, 5), dtype=cp.float32)
        sum_v = convolve(h_safe, fill_k, mode="nearest")
        cnt_v = convolve(valid_float, fill_k, mode="nearest")
        local_mean = sum_v / cp.maximum(cnt_v, 1.0)
        h_filled = cp.where(nan_mask, local_mean, h).astype(cp.float32)

        # ===== 1. 坡度计算 (Sobel 梯度 → 物理坡度角) =====
        if self.pre_sobel_sigma > 0:
            h_for_grad = gaussian_filter(h_filled, sigma=self.pre_sobel_sigma)
        else:
            h_for_grad = h_filled

        res = self.grid_resolution
        sobel_x = cp.array(
            [[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]], dtype=cp.float32
        ) / (8.0 * res)
        sobel_y = cp.array(
            [[-1, -2, -1], [0, 0, 0], [1, 2, 1]], dtype=cp.float32
        ) / (8.0 * res)

        grad_x = convolve(h_for_grad, sobel_x, mode="nearest")
        grad_y = convolve(h_for_grad, sobel_y, mode="nearest")

        slope_tan = cp.sqrt(grad_x ** 2 + grad_y ** 2)
        slope_deg = cp.degrees(cp.arctan(slope_tan))
        slope_norm = cp.clip(slope_deg / max(1e-3, self.slope_critical_deg), 0.0, 1.0)
        slope_cost = cp.power(slope_norm, self.slope_exponent)

        # ===== 2. 粗糙度计算 (局部高程标准差，仅统计有效像元) =====
        ks = self.roughness_kernel
        box = cp.ones((ks, ks), dtype=cp.float32)

        sum_h = convolve(h_safe, box, mode="nearest")
        sum_h2 = convolve(h_safe ** 2, box, mode="nearest")
        valid_count = convolve(valid_float, box, mode="nearest")
        safe_n = cp.maximum(valid_count, 1.0)

        mean_h = sum_h / safe_n
        mean_h2 = sum_h2 / safe_n
        variance = cp.maximum(mean_h2 - mean_h ** 2, 0.0)
        roughness = cp.sqrt(variance)
        roughness_norm = cp.clip(roughness / max(1e-6, self.roughness_critical), 0.0, 1.0)
        roughness_cost = cp.power(roughness_norm, self.roughness_exponent)

        # ===== 3. 加权融合 =====
        cost = self.slope_weight * slope_cost + self.roughness_weight * roughness_cost

        # ===== 3.1 危险区抬升：中等坡度/粗糙度提前显著增大代价 =====
        slope_danger = cp.clip(
            (slope_deg - self.danger_slope_deg)
            / max(1e-3, self.slope_critical_deg - self.danger_slope_deg),
            0.0,
            1.0,
        )
        roughness_danger = cp.clip(
            (roughness - self.danger_roughness)
            / max(1e-6, self.roughness_critical - self.danger_roughness),
            0.0,
            1.0,
        )
        danger_ratio = cp.maximum(slope_danger, roughness_danger)
        cost = cost + self.danger_boost * cp.power(danger_ratio, 2.0)
        cost = cp.clip(cost, 0.0, 1.0)

        # ===== 4. 高斯平滑（先平滑再阈值，避免条纹化） =====
        if self.sigma > 0:
            cost_smooth = cp.where(nan_mask, 0.0, cost)
            valid_smooth = gaussian_filter(valid_float, sigma=self.sigma)
            cost = gaussian_filter(cost_smooth, sigma=self.sigma)
            # 归一化：消除 NaN 区域对平滑的稀释
            cost = cp.where(valid_smooth > 0.1, cost / valid_smooth, 0.0)
            cost = cp.clip(cost, 0.0, 1.0)

        # ===== 5. 阈值截断：低于阈值的代价归零（消除微小起伏噪声） =====
        cost = cp.where(cost < self.cost_threshold, 0.0, cost)

        # ===== 6. NaN 区域处理 =====
        if self.nan_as_cost:
            # Keep scalar clamp in Python space to avoid cupy scalar-clip edge cases.
            fill_value = float(max(0.0, min(1.0, self.nan_cost)))
            cost[has_nan_nearby] = fill_value
            cost[nan_mask] = fill_value
        else:
            cost[has_nan_nearby] = cp.nan
            cost[nan_mask] = cp.nan

        return cost
