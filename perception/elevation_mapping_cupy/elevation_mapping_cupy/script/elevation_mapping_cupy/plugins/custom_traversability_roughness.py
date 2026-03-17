#
# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under the MIT license. See LICENSE file in the project root for details.
#
import cupy as cp
from typing import List
# import cupy.gradient as gradient
# import cupyx.scipy.ndimage as ndimage
from cupyx.scipy.ndimage import convolve
from cupyx.scipy.ndimage import minimum_filter, maximum_filter, gaussian_filter
import numpy as np


from .plugin_manager import PluginBase


class CustomTraversabilitySlope(PluginBase):


    def __init__(self, input_layer_name: str = "elevation", neighbor_range: int = 3,multipier = 1, grid_resolution: float = 0.1, **kwargs):
        super().__init__()
        self.input_layer_name = input_layer_name
        self.range = neighbor_range
        self.multipier = multipier
        self.grid_resolution = float(grid_resolution)
        
    

    def calculate_slope_geometry(self, h: cp.ndarray, grid_resolution: float) -> cp.ndarray:
        """
        参数:
        - h: 高程图 (cp.ndarray)，无效区域为 NaN。
        - grid_resolution: 单个格子的物理尺寸（单位：米）。

        返回:
        - slope: 坡度图 (cp.ndarray)，无效区域为 NaN。
        """
        # 有效值掩码
        valid_mask = ~cp.isnan(h)

        # 定义3x3邻域的卷积核
        footprint = cp.array([[1, 1, 1],
                            [1, 1, 1],
                            [1, 1, 1]], dtype=cp.float32)

        # 设置新的检查范围
        nan_range = 3  # 设置范围大一点，您可以根据需要调整这个值
        footprint = cp.ones((2 * nan_range + 1, 2 * nan_range + 1), dtype=cp.float32)

        # 检查每个格子周围是否有 NaN，使用更大的范围
        nan_mask = cp.isnan(h)
        has_nan_in_neighborhood = convolve(nan_mask.astype(cp.float32), footprint, mode="constant", cval=0) > 0

        # 计算邻域内的最小值和最大值
        h_min_temp = cp.where(valid_mask, h, cp.inf)
        h_max_temp = cp.where(valid_mask, h, -cp.inf)

        # 使用 minimum_filter 和 maximum_filter 计算更大邻域的最小值和最大值
        min_h = minimum_filter(h_min_temp, size=(2 * nan_range + 1), mode="constant", cval=cp.inf)
        max_h = maximum_filter(h_max_temp, size=(2 * nan_range + 1), mode="constant", cval=-cp.inf)

        # 计算最大高程差
        max_diff = max_h - min_h

        # 设置有 NaN 邻域的格子为无效
        max_diff = cp.where(has_nan_in_neighborhood, 0, max_diff)

        # 计算水平距离（对角线）
        max_distance = grid_resolution * cp.sqrt(2)

        # 计算坡度
        slope = max_diff / max_distance
        slope = slope * 45

        # 恢复无效值区域
        slope[has_nan_in_neighborhood] = 0
        slope = cp.where(cp.isinf(slope), 0, slope)

        # # 判断坡度,如果大于一定值则设置为无效
        slope = cp.where(slope > 100, 0, slope)
        
        slope[has_nan_in_neighborhood] = 0

        # gaussian filter
        slope = gaussian_filter(slope, sigma=1)

        return slope




    def __call__(
        self,
        elevation_map: cp.ndarray,
        layer_names: List[str],
        plugin_layers: cp.ndarray,
        plugin_layer_names: List[str],
        *args,
    ) -> cp.ndarray:

        if self.input_layer_name in layer_names:
            idx = layer_names.index(self.input_layer_name)
            h = elevation_map[idx]
        elif self.input_layer_name in plugin_layer_names:
            idx = plugin_layer_names.index(self.input_layer_name)
            h = plugin_layers[idx]
        else:
            print("layer name {} was not found. Using elevation layer.".format(self.input_layer_name))
            h = elevation_map[0]

        # 几何方法计算梯度
        slope = self.calculate_slope_geometry(h, self.grid_resolution)

        return slope


        

