"""Synthetic hazard/elevation mapper (ROS2 stand-in for the Gazebo perception).

The paper's Mode 2 (rocky) builds a local obstacle map from point-cloud height
differences and Mode 3 (challenging) builds a local elevation cost map with GPU
acceleration (``elevation_mapping_cupy``). Neither the MarsSim/Gazebo sim nor
the CuPy mapper is available on a ROS 2 Humble host, so this node derives the
same local maps from the terrain heightmap image and the rover pose, using the
faithful synthesis in :mod:`.elevation_synthesis`:

- ``/mode2/occupancy_grid``: 20 m x 20 m binary obstacle map.  Cells whose
  height rises more than ``rock_height_threshold`` above the local ground plane
  are marked untraversable (port of ``hazard_detection``'s RANSAC-ground +
  above-ground obstacle test).
- ``/mode3/occupancy_grid``: 20 m x 20 m graded cost map.  A weighted fusion of
  Sobel slope and local roughness (port of
  ``custom_traversability_cost.py``) mapped to 0..100 traversal cost.

Both are published at the global-map resolution so the map server can merge
them with its mode-priority accumulation.
"""

import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String

from .elevation_synthesis import compute_rock_map, compute_traversability_cost, load_height


class HazardMapper(Node):
    def __init__(self):
        super().__init__("hazard_mapper")
        self.declare_parameter("image_path", "")
        self.declare_parameter("image_real_meter", 54.0)
        self.declare_parameter("height_per_gray", 4.820803273566 / 255.0)
        self.declare_parameter("resolution", 0.2)
        self.declare_parameter("origin_x", -27.0)
        self.declare_parameter("origin_y", -27.0)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("mode_topic", "/navigation_mode")
        self.declare_parameter("local_map_size", 20.0)
        self.declare_parameter("mode2_topic", "/mode2/occupancy_grid")
        self.declare_parameter("mode3_topic", "/mode3/occupancy_grid")
        self.declare_parameter("mode2_period", 1.0)
        self.declare_parameter("mode3_period", 2.0)

        # Mode 2 (rocky) -- port of hazard_detection.
        self.declare_parameter("ground_sigma", 2.0)          # px, local ground plane
        self.declare_parameter("rock_height_threshold", 0.1)  # m above ground == rock
        self.declare_parameter("rock_dilation_radius_m", 0.0)  # m, inflate rock blobs to the physical boulder footprint
        # Mode 3 (challenging) -- port of custom_traversability_cost.
        self.declare_parameter("slope_weight", 0.7)
        self.declare_parameter("roughness_weight", 0.3)
        self.declare_parameter("slope_critical_deg", 30.0)
        self.declare_parameter("roughness_critical", 0.08)
        self.declare_parameter("slope_exponent", 1.6)
        self.declare_parameter("roughness_exponent", 1.4)
        self.declare_parameter("danger_slope_deg", 20.0)
        self.declare_parameter("danger_roughness", 0.05)
        self.declare_parameter("danger_boost", 0.45)
        self.declare_parameter("cost_threshold", 0.05)
        self.declare_parameter("cost_sigma", 2.0)
        self.declare_parameter("pre_sobel_sigma", 0.8)
        # The CuPy plugin's roughness box is 5 px at its 0.1 m grid == a 0.5 m
        # window.  At our 0.2 m grid, 3 px == 0.6 m is the faithful match; a
        # 5 px (1.0 m) window over-inflates the roughness on a smooth ridge's
        # curvature and can push the ridge core past the hard-obstacle cost
        # (>= 88), blocking Mode 3 even though the slope is traversable.
        self.declare_parameter("roughness_kernel", 3)

        image_path = self.get_parameter("image_path").value
        if not image_path or not os.path.exists(image_path):
            raise FileNotFoundError(f"hazard_mapper image_path not found: {image_path!r}")

        height = load_height(
            image_path,
            float(self.get_parameter("image_real_meter").value),
            float(self.get_parameter("resolution").value),
            float(self.get_parameter("height_per_gray").value),
        )
        self.resolution = float(self.get_parameter("resolution").value)
        self.height = height
        self._rock, self._obstacle = compute_rock_map(
            height,
            ground_sigma=float(self.get_parameter("ground_sigma").value),
            rock_height_threshold=float(self.get_parameter("rock_height_threshold").value),
        )
        # The residual detector flags only the narrow Gaussian *peak* of each
        # boulder (~2 cells), but the physical boulder is ~1.3 m across.  Inflate
        # the rock blobs to the boulder footprint so Mode 2 routes around the
        # whole rock instead of clipping its flank and high-centring.
        dil_r = float(self.get_parameter("rock_dilation_radius_m").value)
        if dil_r > 0:
            k = max(1, int(round(dil_r / self.resolution)))
            kernel = np.ones((2 * k + 1, 2 * k + 1), np.uint8)
            self._obstacle = (cv2.dilate((self._obstacle > 0).astype(np.uint8), kernel) * 100.0).astype(np.float32)
        _, _, self._cost = compute_traversability_cost(
            height,
            self.resolution,
            slope_weight=float(self.get_parameter("slope_weight").value),
            roughness_weight=float(self.get_parameter("roughness_weight").value),
            slope_critical_deg=float(self.get_parameter("slope_critical_deg").value),
            roughness_critical=float(self.get_parameter("roughness_critical").value),
            slope_exponent=float(self.get_parameter("slope_exponent").value),
            roughness_exponent=float(self.get_parameter("roughness_exponent").value),
            danger_slope_deg=float(self.get_parameter("danger_slope_deg").value),
            danger_roughness=float(self.get_parameter("danger_roughness").value),
            danger_boost=float(self.get_parameter("danger_boost").value),
            cost_threshold=float(self.get_parameter("cost_threshold").value),
            cost_sigma=float(self.get_parameter("cost_sigma").value),
            pre_sobel_sigma=float(self.get_parameter("pre_sobel_sigma").value),
            roughness_kernel=int(self.get_parameter("roughness_kernel").value),
        )

        self.pose = None
        self.mode = "Mode 1"
        self.mode2_pub = self.create_publisher(OccupancyGrid, self.get_parameter("mode2_topic").value, 1)
        self.mode3_pub = self.create_publisher(OccupancyGrid, self.get_parameter("mode3_topic").value, 1)
        self.create_subscription(PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, self._pose_cb, 10)
        self.create_subscription(String, self.get_parameter("mode_topic").value, self._mode_cb, 10)

        self.create_timer(float(self.get_parameter("mode2_period").value), self._publish_mode2)
        self.create_timer(float(self.get_parameter("mode3_period").value), self._publish_mode3)
        self.get_logger().info(
            f"hazard_mapper ready (height {self.height.shape}, "
            f"{int(np.sum(self._obstacle > 0))} rock cells, "
            f"{int(np.sum(self._cost >= 88))} hard-cost cells)"
        )

    def _pose_cb(self, msg):
        self.pose = msg

    def _mode_cb(self, msg):
        if msg.data != self.mode:
            self.get_logger().info(f"hazard_mapper switching to {msg.data}")
            self.mode = msg.data

    def _crop(self, data):
        """Crop a local window of ``data`` centered on the rover pose."""
        if self.pose is None:
            return None, None
        size_m = float(self.get_parameter("local_map_size").value)
        cells = int(round(size_m / self.resolution))
        half = cells // 2
        px = self.pose.pose.pose.position.x
        py = self.pose.pose.pose.position.y
        cx = int(round((px - float(self.get_parameter("origin_x").value)) / self.resolution))
        cy = int(round((py - float(self.get_parameter("origin_y").value)) / self.resolution))

        h, w = data.shape
        window = np.full((cells, cells), np.nan, dtype=np.float32)
        for i in range(cells):
            sy = cy - half + i
            for j in range(cells):
                sx = cx - half + j
                if 0 <= sx < w and 0 <= sy < h:
                    window[i, j] = data[sy, sx]

        origin_x = float(self.get_parameter("origin_x").value) + (cx - half) * self.resolution
        origin_y = float(self.get_parameter("origin_y").value) + (cy - half) * self.resolution
        return window, (origin_x, origin_y)

    def _make_msg(self, data, origin, frame_id):
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolution
        msg.info.width = data.shape[1]
        msg.info.height = data.shape[0]
        msg.info.origin.position.x = origin[0]
        msg.info.origin.position.y = origin[1]
        msg.info.origin.orientation.w = 1.0
        data = np.nan_to_num(data, nan=-1.0)
        msg.data = data.flatten().astype(int).tolist()
        return msg

    def _publish_mode2(self):
        # Only emit the rocky-obstacle map while Mode 2 is active; otherwise the
        # map server's mode-priority merge would let stale rocks block Mode 1/3.
        if self.mode != "Mode 2" or self.pose is None:
            return
        window, origin = self._crop(self._obstacle)
        if window is None:
            return
        self.mode2_pub.publish(self._make_msg(window, origin, self.get_parameter("frame_id").value))

    def _publish_mode3(self):
        # Only emit the elevation cost map while Mode 3 is active.
        if self.mode != "Mode 3" or self.pose is None:
            return
        window, origin = self._crop(self._cost)
        if window is None:
            return
        self.mode3_pub.publish(self._make_msg(window, origin, self.get_parameter("frame_id").value))


def main():
    rclpy.init()
    node = HazardMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
