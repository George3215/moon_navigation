"""Global accessibility map publisher (ROS2 port of image2map_initialize.py).

Faithfully ports the original ``image2map_initialize.py`` pipeline, which turns a
grayscale terrain heightmap into a binary traversability OccupancyGrid:

    1. resize heightmap to ``image_real_meter / resolution`` pixels
    2. re-height (gray -> meters)
    3. calculate_accessibility_map: a cell is passable (0) if it has at least one
       8-neighbour whose slope stays within ``max_slope_angle``; otherwise it is an
       obstacle (255)
    4. filter_small_connected_regions: drop 1-pixel obstacle specks
    5. connect_regions_via_distance: distance-transform based cleanup
    6. apply_morphological_operations: close (dilate then erode)
    7. normalize to 0..100 and publish as ``nav_msgs/OccupancyGrid``

The multi-stage cleanup is essential: a naive ``np.gradient`` slope pass fragments
the free space into salt-and-pepper cells, which the global A* (with its 5x5
aggregation) then cannot traverse.
"""

import os

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


def _calculate_accessibility_map(image, map_resolution_meter, max_slope_angle):
    """0 = passable, 255 = obstacle (a cell is passable if it has a gentle neighbour)."""
    height, width = image.shape
    acc = np.full((height, width), 255, dtype=np.uint8)
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    max_slope_ratio = np.tan(np.radians(max_slope_angle))
    for i in range(height):
        for j in range(width):
            current = float(image[i, j])
            for dx, dy in neighbors:
                ni, nj = i + dx, j + dy
                if 0 <= ni < height and 0 <= nj < width:
                    horizontal = map_resolution_meter * (1 if dx == 0 or dy == 0 else np.sqrt(2))
                    if abs(current - float(image[ni, nj])) / horizontal <= max_slope_ratio:
                        acc[i, j] = 0
                        break
    return acc


def _filter_small_connected_regions(accessibility_map, min_size=2):
    """Drop obstacle regions smaller than ``min_size`` pixels (de-speckle)."""
    num_labels, labels = cv2.connectedComponents(accessibility_map)
    out = np.zeros_like(accessibility_map)
    for label in range(1, num_labels):
        region = (labels == label)
        if int(np.sum(region)) >= min_size:
            out[region] = 255
    return out


def _connect_regions_via_distance(accessibility_map, distance_threshold=1):
    """Distance-transform based cleanup of the binary traversability map."""
    dist = cv2.distanceTransform(255 - accessibility_map, cv2.DIST_L2, 5)
    return ((dist < distance_threshold).astype(np.uint8)) * 255


def _apply_morphological_operations(accessibility_map, kernel_size=2):
    """Closing (dilate then erode) to smooth the map."""
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    dilated = cv2.dilate(accessibility_map, kernel, iterations=1)
    return cv2.erode(dilated, kernel, iterations=1)


class ImageToMap(Node):
    def __init__(self):
        super().__init__("image_to_map")
        default_image = os.path.join(
            os.getcwd(), "nav_framework/global_initializer/scripts/images/marsyard2022_terrain_hm.jpg"
        )
        self.declare_parameter("image_path", default_image)
        self.declare_parameter("image_real_meter", 54.0)
        self.declare_parameter("height_per_gray", 4.820803273566 / 255.0)
        self.declare_parameter("max_slope_angle", 2.0)
        self.declare_parameter("resolution", 0.2)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("origin_x", -27.0)
        self.declare_parameter("origin_y", -27.0)
        self.declare_parameter("output_topic", "/mode1/occupancy_grid")
        self.declare_parameter("publish_period", 1.0)

        self.pub = self.create_publisher(OccupancyGrid, self.get_parameter("output_topic").value, 1)
        self.map_msg = self._build_map()
        self.timer = self.create_timer(float(self.get_parameter("publish_period").value), self._publish)
        self.get_logger().info(f"publishing initial map on {self.get_parameter('output_topic').value}")

    def _build_map(self):
        image_path = self.get_parameter("image_path").value
        image = cv2.imread(image_path)
        if image is None:
            raise FileNotFoundError(image_path)

        real_meter = float(self.get_parameter("image_real_meter").value)
        resolution = float(self.get_parameter("resolution").value)
        size = int(round(real_meter / resolution))
        resized = cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY).astype(np.float32)
        height = gray * float(self.get_parameter("height_per_gray").value)

        acc = _calculate_accessibility_map(
            height, resolution, float(self.get_parameter("max_slope_angle").value)
        )
        cleaned = _filter_small_connected_regions(acc)
        connected = _connect_regions_via_distance(cleaned)
        morphed = _apply_morphological_operations(connected)
        normalized = cv2.normalize(morphed, None, 0, 100, cv2.NORM_MINMAX, cv2.CV_8U)
        cost = np.flip(normalized, axis=0)

        msg = OccupancyGrid()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.info.width = size
        msg.info.height = size
        msg.info.resolution = resolution
        msg.info.origin.position.x = float(self.get_parameter("origin_x").value)
        msg.info.origin.position.y = float(self.get_parameter("origin_y").value)
        msg.info.origin.orientation.w = 1.0
        msg.data = cost.flatten().astype(int).tolist()
        return msg

    def _publish(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map_msg)


def main():
    rclpy.init()
    node = ImageToMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
