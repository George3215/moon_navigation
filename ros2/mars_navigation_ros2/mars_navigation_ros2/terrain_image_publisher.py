"""Terrain image publisher (ROS2 stand-in for the rover camera).

No Gazebo/camera is available on a ROS 2 Humble host, so this node feeds the
VLM classifier with representative terrain RGB images. Instead of a fixed
timer, it tracks the rover's position from ``/pose_with_covariance`` and
publishes the image that matches the terrain *segment* the rover is currently
in, so the VLM-driven mode switching reflects the mixed heightmap:

    x <  rocky_lo        -> flat        (Mode 1)
    rocky_lo <= x < rocky_hi -> rocky   (Mode 2)
    rocky_hi <= x < ridge_hi  -> challenging (Mode 3)
    x >= ridge_hi        -> flat        (Mode 1)

The zone boundaries mirror ``ros2/tools/generate_mixed_terrain.py``. Swap this
node for a real camera driver (publishing ``sensor_msgs/Image`` on the same
topic) to run the VLM classifier against live imagery.
"""

import os

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Image


def _to_image_msg(bgr, stamp, frame_id):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(bgr[:, :, ::-1]).tobytes()  # bgr -> rgb
    return msg


def _flat_image(size=(512, 512)):
    """Synthesize a flat, featureless terrain image."""
    h, w = size
    y, x = np.mgrid[0:h, 0:w]
    base = 150 + 20 * np.sin(x / 90.0)
    img = np.repeat(base.astype(np.uint8)[..., None], 3, axis=-1)
    return img


class TerrainImagePublisher(Node):
    def __init__(self):
        super().__init__("terrain_image_publisher")
        self.declare_parameter("image_topic", "/terrain/image")
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("pose_topic", "/pose_with_covariance")
        self.declare_parameter("flat_image", "")
        self.declare_parameter("rocky_image", "")
        self.declare_parameter("challenging_image", "")
        # Zone boundaries (world x, metres) -- keep in sync with generate_mixed_terrain.py.
        self.declare_parameter("rocky_lo", -12.0)
        self.declare_parameter("rocky_hi", -6.0)
        self.declare_parameter("ridge_hi", 6.0)

        self.pub = self.create_publisher(Image, self.get_parameter("image_topic").value, 1)

        flat_path = self.get_parameter("flat_image").value
        rocky_path = self.get_parameter("rocky_image").value
        challenging_path = self.get_parameter("challenging_image").value

        flat = cv2.imread(flat_path) if flat_path and os.path.exists(flat_path) else _flat_image()
        self.images = {"flat": flat}
        for name, path in (("rocky", rocky_path), ("challenging", challenging_path)):
            if path and os.path.exists(path):
                img = cv2.imread(path)
                if img is not None:
                    self.images[name] = img
                else:
                    self.get_logger().warn(f"could not read {path}")
            else:
                self.get_logger().warn(f"missing {name} image: {path!r}")

        self.current_x = None
        self.current_zone = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self._pose_cb,
            1,
        )
        # Publish the first (flat) frame ~1s after startup so late subscribers
        # such as vlm_mode are up before it arrives.
        self.first_timer = self.create_timer(1.0, self._first_publish)
        self.get_logger().info(
            f"terrain_image_publisher ready ({len(self.images)} images, position-driven)"
        )

    def _zone_for_x(self, x):
        if x < self.get_parameter("rocky_lo").value:
            return "flat"
        if x < self.get_parameter("rocky_hi").value:
            return "rocky"
        if x < self.get_parameter("ridge_hi").value:
            return "challenging"
        return "flat"

    def _first_publish(self):
        self._publish_zone("flat")
        self.first_timer.cancel()

    def _pose_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        zone = self._zone_for_x(self.current_x)
        if zone != self.current_zone:
            self._publish_zone(zone)

    def _publish_zone(self, zone):
        img = self.images.get(zone)
        if img is None:
            self.get_logger().warn(f"no image for zone {zone!r}; skipping", throttle_duration_sec=10.0)
            return
        msg = _to_image_msg(img, self.get_clock().now().to_msg(), self.get_parameter("frame_id").value)
        self.pub.publish(msg)
        self.current_zone = zone
        self.get_logger().info(
            f"published terrain image: {zone} (x={self.current_x if self.current_x is not None else '?'})"
        )


def main():
    rclpy.init()
    node = TerrainImagePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
