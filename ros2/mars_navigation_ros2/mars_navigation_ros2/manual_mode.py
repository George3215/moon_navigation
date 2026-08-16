import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ManualMode(Node):
    def __init__(self):
        super().__init__("manual_mode")
        self.declare_parameter("mode_topic", "/navigation_mode")
        # If set to 1/2/3, publish that mode on an interval instead of reading
        # stdin. Lets single-mode experiments run headlessly.
        self.declare_parameter("fixed_mode", 0)
        self.declare_parameter("publish_period", 1.0)
        self.pub = self.create_publisher(String, self.get_parameter("mode_topic").value, 1)

        self.fixed_mode = int(self.get_parameter("fixed_mode").value)
        if self.fixed_mode in (1, 2, 3):
            self.create_timer(float(self.get_parameter("publish_period").value), self._publish_fixed)
            self.get_logger().info(f"manual_mode publishing fixed mode {self.fixed_mode}")
        else:
            self.get_logger().info("Enter mode: 1 efficient, 2 safe, 3 conservative")

    def _publish_fixed(self):
        msg = String()
        msg.data = f"Mode {self.fixed_mode}"
        self.pub.publish(msg)

    def run(self):
        if self.fixed_mode in (1, 2, 3):
            rclpy.spin(self)
            return
        while rclpy.ok():
            try:
                raw = input("mode> ").strip()
            except EOFError:
                return
            if raw not in {"1", "2", "3"}:
                self.get_logger().warning("valid inputs are 1, 2, or 3")
                continue
            msg = String()
            msg.data = f"Mode {raw}"
            self.pub.publish(msg)
            self.get_logger().info(f"published {msg.data}")


def main():
    rclpy.init()
    node = ManualMode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
