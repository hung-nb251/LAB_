import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')
        self.create_subscription(PointStamped, '/coord_transform/target_base', self.cb_target, 10)
        self.create_subscription(PoseStamped, '/cartesian_streamer/current_pose', self.cb_ee, 10)
        self.target_count = 0
        self.ee_count = 0
        self.timer = self.create_timer(1.0, self.timer_cb)

    def cb_target(self, msg):
        self.target_count += 1
    def cb_ee(self, msg):
        self.ee_count += 1
    def timer_cb(self):
        self.get_logger().info(f"Target msgs: {self.target_count}, EE msgs: {self.ee_count}")

rclpy.init()
node = DebugNode()
rclpy.spin(node)
