import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class Tester(Node):
    def __init__(self):
        super().__init__('tester')
        self.sub = self.create_subscription(PoseStamped, '/cartesian_streamer/current_pose', self.cb, 10)
        self.msg_count = 0
    def cb(self, msg):
        self.msg_count += 1
        print(f"Received pose! count={self.msg_count}")

def main():
    rclpy.init()
    node = Tester()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
