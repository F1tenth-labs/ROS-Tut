import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        # Subscribe to /scan
        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )

        # Publisher to /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('NavNode has been started.')

    def scan_callback(self, scan_msg):
        # Very simple "navigation" logic:
        # If the front reading is < 0.5, turn; otherwise go forward
        front_index = len(scan_msg.ranges) // 2
        front_distance = scan_msg.ranges[front_index]

        cmd = Twist()
        if front_distance < 0.5:
            # Turn in place
            cmd.angular.z = 0.5
            self.get_logger().info('Object too close! Turning...')
        else:
            # Move forward
            cmd.linear.x = 0.2
            self.get_logger().info('Moving forward...')

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    rclpy.shutdown()
