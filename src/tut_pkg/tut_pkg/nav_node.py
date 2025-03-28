import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        # 1) Declare parameters (threshold, linear speed, angular speed)
        self.declare_parameter('front_distance_threshold', 0.5)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)

        # 2) Retrieve parameter values
        self.front_distance_threshold = self.get_parameter('front_distance_threshold').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # 3) Create subscriber to /scan
        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
        )

        # 4) Create publisher for /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            'NavNode has been started with params: '
            f'front_distance_threshold={self.front_distance_threshold}, '
            f'linear_speed={self.linear_speed}, '
            f'angular_speed={self.angular_speed}.'
        )

    def scan_callback(self, scan_msg):
        # Grab the distance at the front
        front_index = len(scan_msg.ranges) // 2
        front_distance = scan_msg.ranges[front_index]

        cmd = Twist()

        if front_distance < self.front_distance_threshold:
            # Turn in place
            cmd.angular.z = self.angular_speed
            self.get_logger().info(
                f'Object too close ({front_distance} < {self.front_distance_threshold}); turning in place.'
            )
        else:
            # Move forward
            cmd.linear.x = self.linear_speed
            self.get_logger().info(
                f'Moving forward (front_distance={front_distance}).'
            )

        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    rclpy.shutdown()
