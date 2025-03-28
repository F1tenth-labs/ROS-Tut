import rclpy
from rclpy.node import Node

# We will just publish a dummy LaserScan message.
from sensor_msgs.msg import LaserScan
import math

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        # Publisher to /scan
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        # Create a timer that triggers the publishing callback
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_scan)

        self.get_logger().info('SensorNode has been started.')

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.pi / 180.0
        scan_msg.range_min = 0.2
        scan_msg.range_max = 10.0

        # Fill with dummy data (e.g., 360 samples all at 1 meter)
        num_samples = 360
        scan_msg.ranges = [1.0]*num_samples
        scan_msg.intensities = [0.0]*num_samples

        # Publish message
        self.publisher_.publish(scan_msg)
        self.get_logger().info('Published dummy LaserScan')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
