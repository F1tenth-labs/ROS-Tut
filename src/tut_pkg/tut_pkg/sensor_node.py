import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # 1) Declare parameters and retrieve their values
        self.declare_parameter('publish_frequency', 1.0)
        publish_frequency = self.get_parameter('publish_frequency').value
        
        # 2) Create Publisher
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        
        # 3) Create timer based on the parameter
        timer_period = 1.0 / publish_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.publish_scan)

        self.get_logger().info(
            f'SensorNode started. Publishing LaserScan at {publish_frequency} Hz.'
        )

    def publish_scan(self):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.pi / 180.0
        scan_msg.range_min = 0.2
        scan_msg.range_max = 10.0

        # Fill with dummy data (e.g., 360 samples at 1 meter)
        num_samples = 360
        scan_msg.ranges = [1.0]*num_samples
        scan_msg.intensities = [0.0]*num_samples

        self.publisher_.publish(scan_msg)
        self.get_logger().debug('Published dummy LaserScan')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
