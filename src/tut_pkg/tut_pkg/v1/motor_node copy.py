import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        # Subscribe to /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )

        self.get_logger().info('MotorNode has been started.')

    def cmd_vel_callback(self, twist_msg):
        # Here you'd normally interface with your motor hardware.
        # For example, send twist_msg.linear.x and twist_msg.angular.z
        # to your motor driver or microcontroller.
        self.get_logger().info(f'Received Twist: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    rclpy.shutdown()
