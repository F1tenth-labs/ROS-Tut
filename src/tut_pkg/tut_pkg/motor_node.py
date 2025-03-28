import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # 1) Declare a parameter to, for example, identify the motor
        self.declare_parameter('motor_name', 'motor_A')
        self.motor_name = self.get_parameter('motor_name').value

        # 2) Subscribe to /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )

        self.get_logger().info(
            f'MotorNode has been started for {self.motor_name}.'
        )

    def cmd_vel_callback(self, twist_msg):
        # Normally you would interface with hardware here
        self.get_logger().info(
            f'{self.motor_name} received Twist: '
            f'linear.x={twist_msg.linear.x}, '
            f'angular.z={twist_msg.angular.z}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    rclpy.shutdown()
