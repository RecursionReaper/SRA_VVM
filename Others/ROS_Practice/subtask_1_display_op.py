#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class DisplayOp(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        self.num1 = None
        self.num2 = None

        self.create_subscription(Int32, "Addition_Topic1", self.number1_callback, 10)
        self.create_subscription(Int32, "Addition_Topic2", self.number2_callback, 10)

    def number1_callback(self, msg: Int32):
        self.num1 = msg.data
        self.calculate_and_display_sum()

    def number2_callback(self, msg: Int32):
        self.num2 = msg.data
        self.calculate_and_display_sum()

    def calculate_and_display_sum(self):
        if self.num1 is not None and self.num2 is not None:
            total_sum = self.num1 + self.num2
            self.get_logger().info(f"Sum of nos: {total_sum}")
            self.num1 = None
            self.num2 = None
        

def main(args=None):
    rclpy.init(args=args)
    node = DisplayOp()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
