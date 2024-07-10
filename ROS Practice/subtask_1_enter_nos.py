#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class EnterNos(Node):

    def __init__(self):
        super().__init__("Addition_Publisher")
        self.number_pub1 = self.create_publisher(Int32, "Addition_Topic1", 10)
        self.number_pub2 = self.create_publisher(Int32, "Addition_Topic2", 10)

    def takeInput(self):
        self.get_logger().info("Enter 1st number - ")
        num1 = int(input())
        self.get_logger().info("Enter 2nd number - ")
        num2 = int(input())

        msg1 = Int32()
        msg1.data = num1
        msg2 = Int32()
        msg2.data = num2

        self.number_pub1.publish(msg1)
        self.number_pub2.publish(msg2)

        # self.get_logger().info(f"Published {num1} to Addition_Topic1 and {num2} to Addition_Topic2")

def main(args=None):
    rclpy.init(args=args)
    node = EnterNos()
    node.takeInput()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
