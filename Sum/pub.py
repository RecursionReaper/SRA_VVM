import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class give2values(Node):
    def __init__(self):
        super().__init__('Publisher_Node')
        self.num_publisher=self.create_publisher(Int32,"add",10)
        self.value_input()
        
    def value_input(self):
        print("Enter the first Number:")
        a = int(input())
        msg = Int32()
        msg.data = a
        self.num_publisher.publish(msg)

        print("Enter the second Number:")
        b = int(input())
        msg = Int32()
        msg.data = b
        self.num_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node=give2values()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()