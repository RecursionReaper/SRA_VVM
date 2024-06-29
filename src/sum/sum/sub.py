import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class take2values(Node):
    def __init__(self):
        super().__init__('Subscriber_Node')
        self.counter = 0
        self.sum = 0
        self.subscriber = self.create_subscription(Int32,"add",self.callback,10)
        
    def callback(self,msg):
        self.counter += 1
        self.sum += msg.data
        if self.counter == 2:
            print("The Sum for the given values is :",end = " ")
            print(self.sum)


def main(args=None):
    rclpy.init(args=args)
    node1=take2values()
    rclpy.spin(node1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

