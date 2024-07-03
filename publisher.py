import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__("talker_node")
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        
        # Prompt user for input
        num1 = float(input("Enter the first number: "))
        num2 = float(input("Enter the second number: "))
        
        # Calculate the sum
        self.sum = num1 + num2

        # Publish the sum
        self.publish_sum()

    def publish_sum(self):
        msg = String()
        msg.data = str(self.sum) 
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    # create node
    talker_node = TalkerNode()
    # use node
    rclpy.spin_once(talker_node)
    # destroy node
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
