#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys

class ObjPositionPublisher(Node):
    def __init__(self):
        super().__init__('obj_position_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/obj_position', 10)
        self.timer = self.create_timer(0.5, self.publish_obj_position)

    def publish_obj_position(self):

        
        ObjDistance_X = float(input("Enter ObjDistance X coordinate: "))
        ObjDistance_Y = float(input("Enter ObjDistance Y coordinate: "))
        ObjDistance_Z = float(input("Enter ObjDistance Z coordinate: "))
        

       
        msg = Float64MultiArray()
        msg.data = [ObjDistance_X, ObjDistance_Y, ObjDistance_Z]
        
       
        self.publisher_.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = ObjPositionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
