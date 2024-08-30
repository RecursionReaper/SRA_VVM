#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        self.subscription = self.create_subscription(JointState,'/joint_states',self.listener_callback,10)
        self.publisher = self.create_publisher(JointState,'current_angle_topic',10)

        self.angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def listener_callback(self, msg):
        joint_names = msg.name
        joint_positions = msg.position

        for i, (name, position) in enumerate(zip(joint_names, joint_positions)):
            angle_degrees = position * (180.0 / math.pi)
            if i < len(self.angles):
                self.angles[i] = angle_degrees

        print("---------------------------- ")
        print("---------------------------- ")
        print("Joint 1 - ", self.angles[2])
        print("Joint 2 - ", self.angles[0])
        print("Joint 3 - ", self.angles[1])
        print("Joint 4 - ", self.angles[3])


        angle_msg = JointState()
        angle_msg.name = ['Theta1', 'Theta2', 'Theta3', 'Theta4']
        angle_msg.position = [self.angles[0], self.angles[1], self.angles[2], self.angles[3]]
        self.publisher.publish(angle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
