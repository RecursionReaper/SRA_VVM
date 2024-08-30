#!/usr/bin/python3

''' Subscribe from current_angle_topic and obj_position, then publish to inverse_kinematics/inputs '''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

       
        self.angles = [0.0, 0.0, 0.0, 0.0]
        self.obj_position = [0.0, 0.0, 0.0]

      
        self.angle_subscription = self.create_subscription(
            Float64MultiArray,
            'current_angle_topic',
            self.angle_callback,
            10
        )

        
        self.position_subscription = self.create_subscription(
            Float64MultiArray,
            '/obj_position',
            self.position_callback,
            10
        )

        self.publisher_ = self.create_publisher(Float64MultiArray, 'inverse_kinematics/inputs', 10)

    def angle_callback(self, msg):
        if len(msg.data) == 4:
            self.angles = msg.data

            print("Received angles:-----------------")
            print("Theta1 - ", (self.angles[0]))
            print("Theta2 - ", (self.angles[1]))
            print("Theta3 - ", (self.angles[2]))
            print("Theta4 - ", (self.angles[3]))

            self.calculate_and_publish()

    def position_callback(self, msg):
        if len(msg.data) == 3:
            self.obj_position = msg.data

            print("Received object position:-----------")
            print("ObjDistance_X - ", self.obj_position[0])
            print("ObjDistance_Y - ", self.obj_position[1])
            print("ObjDistance_Z - ", self.obj_position[2])

            self.calculate_and_publish()

    def calculate_and_publish(self):
        
        if self.angles and self.obj_position:
            theta1 = (self.angles[0])
            theta2 = -(self.angles[1])
            theta3 = -(self.angles[2])
            theta4 = -(self.angles[3])
            ObjDistance_X = self.obj_position[0]
            ObjDistance_Y = self.obj_position[1]
            ObjDistance_Z = self.obj_position[2]

            H1 = np.array([[math.cos(theta1), 0, math.sin(theta1), 0],
                           [math.sin(theta1), 0, -math.cos(theta1), 0],
                           [0, 1, 0, 0.077],
                           [0, 0, 0, 1]])

            H2 = np.array([[math.cos(theta2), -math.sin(theta2), 0, 0],
                           [math.sin(theta2), math.cos(theta2), 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

            H2T = np.array([[1, 0, 0, 0.024],
                            [0, 1, 0, 0.128],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

            H3 = np.array([[math.cos(theta3), -math.sin(theta3), 0, 0.124 * math.cos(theta3)],
                           [math.sin(theta3), math.cos(theta3), 0, 0.124 * math.sin(theta3)],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])

            H4 = np.array([[math.cos(theta4), 0, -math.sin(theta4), 0.126 * math.cos(theta4)],
                           [math.sin(theta4), 0, math.cos(theta4), 0.126 * math.sin(theta4)],
                           [0, -1, 0, 0],
                           [0, 0, 0, 1]])

            HCam = np.array([[0, 0, 0, ObjDistance_X],
                             [0, 0, 0, ObjDistance_Y],
                             [0, 0, 0, ObjDistance_Z],
                             [0, 0, 0, 1]])

            H12 = np.dot(H1, H2)
            H12T = np.dot(H12, H2T)
            H123 = np.dot(H12T, H3)
            H1234 = np.dot(H123, H4)
            H1234Cam = np.dot(H1234, HCam)

            EEX = H1234Cam[0, 3]
            EEY = H1234Cam[1, 3]
            EEZ = H1234Cam[2, 3]

            print("Calculated end-effector position:----")
            print("EEX - ", EEX)
            print("EEY - ", EEY)
            print("EEZ - ", EEZ)

            
            msg_to_publish = Float64MultiArray()
            msg_to_publish.data = [EEX, EEY, EEZ]
            self.publisher_.publish(msg_to_publish)

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
