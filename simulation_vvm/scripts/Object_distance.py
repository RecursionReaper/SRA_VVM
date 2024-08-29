#!/usr/bin/python3

''' Subsribe from forward_position_controller/commands and publish to forward kinematics.py '''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math

class PositionCommandSubscriber(Node):
    def __init__(self):
        super().__init__('position_command_subscriber')
        self.subscription = self.create_subscription(Float64MultiArray,'forward_position_controller/commands',self.listener_callback,10)
        self.subscription = self.create_subscription(Float64MultiArray,'/obj_position',self.listener_callback,10)
        
        self.publisher_ = self.create_publisher(Float64MultiArray, 'inverse_kinematics/inputs', 10)

    def listener_callback(self, msg):
        if len(msg.data) == 4:
            joint_positions = {
                "Joint 1": msg.data[0],
                "Joint 2": msg.data[1],
                "Joint 3": msg.data[2],
                "Joint 4": msg.data[3],
                "Joint 5": msg.data[4],
                "Joint 6": msg.data[5],
            }
            print("Joint 1 - ",math.degrees(msg.data[0]))
            print("Joint 2 - ",-math.degrees(msg.data[1]))
            print("Joint 3 - ",-math.degrees(msg.data[2]))
            print("Joint 4 - ",-math.degrees(msg.data[3]))
            print("Joint 5 - ",msg.data[4])
            print("Joint 6 - ",msg.data[5])

            theta1 = msg.data[0]
            theta2 = -(msg.data[1])
            theta3 = -(msg.data[2])
            theta4 = -(msg.data[3])
            theta5 = msg.data[4]

            H1 = np.array([[math.cos(theta1),0,math.sin(theta1),0],
               [math.sin(theta1),0,-math.cos(theta1),0],
               [0,1,0,0.077],
               [0,0,0,1]])

            H2 = np.array([[math.cos(theta2),-math.sin(theta2),0,0],
                        [math.sin(theta2),math.cos(theta2),0,0],
                        [0,0,1,0],
                        [0,0,0,1]])

            H2T = np.array([[1,0,0,0.024],
                        [0,1,0,0.128],
                        [0,0,1,0],
                        [0,0,0,1]])


            H3 = np.array([[math.cos(theta3),-math.sin(theta3),0,0.124*math.cos(theta3)],
                        [math.sin(theta3),math.cos(theta3),0,0.124*math.sin(theta3)],
                        [0,0,1,0],
                        [0,0,0,1]])

            H4 = np.array([[math.cos(theta4),0,-math.sin(theta4),0.126*math.cos(theta4)],
                        [math.sin(theta4),0,math.cos(theta4),0.126*math.sin(theta4)],
                        [0,-1,0,0],
                        [0,0,0,1]])
            

            ObjDistance_X = msg.Data[0]  
            ObjDistance_Y = msg.Data[1] 
            ObjDistance_Z = msg.Data[2]
            ObjDistance_dist = msg.Data[3]  

            

            HCam = np.array([[0,0,0,ObjDistance_X],
                             [0,0,0,ObjDistance_Y],
                             [0,0,0,ObjDistance_Z],
                             [0,0,0,1]])

            H12 = np.dot(H1,H2)
            H12T=np.dot(H12,H2T)
            H123 = np.dot(H12T,H3)
            H1234 = np.dot(H123,H4)
            H1234Cam = np.dot(H1234,HCam)

            EEX = H1234Cam[0, 3]
            EEY = H1234Cam[1, 3]
            EEZ = H1234Cam[2, 3]

            if theta5>0.01:
                GRIPPER = 1.0
            else:
                GRIPPER = 0.0

            print("EEX - ",EEX)
            print("EEY - ",EEY)
            print("EEZ - ",EEZ)
            print("Gripper - ",GRIPPER)

            msg_to_publish = Float64MultiArray()
            msg_to_publish.info = [EEX,EEY,EEZ,GRIPPER]
            self.publisher_.publish(msg_to_publish)

        
        else:
            print("ERROR RECIEVING VALUES ")

    
    


def main(args=None):
    rclpy.init(args=args)
    node = PositionCommandSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
