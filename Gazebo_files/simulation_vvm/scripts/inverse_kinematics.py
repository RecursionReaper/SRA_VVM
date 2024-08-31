#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
import math
import sys

# Global variables
EEX = 0.274
EEY = 0.0
EEZ = 0.205
GRIPPER = 0 

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_publisher')

        self.create_subscription(Float64MultiArray, 'inverse_kinematics/inputs', self.position_callback, 10)
        self.joints_publisher = self.create_publisher(Float64MultiArray, 'trajectory_inputs', 10)
        self.timer = self.create_timer(0.2, self.inverse_kinematics_publisher)
        
    def position_callback(self, msg):
        global EEX, EEY, EEZ
        EEX = msg.data[0]
        EEY = msg.data[1]
        EEZ = msg.data[2]

    def inverse_kinematics_publisher(self):
        joint = Float64MultiArray()
        joint.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Link lengths
        L1 = 0.077
        L2z = 0.128
        L2x = 0.024
        L2 = 0.130
        L3 = 0.124
        L4 = 0.126

        if (math.sqrt((EEX * EEX) + (EEY * EEY))) == 0:
            alpha = math.pi
        else:
            alpha = math.atan((EEZ - L1) / (math.sqrt((EEX * EEX) + (EEY * EEY))))

        translation_matrix = np.array([
            [1, 0, 0, EEX],
            [0, 1, 0, EEY],
            [0, 0, 1, EEZ - L1],
            [0, 0, 0, 1]
        ])

        rotation_matrix = np.array([
            [math.cos(alpha), -math.sin(alpha), 0, 0],
            [math.sin(alpha), math.cos(alpha), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        end_effector_matrix = np.dot(translation_matrix, rotation_matrix)
        orientation_matrix = end_effector_matrix[:3, :3]

        if EEX >= 0 and EEZ >= 0 and ((EEX * EEX) + ((EEZ - L1) * (EEZ - L1)) + (EEY * EEY)) <= (L2 + L3 + L4) * (L2 + L3 + L4):
            T1 = math.atan(EEY / EEX)

            Calc_Var_A = (EEX / math.cos(T1)) - (L4 * math.cos(alpha))
            Calc_Var_B = (EEZ - L1) - (L4 * math.sin(alpha))

            T3 = math.asin(((Calc_Var_A * Calc_Var_A) + (Calc_Var_B * Calc_Var_B) - (L3 * L3) - (L2 * L2)) / (2 * L2 * L3)) - math.atan(L2x / L2z)

            Calc_Var_C1 = (L3 * math.sin(T3) + L2 * math.cos(math.atan(L2x / L2z)))
            Calc_Var_C2 = (L3 * math.cos(T3) + L2 * math.sin(math.atan(L2x / L2z)))

            Calc_Var_A_Bar = (Calc_Var_A) / (math.sqrt((Calc_Var_C1 * Calc_Var_C1) + (Calc_Var_C2 * Calc_Var_C2)))
            Calc_Var_B_Bar = (Calc_Var_B) / (math.sqrt((Calc_Var_C1 * Calc_Var_C1) + (Calc_Var_C2 * Calc_Var_C2)))

            T2 = math.atan(Calc_Var_C2 / Calc_Var_C1) - math.atan(Calc_Var_A_Bar / Calc_Var_B_Bar)

            T4 = alpha - T2 - T3

            if -91 < math.degrees(T1) < 91 and -91 < math.degrees(T2) < 91 and -91 < math.degrees(T3) < 91 and -91 < math.degrees(T4) < 91:
                print("-------------------------------------- ")
                print("Theta 1 is equal to - ", math.degrees(T1))
                print("Theta 2 is equal to - ", math.degrees(T2))
                print("Theta 3 is equal to - ", math.degrees(T3))
                print("Theta 4 is equal to - ", math.degrees(T4))
                joint.data[0] = T1
                joint.data[1] = -T2
                joint.data[2] = -T3
                joint.data[3] = -T4
            else:
                print("This point is not reachable but in accessible space")
                joint.data[0] = 0.0
                joint.data[1] = 0.0
                joint.data[2] = 0.0
                joint.data[3] = 0.0

        else:
            print("This point is not in accessible space")
            joint.data[0] = 0.0
            joint.data[1] = 0.0
            joint.data[2] = 0.0
            joint.data[3] = 0.0

        # if GRIPPER == 1:
        #     # joint.data[4] = 0.019
        #     # joint.data[5] = 0.019
        # else:
        #     # joint.data[4] = 0.0
        #     # joint.data[5] = 0.0
        self.joints_publisher.publish(joint)
    
if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = InverseKinematicsNode()

    rclpy.spin(node)
    rclpy.shutdown()
