#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from rclpy import qos
import math
import sys

L1 = 0.077
L2z = 0.128
L2x = 0.024
L2 = 0.130
L3 = 0.124
L4 = 0.126

def seperate_angle_publisher():
    Joints = node.create_publisher(Float64MultiArray, 'trajectory_inputs',qos_profile=qos.qos_profile_parameter_events)
    joint = Float64MultiArray()
    joint.data = [0.0,0.0,0.0,0.0]

    EEX = float(input("ENTER X COORDINATE - "))
    EEY = float(input("ENTER Y COORDINATE - "))
    EEZ = float(input("ENTER Z COORDINATE - "))

    alpha = math.atan((EEZ - L1)/(math.sqrt((EEX * EEX) + (EEY * EEY))))

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
    # print(end_effector_matrix)


    orientation_matrix = end_effector_matrix[:3, :3]


    if EEX>=0 and EEZ>=0 and ((EEX*EEX)+((EEZ-L1)*(EEZ-L1))+(EEY*EEY)) <= (L2+L3+L4)*(L2+L3+L4): # This is to check if the point is accessible 

        if EEX==0 :
            T1 = 1.5708
        else:
            T1 = math.atan(EEY/EEX)

        Calc_Var_A = (EEX/math.cos(T1))- (L4*math.cos(alpha))
        Calc_Var_B = (EEZ-L1)-(L4*math.sin(alpha))

        T3 = math.asin(((Calc_Var_A*Calc_Var_A)+(Calc_Var_B*Calc_Var_B)-(L3*L3)-(L2*L2))/(2*L2*L3)) - math.atan(L2x/L2z)

        Calc_Var_C1 = (L3*math.sin(T3)+L2*math.cos(math.atan(L2x/L2z)))
        Calc_Var_C2 = (L3*math.cos(T3)+L2*math.sin(math.atan(L2x/L2z)))

        Calc_Var_A_Bar = (Calc_Var_A)/(math.sqrt((Calc_Var_C1*Calc_Var_C1)+(Calc_Var_C2*Calc_Var_C2)))
        Calc_Var_B_Bar = (Calc_Var_B)/(math.sqrt((Calc_Var_C1*Calc_Var_C1)+(Calc_Var_C2*Calc_Var_C2)))

        T2 = math.atan(Calc_Var_C2/Calc_Var_C1) - math.atan(Calc_Var_A_Bar/Calc_Var_B_Bar)

        T4 = alpha - T2 - T3

        if math.degrees(T1)<91 and math.degrees(T1)>-91 and math.degrees(T2)<91 and math.degrees(T2)>-91 and math.degrees(T3)<91 and math.degrees(T3)>-91 and math.degrees(T4)<91 and math.degrees(T4)>-91:
            print("Theta 1 is equal to - ",math.degrees(T1))
            print("Theta 2 is equal to - ",-math.degrees(T2))
            print("Theta 3 is equal to - ",-math.degrees(T3))
            print("Theta 4 is equal to - ",-math.degrees(T4))
            joint.data[0]=T1
            joint.data[1]=-T2
            joint.data[2]=-T3
            joint.data[3]=-T4
            
            Var = input("Gripper 1/0 : ")
            if Var == 1 :
                joint.data[4]=0.019
            if Var == 0:
                joint.data[4]=0.013

            Joints.publish(joint)

        else :
            print("This point is not reachable but in accessible space ")   
        
    else :
        print("This point is not in accessible space")

   


if __name__ == '__main__':

    rclpy.init(args=sys.argv)
    global node 
    node = Node('seperate_angle_publisher')
    node.create_timer(0.2, seperate_angle_publisher)
    rclpy.spin(node)
    rclpy.shutdown()