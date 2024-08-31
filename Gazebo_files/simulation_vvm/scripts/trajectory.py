#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from rclpy import qos
import time
import sys

J1, J2, J3, J4 = 0.0, 0.0, 0.0, 0.0
Jc1, Jc2, Jc3, Jc4 = 0.0, 0.0, 0.0, 0.0

Trajectory = 0


def Input_function(msg):
    global J1, J2, J3, J4
    J1 = msg.data[0]
    J2 = msg.data[1]
    J3 = msg.data[2]
    J4 = msg.data[3]

def angle_callback(angle):
    global Jc1, Jc2, Jc3, Jc4
    Jc1 = angle.data[0]
    Jc2 = angle.data[1]
    Jc3 = angle.data[2]
    Jc4 = angle.data[3]

def joint_trajectory_publisher():
    global Jc1, Jc2, Jc3, Jc4, J1, J2, J3, J4


    Pos_subscription = node.create_subscription(Float64MultiArray, 'trajectory_inputs', Input_function, 10)
    angle_subscription = node.create_subscription(Float64MultiArray, 'current_angle_topic', angle_callback, 10)
    Joints = node.create_publisher(Float64MultiArray, '/forward_position_controller/commands', qos_profile=qos.qos_profile_parameter_events)
    
    joint = Float64MultiArray()
    joint.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    TIME = 5  # Total time 
    STEP_TIME = 0.05  # Time per step 50ms
    STEPS = int(TIME / STEP_TIME)  # Number of steps

    delta_J1 = (J1 - Jc1) / STEPS
    delta_J2 = (J2 - Jc2) / STEPS
    delta_J3 = (J3 - Jc3) / STEPS
    delta_J4 = (J4 - Jc4) / STEPS

    for i in range(STEPS):
        joint.data[0] = Jc1 + delta_J1 * (i + 1)
        joint.data[1] = Jc2 + delta_J2 * (i + 1)
        joint.data[2] = Jc3 + delta_J3 * (i + 1)
        joint.data[3] = Jc4 + delta_J4 * (i + 1)
        print("---------TRAJECTORY------------")
        
        Joints.publish(joint)
        
        time.sleep(STEP_TIME)

    Trajectory = 1
    

if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    global node 
    node = Node('joint_trajectory_publisher')
    node.create_subscription(Float64MultiArray, 'trajectory_input', Input_function, 10)
    node.create_timer(0.2, joint_trajectory_publisher)
    rclpy.spin(node)
    
    rclpy.shutdown()
