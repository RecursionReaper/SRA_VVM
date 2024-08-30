#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from rclpy import qos
import math
import sys

def direct_angle_publisher():
    Joints = node.create_publisher(Float64MultiArray, 'trajectory_input',qos_profile=qos.qos_profile_parameter_events)
    joint = Float64MultiArray()
    joint.data = [0.0,0.0,0.0,0.0]

    joint.data[0]=math.radians(float(input("ENTER JOINT 1 ANGLE- ")))
    joint.data[1]=math.radians(float(input("ENTER JOINT 2 ANGLE- ")))
    joint.data[2]=math.radians(float(input("ENTER JOINT 3 ANGLE- ")))
    joint.data[3]=math.radians(float(input("ENTER JOINT 4 ANGLE- ")))
    # joint.data[4]=float(input("Entey Gripper L -"))
    # joint.data[5]=float(input("Entey Gripper R -"))

    
    Joints.publish(joint)

if __name__ == '__main__':

    rclpy.init(args=sys.argv)
    global node 
    node = Node('direct_angle_publisher')
    node.create_timer(0.2, direct_angle_publisher)
    rclpy.spin(node)
    rclpy.shutdown()