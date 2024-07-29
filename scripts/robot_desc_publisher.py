#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy import qos
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import subprocess

class RoboDescPublisher(Node):
    def __init__(self):
        super().__init__("robo_description_publisher")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(String,"/robot_description",qos_profile)
        
        # Reading URDF file
        package_description = "simulation_vvm"  
        urdf_file = 'open_manipulator_robot.urdf.xacro'     
        urdf_path = os.path.join(get_package_share_directory(package_description),"urdf",urdf_file)

        print("#############"+str(urdf_path))
        
        # urdf_content = self.process_xacro(urdf_path)
        # self.robot_description = urdf_content

        self.robot_description = urdf_path

        print(f"Looking for URDF file at: {urdf_path}")
        print(f"Does the file exist? {os.path.exists(urdf_path)}")

        with open(urdf_path, 'r') as file:
            self.robot_description = file.read()

        self.msg = String()
        self.msg.data = self.robot_description
        self.pub.publish(self.msg)
        self.get_logger().info('Published robot description')

        # Publish robot description periodically
        # self.timer = self.create_timer(1.0, self.publish_description)

    def process_xacro(self,xacro_file):
        cmd = ['xacro', xacro_file]
        return subprocess.check_output(cmd).decode('utf-8')


    def publish_description(self):
        msg = String()
        msg.data = self.robot_description
        self.pub.publish(msg)
        self.get_logger().info('Published robot description')
        print("##########################################################")
        print(f"URDF content: {self.robot_description[:100]}...")  # Print first 100 characters


def main(args=None):
    rclpy.init(args=args)
    node = RoboDescPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()