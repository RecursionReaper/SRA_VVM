#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/command', 10)
        self.timer = self.create_timer(1.0, self.publish_trajectory)  # Adjust frequency as needed

    def publish_trajectory(self):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper', 'gripper_sub']

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0
        traj.points.append(point)

        point = JointTrajectoryPoint()
        point.positions = [1.0, 1.0, 1.0, 1.0, 0.5, 0.5]
        point.time_from_start.sec = 10
        point.time_from_start.nanosec = 0
        traj.points.append(point)

        self.publisher.publish(traj)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
