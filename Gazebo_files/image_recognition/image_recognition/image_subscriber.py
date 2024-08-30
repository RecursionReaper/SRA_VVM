import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import sys


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/Intel_Realsense/image_raw',
            self.listener_callback,
            10)
        self.subscription 
        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving image')

        current_frame = self.br.imgmsg_to_cv2(msg)       
        cv2.imshow("Camera Feed", current_frame)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
