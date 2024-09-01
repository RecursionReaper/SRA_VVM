import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImageProcessorPublisher(Node):

    def __init__(self):
        super().__init__('image_processor_publisher')
        self.subscription = self.create_subscription(
            Image,
            '/Intel_Realsense/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float64MultiArray, '/obj_position', 10)
        self.br = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Processing image')

        current_frame = self.br.imgmsg_to_cv2(msg)
        
        focal = 551  # Focal length in pixels
        width = 5  # Real-world width of the object in cm

        def get_dist(rectangle_params):
            pixels = rectangle_params[1][0]
            if pixels > 0:
                dist = (width * focal) / pixels
            else:
                dist = float('inf')
            return dist

        def get_3d_coordinates(x, y, dist):
            Z = dist
            X = (x - 320) * Z / focal
            Y = (y - 240) * Z / focal
            return X, Y, Z

        def annotate_image(image, bbox, coords_3d):
            x1, y1, x2, y2 = bbox
            X, Y, Z = coords_3d
            # Draw bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Draw labels with 2D coordinates and 3D coordinates
            label_2d = f'2D: ({x1}, {y1}), ({x2}, {y2})'
            label_3d = f'3D: (X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}) cm'
            cv2.putText(image, label_2d, (x1, y1 - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(image, label_3d, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            return image

        # Define the range of red color in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Create masks for detecting red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)

        # Find contours in the masked image
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # Filter small areas to reduce noise
                x1, y1, w, h = cv2.boundingRect(cnt)
                x2, y2 = x1 + w, y1 + h

                # Calculate the distance to the object
                dist = get_dist(((0, 0), (w, 0), 0))

                # Calculate the 3D coordinates
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                X, Y, Z = get_3d_coordinates(center_x, center_y, dist)

                # Annotate the image
                current_frame = annotate_image(current_frame, (x1, y1, x2, y2), (X, Y, Z))

                # Publish the 3D coordinates
                msg = Float64MultiArray()
                msg.data = [X, Y, Z]
                self.publisher_.publish(msg)

                self.get_logger().info(f'Published coordinates: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}')

                # Break after processing the first valid contour
                break

        cv2.imshow("Camera Feed", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_processor_publisher = ImageProcessorPublisher()
    rclpy.spin(image_processor_publisher)
    image_processor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()