import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import numpy as np
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

        focal = 551  # Focal length in pixels
        width = 5  # Real-world width of the object in cm

        # Function to calculate the distance from the camera
        def get_dist(rectangle_params):
            pixels = rectangle_params[1][0]
            if pixels > 0:
                dist = (width * focal) / pixels
            else:
                dist = float('inf')  # In case of zero width, assign infinity distance
            return dist

        # Function to calculate 3D coordinates
        def get_3d_coordinates(x, y, dist):
            Z = dist  # Z is the distance from the camera (depth)
            X = (x - 320) * Z / focal  # X coordinate in 3D space, adjusted for principal point
            Y = (y - 240) * Z / focal  # Y coordinate in 3D space, adjusted for principal point
            return X, Y, Z

        # Function to draw distance, coordinates, and 3D coordinates on the image
        def annotate_image(image, bbox, dist, coords_3d):
            x1, y1, x2, y2 = bbox
            X, Y, Z = coords_3d
            # Draw bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            # Draw labels with 2D coordinates and distance
            label = f'Coords: ({x1}, {y1}), ({x2}, {y2})'
            distance_label = f'Distance: {dist:.2f} cm'
            coords_3d_label = f'3D Coords: (X={X:.2f} cm, Y={Y:.2f} cm, Z={Z:.2f} cm)'
            cv2.putText(image, label, (x1, y1 - 40), font, fontScale, color, thickness, cv2.LINE_AA)
            cv2.putText(image, distance_label, (x1, y1 - 20), font, fontScale, color, thickness, cv2.LINE_AA)
            cv2.putText(image, coords_3d_label, (x1, y1), font, fontScale, color, thickness, cv2.LINE_AA)
            return image

        # Basic constants for OpenCV functions
        font = cv2.FONT_HERSHEY_SIMPLEX 
        fontScale = 0.6 
        color = (0, 0, 255) 
        thickness = 2

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
                bbox = (x1, y1, x2, y2)

                # Calculate the distance to the object
                dist = get_dist(((0, 0), (w, 0), 0))

                # Calculate the 3D coordinates
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                X, Y, Z = get_3d_coordinates(center_x, center_y, dist)

                # Annotate image with 2D coordinates, distance, and 3D coordinates
                current_frame = annotate_image(current_frame, bbox, dist, (X, Y, Z))

                # Print 2D coordinates, distance, and 3D coordinates to the command line
                print(f'Object 2D Coordinates: ({x1}, {y1}), ({x2}, {y2})')
                print(f'Distance from Camera: {dist:.2f} cm')
                print(f'3D Coordinates: (X={X:.2f} cm, Y={Y:.2f} cm, Z={Z:.2f} cm)')

        # Display the annotated image
        cv2.imshow('Object Dist Measure', current_frame)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
