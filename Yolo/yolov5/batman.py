import numpy as np
import cv2
import torch

# Define camera parameters and object-specific variables
focal = 665  # Focal length in pixels
width = 7.1  # Real-world width of the object in cm

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

# Load YOLO model only once
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
model.eval()

# Change video source to read from file
cap = cv2.VideoCapture('/home/oreki/Downloads/senpai.mp4')

# Basic constants for OpenCV functions
font = cv2.FONT_HERSHEY_SIMPLEX 
fontScale = 0.6 
color = (0, 0, 255) 
thickness = 2

cv2.namedWindow('Object Dist Measure', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Object Dist Measure', 700, 600)

# Loop to capture video frames
while True:
    ret, img = cap.read()
    if not ret:
        print("End of video or failed to grab frame.")
        break

    # Resize the image for YOLO model input (if necessary)
    # Optional: img = cv2.resize(img, (640, 480))  # Resize to desired resolution for YOLO

    # Perform object detection
    results = model(img)
    detections = results.xyxy[0].cpu().numpy()  # Get detections in numpy array format

    for *xyxy, conf, cls in detections:
        # Convert coordinates from float to int
        x1, y1, x2, y2 = map(int, xyxy)
        bbox = (x1, y1, x2, y2)
        bbox_width = x2 - x1

        # Calculate the distance to the object
        dist = get_dist(((0, 0), (bbox_width, 0), 0))

        # Calculate the 3D coordinates
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        X, Y, Z = get_3d_coordinates(center_x, center_y, dist)

        # Annotate image with 2D coordinates, distance, and 3D coordinates
        img = annotate_image(img, bbox, dist, (X, Y, Z))

        # Print 2D coordinates, distance, and 3D coordinates to the command line
        print(f'Object 2D Coordinates: ({x1}, {y1}), ({x2}, {y2})')
        print(f'Distance from Camera: {dist:.2f} cm')
        print(f'3D Coordinates: (X={X:.2f} cm, Y={Y:.2f} cm, Z={Z:.2f} cm)')

    # Display the image
    cv2.imshow('Object Dist Measure', img)

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
