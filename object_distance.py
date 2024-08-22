import numpy as np
import cv2
import torch

# Define object-specific variables
dist = 34
focal = 665
width = 7.1

# Function to find the distance from the camera
def get_dist(rectangle_params):
    pixels = rectangle_params[1][0]
    dist = (width * focal) / pixels
    return dist

# Function to draw distance and coordinates on the image
def annotate_image(image, bbox, dist, coordinates):
    x1, y1, x2, y2 = bbox
    # Draw bounding box
    cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
    # Draw label with coordinates and distance
    label = f'Coords: ({x1}, {y1}), ({x2}, {y2})'
    distance_label = f'Distance: {dist:.2f} cm'
    cv2.putText(image, label, (x1, y1 - 30), font, fontScale, color, thickness, cv2.LINE_AA)
    cv2.putText(image, distance_label, (x1, y1 - 10), font, fontScale, color, thickness, cv2.LINE_AA)
    return image

# Load YOLO model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Load YOLOv5 model
model.eval()

# Extract frames
cap = cv2.VideoCapture(0)

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
        break

    # Perform object detection
    results = model(img)
    detections = results.xyxy[0].cpu().numpy()  # Get detections in numpy array format

    for *xyxy, conf, cls in detections:
        # Convert coordinates from float to int
        x1, y1, x2, y2 = map(int, xyxy)
        bbox = (x1, y1, x2, y2)
        bbox_width = x2 - x1

        # Calculate distance
        dist = get_dist(((0, 0), (bbox_width, 0), 0))

        # Annotate image with coordinates and distance
        img = annotate_image(img, bbox, dist, (x1, y1, x2, y2))

        # Print coordinates and distance to command line
        print(f'Object Coordinates: ({x1}, {y1}), ({x2}, {y2})')
        print(f'Distance from Camera: {dist:.2f} cm')

    cv2.imshow('Object Dist Measure', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
