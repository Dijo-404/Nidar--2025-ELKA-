#!/usr/bin/env python3
import glob
import cv2
from ultralytics import YOLO

# Find all available best.pt weights files
weights = sorted(glob.glob('/home/dijo404/yolo_train/yolov8n_human_best/yolo-human-detection2/weights/best.pt'))
print("Available weights:", weights)

if not weights:
    raise FileNotFoundError("No trained weights found in 'runs/detect/train*/weights/best.pt'")

# Load the latest trained model
model = YOLO(weights[-1])
print(f"Loaded model weights from: {weights[-1]}")

# Replace with your RTSP stream URL
rtsp_url = 'rtsp://192.168.144.25:8554/main.264'

# Open RTSP stream
cap = cv2.VideoCapture(rtsp_url)

if not cap.isOpened():
    raise RuntimeError("Failed to open RTSP stream")

print("Starting real-time detection. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame, exiting...")
        break

    # Run inference on the frame
    results = model(frame)

    # Annotate the frame with bounding boxes
    annotated_frame = results[0].plot()

    # Display the annotated frame
    cv2.imshow("YOLOv8 RTSP Detection", annotated_frame)

    # Quit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
