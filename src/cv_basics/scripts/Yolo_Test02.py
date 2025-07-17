import cv2
import numpy as np
from ultralytics import YOLO
import threading

# Load YOLO model (nano version for faster inference)
model = YOLO('yolo11n.pt')    # Use pretrained YOLOv11 model

# Camera matrix (from calibration)
camera_matrix = np.array([[1000, 0, 640],
                          [0, 1000, 480],
                          [0, 0, 1]])
scaling_factor = 0.026  # Example scaling factor

# IP Camera Stream
ip_camera_url = "http://192.168.176.229:8080/video"  # Replace with actual URL
cap = cv2.VideoCapture(ip_camera_url)
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Multithreading for video capture
frame_queue = []
frame_lock = threading.Lock()

def capture_frames(cap, frame_queue):
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        with frame_lock:
            if len(frame_queue) < 10:  # Limit queue size
                frame_queue.append(frame)

capture_thread = threading.Thread(target=capture_frames, args=(cap, frame_queue))
capture_thread.start()

# Main loop for YOLO inference
while True:
    # Get a frame from the queue
    with frame_lock:
        if len(frame_queue) > 0:
            frame = frame_queue.pop(0)
        else:
            continue

    # Downscale frame for faster processing
    frame_resized = cv2.resize(frame, (640, 480))

    # YOLO inference
    #results = model(frame_resized, stream=True)  # Stream inference for faster processing
    classes = [0]
    results = model(frame_resized,classes=classes)
    # Draw detections
    for result in results:
        for box in result.boxes:
            bbox = box.xyxy[0].cpu().numpy()  # Bounding box
            cls = int(box.cls.cpu().numpy())  # Class index
            x_center = (bbox[0] + bbox[2]) / 2
            y_center = (bbox[1] + bbox[3]) / 2

            # Pixel to world coordinates
            pixel_coords = np.array([[x_center], [y_center], [1]])
            real_coords = np.linalg.inv(camera_matrix).dot(pixel_coords) * 100
            real_coords_cm = real_coords * scaling_factor
           
            # Draw bounding box
            cv2.rectangle(frame_resized, (int(bbox[0]), int(bbox[1])), 
                          (int(bbox[2]), int(bbox[3])), (0, 255, 0), 2)
            label = f"Class: {model.names[cls]}, Pos: ({float(real_coords_cm[0]):.2f}, {float(real_coords_cm[1]):.2f} cm)"
            print(label)
            cv2.putText(frame_resized, label, (int(bbox[0]), int(bbox[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
   
    cv2.imshow("YOLO Detection", frame_resized)
    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
