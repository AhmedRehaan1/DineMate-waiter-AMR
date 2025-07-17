import cv2
from ultralytics import YOLO
# Load the YOLO model (replace with your model path if custom-trained)
model = YOLO("yolo11n.pt")  # YOLOv8 pre-trained model
# IP camera stream URL
ip_camera_url = "http://192.168.100.57:8080/video"  # Replace with actual URL
# Open the IP camera stream
cap = cv2.VideoCapture(ip_camera_url)
if not cap.isOpened():
    print("Error: Could not open IP camera stream.")
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("CAM Error")
        break
    # Resize frame for faster processing (optional)
    frame_resized = cv2.resize(frame, (640, 480))
    # Perform YOLO detection
    results = model(frame_resized)
    # Draw detections on the frame
    annotated_frame = results[0].plot()
    # Display the frame
    cv2.imshow("YOLOv8 IP Camera Detection", annotated_frame)
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release resources
cap.release()
cv2.destroyAllWindows()
