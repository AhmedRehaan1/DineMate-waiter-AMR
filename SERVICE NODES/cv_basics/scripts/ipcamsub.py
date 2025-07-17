from ultralytics import YOLO
import cv2

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam")
    exit()

print("Starting webcam stream...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    results = model(frame)[0]

    for box in results.boxes:
        cls_id = int(box.cls[0])
        class_name = model.names[cls_id]
        conf = box.conf[0]

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{class_name} {conf:.2f}"
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("YOLOv8 Live Detection", frame)

    key = cv2.waitKey(1)
    if key == ord('q'):
        print("Quitting...")
        break

cap.release()
cv2.destroyAllWindows()
