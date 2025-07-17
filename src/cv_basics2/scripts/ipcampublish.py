#!/usr/bin/env python3
import cv2

ip_camera_url = "http://192.168.100.57:8080/video"
cap = cv2.VideoCapture(ip_camera_url)

if not cap.isOpened():
    print("Error: Cannot open IP camera stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to retrieve frame.")
        break

    cv2.imshow("IP Camera Test", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
