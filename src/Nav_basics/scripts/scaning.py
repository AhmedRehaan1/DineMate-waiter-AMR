#!/usr/bin/env python3

import cv2
from ultralytics import YOLO
import rospy
from std_msgs.msg import Int32, String
import threading
import time
import torch

class PersonCounter:
    def __init__(self):
        rospy.init_node('person_counter', anonymous=True)
        
        # Configuration
        self.webcam_url = "http://192.168.35.7:8080/video"
        self.model_weights = 'yolov8n.pt'
        self.target_fps = 15
        self.timeout_duration = 30.0
        self.activation_sign = "3"
        
        # ROS Communication
        self.count_pub = rospy.Publisher('person_count', Int32, queue_size=10)
        rospy.Subscriber('chatter', String, self.sign_callback)
        
        # Tracking variables
        self.activated = False
        self.activation_time = 0
        self.unique_people = set()  # Stores all unique person IDs seen
        self.current_count = 0      # Current people in frame (for visualization)
        self.frame_lock = threading.Lock()
        self.current_frame = None
        
        # Model setup
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = YOLO(self.model_weights).to(self.device)
        self.model.fuse()
        
        # Start threads
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.processing_thread = threading.Thread(target=self.process_frames)
        self.capture_thread.daemon = True
        self.processing_thread.daemon = True
        
        rospy.on_shutdown(self.cleanup)
        rospy.loginfo(f"Person counter initialized. Waiting for '{self.activation_sign}' sign...")

    def sign_callback(self, msg):
        if msg.data == self.activation_sign:
            if not self.activated:
                rospy.loginfo("Activation signal received! Starting person counter...")
                self.unique_people.clear()  # Reset counter on new activation
            self.activated = True
            self.activation_time = time.time()

    def capture_frames(self):
        cap = cv2.VideoCapture(self.webcam_url)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, self.target_fps)
        
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                with self.frame_lock:
                    self.current_frame = frame
            else:
                rospy.logwarn("Failed to capture frame. Reconnecting...")
                cap.release()
                time.sleep(1)
                cap = cv2.VideoCapture(self.webcam_url)
        
        cap.release()

    def process_frames(self):
        while not rospy.is_shutdown():
            current_time = time.time()
            
            # Handle activation timeout
            if self.activated and (current_time - self.activation_time > self.timeout_duration):
                rospy.loginfo(f"Counter deactivated. Total unique people: {len(self.unique_people)}")
                self.count_pub.publish(len(self.unique_people))
                self.activated = False
            
            if not self.activated:
                time.sleep(0.1)
                continue
            
            # Get frame
            with self.frame_lock:
                if self.current_frame is None:
                    time.sleep(0.01)
                    continue
                frame = self.current_frame.copy()
            
            # Process frame
            try:
                results = self.model.track(
                    frame,
                    persist=True,
                    classes=[0],  # Only person class
                    verbose=False,
                    imgsz=320,
                    device=self.device,
                    conf=0.5
                )
                
                # Update unique people count
                if results[0].boxes.id is not None:
                    ids = results[0].boxes.id.cpu().numpy().astype(int)
                    self.unique_people.update(ids)  # Add new IDs to set
                    self.current_count = len(ids)   # For visualization only
                
                # Visualize (optional)
                self.visualize(frame, results)
                
            except Exception as e:
                rospy.logerr(f"Processing error: {str(e)}")
            
            time.sleep(1.0/self.target_fps)  # Control processing rate

    def visualize(self, frame, results):
        # Draw bounding boxes
        if results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.cpu().numpy()
            ids = results[0].boxes.id.cpu().numpy().astype(int)
            
            for box, id in zip(boxes, ids):
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'ID: {id}', (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Status overlay
        status = "ACTIVE" if self.activated else "WAITING"
        color = (0, 255, 0) if self.activated else (0, 0, 255)
        
        cv2.putText(frame, f'Status: {status}', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(frame, f'Current: {self.current_count}', (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.putText(frame, f'Total: {len(self.unique_people)}', (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        if self.activated:
            remaining = max(0, self.timeout_duration - (time.time() - self.activation_time))
            cv2.putText(frame, f'Time left: {remaining:.1f}s', (20, 160),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        cv2.imshow('Person Counter', frame)
        cv2.waitKey(1)

    def cleanup(self):
        cv2.destroyAllWindows()
        rospy.loginfo("Cleanly shutting down person counter")

    def run(self):
        self.capture_thread.start()
        self.processing_thread.start()
        rospy.spin()

if __name__ == '__main__':
    try:
        counter = PersonCounter()
        counter.run()
    except rospy.ROSInterruptException:
        pass
