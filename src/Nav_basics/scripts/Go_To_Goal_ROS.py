#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import threading
import queue
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import String
from math import pow, atan2, sqrt
from ultralytics import YOLO
import time

class TurtleBotObstacleDetector:
    def __init__(self):
        # ROS setup
        rospy.init_node('GOtoGOal_YOLO', anonymous=True)
        
        # Velocity publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Pose subscriber
        self.pose_subscriber = rospy.Subscriber('/pose', Pose, self.update_pose)
        
        # Sign language command subscriber
        self.sign_subscriber = rospy.Subscriber('chatter', String, self.sign_callback)
        
        self.pose = Pose()
        self.goal = Pose()
        self.sign_active = False
        
        # YOLO model setup
        self.model = YOLO('yolov8n.pt')
        self.DISTANCE_THRESHOLD = 170  # 90 cm threshold
        self.CONFIDENCE_THRESHOLD = 0.5  # Minimum confidence for detection
        
        # Distance calculation parameters
        self.known_width_cm = 19  # Known width of target object in cm
        self.focal_length = 700   # Camera focal length
        
        # Video capture
        self.ip_webcam_url = "http://192.168.35.70:8080/video"
        self.cap = cv2.VideoCapture(self.ip_webcam_url)
        
        # Camera settings
        self.set_camera_properties()
        
        # Frame processing
        self.frame_queue = queue.Queue(maxsize=2)  # Smaller queue for fresher frames
        self.processed_frame = None
        self.frame_lock = threading.Lock()
        self.stop_event = threading.Event()
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0
        
        # Start threads
        self.capture_thread = threading.Thread(
            target=self.capture_frames,
            daemon=True
        )
        self.capture_thread.start()
        
        self.processing_thread = threading.Thread(
            target=self.process_frames,
            daemon=True
        )
        self.processing_thread.start()
        
        # Velocity message
        self.cmd_vel_msg = Twist()
        self.rate = rospy.Rate(200)

    def set_camera_properties(self):
        """Configure camera settings for better performance"""
        if self.cap.isOpened():
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # Adjust camera parameters (if supported)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
            self.cap.set(cv2.CAP_PROP_FOCUS, 50)    # Set fixed focus
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
            self.cap.set(cv2.CAP_PROP_CONTRAST, 0.5)
            self.cap.set(cv2.CAP_PROP_SATURATION, 0.5)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

    def sign_callback(self, msg):
        """Handle sign language commands"""
        sign = msg.data
        
        if sign == "1":   # Sign 'A' detected
            self.goal.x = 1.9
            self.goal.y = 1.9
            self.sign_active = True
            rospy.loginfo("Sign A detected - Setting goal to (3, 3)")
        elif sign == "2":  # Sign 'B' detected
            self.goal.x = 3.2
            self.goal.y = 3.2
            self.sign_active = True
            rospy.loginfo("Sign B detected - Setting goal to (6, 6)")
        elif sign == "3":  # Sign 'L' detected
            self.goal.x = 1
            self.goal.y = 1
            self.sign_active = True
            rospy.loginfo("Sign L detected - Setting goal to (8, 8)")
        elif sign == "0":  # No sign detected
            self.sign_active = False
            rospy.loginfo("No sign detected - Waiting for command")

    def capture_frames(self):
        """Thread for capturing frames from camera"""
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                # Pre-process frame (resize, convert color space)
                frame = cv2.resize(frame, (640, 480))
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                try:
                    # Clear queue if full to get newest frame
                    if self.frame_queue.full():
                        self.frame_queue.get_nowait()
                    self.frame_queue.put(frame, timeout=0.5)
                except queue.Full:
                    continue
            else:
                rospy.logwarn("Failed to capture frame")
                time.sleep(0.1)

    def process_frames(self):
        """Thread for processing frames with YOLO"""
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                frame = self.frame_queue.get(timeout=1.0)
                
                # Calculate FPS
                self.frame_count += 1
                if time.time() - self.last_fps_time >= 1.0:
                    self.fps = self.frame_count
                    self.frame_count = 0
                    self.last_fps_time = time.time()
                
                # Run YOLO detection (only on every 2nd frame to reduce load)
                if self.frame_count % 2 == 0:
                    results = self.model(frame, classes=[11, 43], 
                                       conf=self.CONFIDENCE_THRESHOLD, 
                                       verbose=False)
                    
                    # Process results
                    processed_frame = self.draw_detections(frame.copy(), results)
                    
                    with self.frame_lock:
                        self.processed_frame = processed_frame
                
            except queue.Empty:
                continue

    def draw_detections(self, frame, results):
        """Draw detection results on frame"""
        frame_width = frame.shape[1]
        obstacle_status = 0
        
        for result in results:
            for box in result.boxes:
                bbox = box.xyxy[0].cpu().numpy()
                conf = box.conf[0].cpu().numpy()
                cls_id = box.cls[0].cpu().numpy()
                
                # Calculate distance in cm
                pixel_width = bbox[2] - bbox[0]
                distance_cm = (self.known_width_cm * self.focal_length) / pixel_width
                
                if distance_cm < self.DISTANCE_THRESHOLD:
                    x_center = (bbox[0] + bbox[2]) / 2
                    
                    if x_center < frame_width // 2:
                        obstacle_status = 1  # Left obstacle
                        color = (0, 255, 0)  # Green
                    else:
                        obstacle_status = 2  # Right obstacle
                        color = (0, 0, 255)  # Red
                    
                    # Draw bounding box
                    cv2.rectangle(frame, 
                                (int(bbox[0]), int(bbox[1])),
                                (int(bbox[2]), int(bbox[3])), 
                                color, 2)
                    
                    # Draw label
                    label = f"{distance_cm:.1f}cm ({conf:.2f})"
                    cv2.putText(frame, label, 
                              (int(bbox[0]), int(bbox[1]) - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        
        # Display status and FPS
        status_text = ["Clear", "Left Obstacle!", "Right Obstacle!"]
        cv2.putText(frame, status_text[obstacle_status],
                   (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.putText(frame, f"FPS: {self.fps}", 
                   (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        return frame, obstacle_status

    def update_pose(self, data):
        """Updates current robot pose."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Calculates distance to goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1):
        """PID-like linear velocity control."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """Calculates heading angle to goal."""
        return atan2(goal_pose.y , goal_pose.x )

    def angular_vel(self, goal_pose, constant=1):
        """PID-like angular velocity control."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def obstacle_detection(self):
        """Returns the latest obstacle status and shows processed frame."""
        obstacle_status = 0
        
        with self.frame_lock:
            if self.processed_frame is not None:
                frame, obstacle_status = self.processed_frame
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("Obstacle Detection", frame)
                if cv2.waitKey(1) == ord('q'):
                    return None
        
        return obstacle_status

    def move2goal(self):
        """Main control loop to reach goal with obstacle avoidance."""
        distance_tolerance = 0.1
        
        while not rospy.is_shutdown():
            if not self.sign_active:
                rospy.loginfo("Waiting for sign command...")
                rospy.sleep(1)
                continue
                
            rospy.loginfo(f"Moving to goal: ({self.goal.x}, {self.goal.y})")
            
            obstacle_status = self.obstacle_detection()
            if obstacle_status is None:  # 'q' pressed
                break
            
            if self.euclidean_distance(self.goal) >= distance_tolerance:
                # Obstacle avoidance logic
                if obstacle_status == 1:  # Left obstacle
                    self.cmd_vel_msg.angular.z = self.linear_vel(self.goal)
                    self.cmd_vel_msg.linear.y = 2.0  # Move right
                    self.cmd_vel_msg.angular.z = self.angular_vel(self.goal)
                    rospy.loginfo("Avoiding LEFT obstacle")
                elif obstacle_status == 2:  # Right obstacle
                    self.cmd_vel_msg.angular.x = self.linear_vel(self.goal)
                    self.cmd_vel_msg.linear.y = 1  # Move left
                    self.cmd_vel_msg.angular.z = self.angular_vel(self.goal)
                    rospy.loginfo("Avoiding RIGHT obstacle")
                else:  # No obstacle
                    self.cmd_vel_msg.linear.x = self.linear_vel(self.goal)
                    self.cmd_vel_msg.linear.y = 0.0
                    self.cmd_vel_msg.angular.z = self.angular_vel(self.goal)
                
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            else:
                rospy.loginfo(f"Reached goal at ({self.goal.x}, {self.goal.y})!")
                self.sign_active = False
                break
            
            self.rate.sleep()

        # Cleanup
        self.stop_event.set()
        self.capture_thread.join()
        self.processing_thread.join()
        self.cap.release()
        cv2.destroyAllWindows()
        
        # Stop the robot
        self.cmd_vel_msg.linear.x = 0
        self.cmd_vel_msg.linear.y = 0
        self.cmd_vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

if __name__ == '__main__':
    try:
        bot = TurtleBotObstacleDetector()
        bot.move2goal()
    except rospy.ROSInterruptException:
        pass
