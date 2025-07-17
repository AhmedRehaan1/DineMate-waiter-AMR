#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import threading
import queue
from geometry_msgs.msg import Twist, Pose
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from ultralytics import YOLO

class TurtleBotObstacleDetector:
    def __init__(self):
        # ROS setup
        rospy.init_node('GOtoGOal_YOLO', anonymous=True)
        
        # Velocity publisher
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Pose subscriber
        self.pose_subscriber = rospy.Subscriber('/pose', Pose, self.update_pose)
        
        self.pose = Pose()
        self.goal = Pose()
        self.goal.x = 2.0  # Fixed goal: x=3
        self.goal.y = 1.0  # Fixed goal: y=3
        
        # YOLO model setup
        self.model = YOLO('yolov8n.pt')
        self.DISTANCE_THRESHOLD = 0.9  # 0.9 meters threshold
        
        # Camera calibration (simplified)
        self.camera_matrix = np.array([
            [1000, 0, 640],
            [0, 1000, 480],
            [0, 0, 1]
        ], dtype=np.float32)
        self.scaling_factor = 0.026  # Converts pixels to real-world distance
        
        # Video capture
        self.cap = cv2.VideoCapture(0)  # Use 0 for webcam or IP camera URL
        self.frame_queue = queue.Queue(maxsize=10)
        self.stop_event = threading.Event()
        
        # Start capture thread
        self.capture_thread = threading.Thread(
            target=self.capture_frames,
            daemon=True
        )
        self.capture_thread.start()
        
        # Velocity message
        self.cmd_vel_msg = Twist()
        self.rate = rospy.Rate(200)  # Reduced to 10Hz for stability

    def capture_frames(self):
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                try:
                    self.frame_queue.put(frame, timeout=1.0)
                except queue.Full:
                    continue

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
        return atan2(goal_pose.y , goal_pose.x)

    def angular_vel(self, goal_pose, constant=1):
        """PID-like angular velocity control."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def draw_dashed_line(self, img, pt1, pt2, color, thickness=1, dash_length=10):
        """Draws a dashed line between two points."""
        dist = int(np.hypot(pt2[0]-pt1[0], pt2[1]-pt1[1]))
        for i in range(0, dist, dash_length*2):
            start = (int(pt1[0] + (pt2[0]-pt1[0])*i/dist), 
                    int(pt1[1] + (pt2[1]-pt1[1])*i/dist))
            end = (int(pt1[0] + (pt2[0]-pt1[0])*(i+dash_length)/dist), 
                  int(pt1[1] + (pt2[1]-pt1[1])*(i+dash_length)/dist))
            cv2.line(img, start, end, color, thickness)

    def obstacle_detection(self):
        """Detects obstacles using YOLO and returns avoidance status."""
        try:
            frame = self.frame_queue.get(timeout=1.0)
        except queue.Empty:
            return 0  # No obstacle if no frame
        
        frame_resized = cv2.resize(frame, (640, 480))
        frame_width = frame_resized.shape[1]
        
        # YOLO detection (classes: 0: person, 11: stop sign, 43: bottle)
        results = self.model(frame_resized, classes=[11, 43])
        
        obstacle_status = 0  # 0: no obstacle, 1: left, 2: right
        
        for result in results:
            for box in result.boxes:
                bbox = box.xyxy[0].cpu().numpy()
                x_center = (bbox[0] + bbox[2]) / 2
                
                # Simplified distance estimation
                bbox_width = bbox[2] - bbox[0]
                distance = (1.0 / bbox_width) * 10  # Inverse proportional
                
                if distance < self.DISTANCE_THRESHOLD:
                    if x_center < frame_width // 2:
                        obstacle_status = 1  # Left obstacle
                        cv2.rectangle(frame_resized, 
                                     (int(bbox[0]), int(bbox[1])),
                                     (int(bbox[2]), int(bbox[3])), 
                                     (0, 255, 0), 2)
                    else:
                        obstacle_status = 2  # Right obstacle
                        cv2.rectangle(frame_resized, 
                                     (int(bbox[0]), int(bbox[1])),
                                     (int(bbox[2]), int(bbox[3])), 
                                     (0, 0, 255), 2)
        
        # Display status
        status_text = ["Clear", "Left Obstacle!", "Right Obstacle!"]
        cv2.putText(frame_resized, status_text[obstacle_status],
                   (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        cv2.imshow("Obstacle Detection", frame_resized)
        if cv2.waitKey(1) == ord('q'):
            return None
        
        return obstacle_status

    def move2goal(self):
        """Main control loop to reach (3, 3) with obstacle avoidance."""
        distance_tolerance = 0.1
        
        rospy.loginfo(f"Moving to fixed goal: ({self.goal.x}, {self.goal.y})")
        
        while not rospy.is_shutdown():
            obstacle_status = self.obstacle_detection()
            if obstacle_status is None:  # 'q' pressed
                break
            
            if self.euclidean_distance(self.goal) >= distance_tolerance:
                # Obstacle avoidance logic
                if obstacle_status == 1:  # Left obstacle
                    self.cmd_vel_msg.angular.z = self.linear_vel(self.goal)
                    self.cmd_vel_msg.linear.y = 1.0  # Move right
                    self.cmd_vel_msg.angular.z = self.angular_vel(self.goal)
                    rospy.loginfo("Avoiding LEFT obstacle")
                elif obstacle_status == 2:  # Right obstacle
                    self.cmd_vel_msg.angular.x = self.linear_vel(self.goal)
                    self.cmd_vel_msg.linear.y = 2  # Move left
                    self.cmd_vel_msg.angular.z = self.angular_vel(self.goal)
                    rospy.loginfo("Avoiding RIGHT obstacle")
                else:  # No obstacle
                    self.cmd_vel_msg.linear.x = self.linear_vel(self.goal)
                    self.cmd_vel_msg.linear.y = 0.0
                    self.cmd_vel_msg.angular.z = self.angular_vel(self.goal)
                
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            else:
                rospy.loginfo("Reached goal at (3, 3)!")
                break
            
            self.rate.sleep()

        # Cleanup
        self.stop_event.set()
        self.capture_thread.join()
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
