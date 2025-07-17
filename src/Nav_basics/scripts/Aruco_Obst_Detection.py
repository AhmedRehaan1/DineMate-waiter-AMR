#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from math import atan2, sqrt, pow, pi
from tf.transformations import euler_from_quaternion

class IntegratedNavigation:
    def __init__(self):
        # ROS setup
        rospy.init_node('integrated_navigation', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Camera setup
        self.camera_matrix = np.array([
            [1.07139173e+03, 0.00000000e+00, 9.72592860e+02],
            [0.00000000e+00, 1.05703915e+03, 6.36955649e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ], dtype=np.float32)
        self.dist_coeffs = np.array([[0.31466559, -1.09088039, 0.03388564, -0.00394274, 2.31947725]])
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.get_obstacle_ids()
        self.cap = cv2.VideoCapture("http://172.20.10.7:8080/video")

        # Navigation variables
        self.goal = Point()
        self.current_pose = Point()
        self.current_pose.z = 0.0  # yaw
        self.obstacle_detected = False
        self.closest_obstacle = {'distance': float('inf'), 'angle': 0, 'id': -1}

        # Control parameters
        self.linear_tolerance = 0.1
        self.angular_tolerance = 0.1
        self.kp_linear = 1.5
        self.kp_angular = 6
        self.safe_distance = 0.4
        self.rate = rospy.Rate(200)

    def get_obstacle_ids(self):
        while not rospy.is_shutdown():
            try:
                obstacle_ids_input = input("Enter ArUco Goal ID ")
                self.obstacle_ids = [int(id.strip()) for id in obstacle_ids_input.split(',')]
                if not self.obstacle_ids:
                    raise ValueError
                rospy.loginfo(f"goinig to goal with ID: {self.obstacle_ids}")
                break
            except ValueError:
                rospy.logerr("Invalid input! Please enter comma-separated number")

    def draw_dashed_line(self, img, pt1, pt2, color, thickness=1, dash_length=10):
        dist = int(np.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1]))
        for i in range(0, dist, dash_length * 2):
            start = (int(pt1[0] + (pt2[0] - pt1[0]) * i / dist), 
                    int(pt1[1] + (pt2[1] - pt1[1]) * i / dist))
            end = (int(pt1[0] + (pt2[0] - pt1[0]) * (i + dash_length) / dist),
                   int(pt1[1] + (pt2[1] - pt1[1]) * (i + dash_length) / dist))
            cv2.line(img, start, end, color, thickness)

    def update_pose(self, data):
        self.current_pose.x = data.pose.pose.position.x
        self.current_pose.y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_pose.z = yaw

    def get_distance(self):
        return sqrt(pow((self.goal.x - self.current_pose.x), 2) + pow((self.goal.y - self.current_pose.y), 2))

    def get_orientation(self):
        return atan2(self.goal.y - self.current_pose.y, self.goal.x - self.current_pose.x)

    def check_obstacles(self):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logerr("Frame capture error")
            return False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        
        self.closest_obstacle = {'distance': float('inf'), 'angle': 0, 'id': -1}
        obstacle_detected = False
        
        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if marker_id in self.obstacle_ids:
                    corner = corners[i]
                    cv2.aruco.drawDetectedMarkers(frame, [corner], ids[i:i+1])
                    
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corner, 0.05, self.camera_matrix, self.dist_coeffs)
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                    
                    distance = np.linalg.norm(tvec)
                    angle = np.degrees(np.arctan2(tvec[0][0][0], tvec[0][0][2]))
                    center = tuple(np.mean(corner[0], axis=0).astype(int))
                    
                    # Update closest obstacle info
                    if distance < self.closest_obstacle['distance']:
                        self.closest_obstacle = {
                            'distance': distance,
                            'angle': angle,
                            'id': marker_id
                        }
                    
                    # Draw detection info
                    cv2.putText(frame, f"ID:{marker_id} {distance:.2f}m {angle:.2f}°",
                                (center[0] + 10, center[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    self.draw_dashed_line(frame, (frame.shape[1]//2, frame.shape[0]//2), center, (0, 255, 0), 2)
                    
                    if distance < self.safe_distance:
                        obstacle_detected = True
                        rospy.loginfo(f"Obstacle detected! ID: {marker_id}, Distance: {distance:.2f}m, Angle: {angle:.2f}°")
        
        # Display CmdVel info
        cv2.putText(frame, f"CmdVel X: {self.cmd_vel_msg.linear.x:.1f}, Z: {self.cmd_vel_msg.angular.z:.1f}", 
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Obstacle Detection", frame)
        cv2.waitKey(1)
        return obstacle_detected

    def navigate(self):
        self.cmd_vel_msg = Twist()  # Initialize cmd_vel_msg
        
        while not rospy.is_shutdown():
            # Check for obstacles
            self.obstacle_detected = self.check_obstacles()
            
            if self.obstacle_detected:
                # Obstacle avoidance behavior
                self.cmd_vel_msg.linear.y = 1.0
                #self.cmd_vel_msg.angular.z = 0.5
                rospy.loginfo(f"going to ID {self.closest_obstacle['id']} at {self.closest_obstacle['distance']:.2f}m")
            else:
                # Goal seeking behavior
                distance_to_goal = self.get_distance()
                if distance_to_goal > self.linear_tolerance:
                    desired_angle = self.get_orientation()
                    angle_error = (desired_angle - self.current_pose.z + pi) % (2*pi) - pi
                    
                    self.cmd_vel_msg.linear.x = self.kp_linear * distance_to_goal
                    self.cmd_vel_msg.angular.z = self.kp_angular * angle_error
                    
                    rospy.loginfo(f"Moving to goal")
                else:
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_msg.angular.z = 0.0
                    rospy.loginfo("Goal reached!")
                    break
            
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            self.rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        nav = IntegratedNavigation()
        while True:
            try:
                nav.goal.x = float(input("Enter goal x-coordinate: "))
                nav.goal.y = float(input("Enter goal y-coordinate: "))
                break
            except ValueError:
                print("Please enter valid numbers only!")
        nav.navigate()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nNavigation stopped by user")