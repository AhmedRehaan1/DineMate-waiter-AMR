#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_publisher():
    # Initialize the ROS node
    rospy.init_node('wifi_cmd_vel_publisher', anonymous=True)

    # Create a publisher on the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Set the publishing rate (Hz)
    rate = rospy.Rate(1)  # 1 Hz

    # Create and fill a Twist message
    msg = Twist()
    msg.linear.x = 1       # meters per second
    msg.angular.z = 0.4     # radians per second

    rospy.loginfo("Starting to publish /cmd_vel...")

    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo(f"Published -> linear.x: {msg.linear.x} | angular.z: {msg.angular.z}")
        rate.sleep()

if __name__ == 'main':
    try:
        cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass