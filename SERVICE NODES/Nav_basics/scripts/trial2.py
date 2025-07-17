#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys
import tty
import termios

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def keyboard_publisher():
    rospy.init_node('keyboard_publisher', anonymous=True)
    pub = rospy.Publisher('keyboard_input', String, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz
    
    rospy.loginfo("Press W, A, or D to publish messages. Press Q to quit.")
    
    while not rospy.is_shutdown():
        key = get_key()
        
        if key.lower() == 'w':
            pub.publish("W")
            rospy.loginfo("Published: W - Forward signal")
        elif key.lower() == 'a':
            pub.publish("A")
            rospy.loginfo("Published: A - Left signal")
        elif key.lower() == 'd':
            pub.publish("D")
            rospy.loginfo("Published: D - Right signal")
        elif key.lower() == 'q':
            rospy.loginfo("Quitting...")
            break
            
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_publisher()
    except rospy.ROSInterruptException:
        pass