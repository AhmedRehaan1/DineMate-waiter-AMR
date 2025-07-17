#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import speech_recognition as sr
import sys
import tty
import termios
from collections import OrderedDict
import threading
import time

class UnifiedTeleopNode:
    def __init__(self):
        rospy.init_node('unified_teleop', log_level=rospy.INFO)
        
        # Publisher using original voice control topic
        self.cmd_pub = rospy.Publisher('motor_command', Int32, queue_size=1)
        
        # Voice recognition setup
        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = False
        self.recognizer.pause_threshold = 0.5
        self.microphone = sr.Microphone()
        
        # Command mappings (maintaining original voice command values)
        self.voice_commands = OrderedDict([
            ('stop', 0),
            ('forward', 1),
            ('turn right', 2),
            ('turn left', 3),
            ('speed up', 4),
            ('back', 5)
        ])
        
        # Keyboard mappings (converted to voice command values)
        self.key_commands = {
            'w': 1,  # Forward
            'a': 3,  # Left
            'd': 2,  # Right
            's': 0,  # Stop
            'u': 4,  # Speed up
            'b': 5   # Back
        }
        
        # Control state
        self.current_mode = "IDLE"
        self.last_command_time = 0
        self.command_timeout = 1.0
        
        # Start voice thread
        self.voice_active = True
        self.voice_thread = threading.Thread(target=self.voice_listener)
        self.voice_thread.start()
        
        rospy.on_shutdown(self.shutdown)

    def get_key(self):
        """Non-blocking keyboard input"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def voice_listener(self):
        """Continuous voice recognition thread"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.3)
            while self.voice_active and not rospy.is_shutdown():
                try:
                    audio = self.recognizer.listen(
                        source, 
                        timeout=1, 
                        phrase_time_limit=2
                    )
                    text = self.recognizer.recognize_google(audio).lower()
                    self.process_voice(text)
                except (sr.UnknownValueError, sr.WaitTimeoutError):
                    continue
                except Exception as e:
                    rospy.logwarn(f"Voice error: {str(e)}")

    def process_voice(self, text):
        """Handle voice commands"""
        for cmd, val in self.voice_commands.items():
            if cmd in text:
                self.current_mode = "VOICE"
                self.last_command_time = time.time()
                self.cmd_pub.publish(val)
                rospy.loginfo(f"Voice command: {cmd} ({val})")
                return

    def run(self):
        """Main control loop"""
        rospy.loginfo("Unified teleop ready. Using 'motor_command' topic")
        rospy.loginfo("Keyboard: W=1(Forward), A=3(Left), D=2(Right)")
        rospy.loginfo("S=0(Stop), U=4(Speed), B=5(Back)")
        
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # Keyboard input
            key = self.get_key()
            if key.lower() in self.key_commands:
                cmd = self.key_commands[key.lower()]
                self.current_mode = "KEYBOARD"
                self.last_command_time = time.time()
                self.cmd_pub.publish(cmd)
                rospy.loginfo(f"Keyboard command: {key} ({cmd})")
            
            # Command timeout
            if time.time() - self.last_command_time > self.command_timeout:
                if self.current_mode != "IDLE":
                    self.current_mode = "IDLE"
                    self.cmd_pub.publish(0)
                    rospy.loginfo("Command timeout - stopping")
            
            rate.sleep()

    def shutdown(self):
        """Clean shutdown"""
        self.voice_active = False
        self.voice_thread.join()
        self.cmd_pub.publish(0)

if __name__ == '__main__':
    try:
        node = UnifiedTeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
