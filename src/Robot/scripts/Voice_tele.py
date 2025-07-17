#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import speech_recognition as sr
from collections import OrderedDict
import time

class VoiceControlPublisher:
    def __init__(self):
        rospy.init_node('voice_control_publisher', log_level=rospy.INFO)

        self.pub = rospy.Publisher('motor_command', Int32, queue_size=1)
        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = False  
        self.recognizer.pause_threshold = 0.5  
        
        self.microphone = sr.Microphone()
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.3) 
    
        self.command_map = OrderedDict([
            ('stop', 0),
            ('forward', 1),
            ('turn right', 2),
            ('turn left', 3),
            ('speed up', 4)
        ])
        
        # Pre-compiled data structures for faster matching
        self.command_words = {cmd: set(cmd.split()) for cmd in self.command_map.keys()}
        self.min_command_length = min(len(cmd) for cmd in self.command_map.keys())
        
        # Performance monitoring
        self.last_command_time = time.time()
        self.command_count = 0

    def recognize_speech(self):
        """Optimized speech recognition with timeout handling"""
        try:
            with self.microphone as source:
                audio = self.recognizer.listen(
                    source, 
                    timeout=2, 
                    phrase_time_limit=2  
                )
            start_time = time.time()
            text = self.recognizer.recognize_google(audio, language="en-US").lower()
            rospy.logdebug(f"Recognition took {time.time()-start_time:.3f}s")
            return text
        except sr.WaitTimeoutError:
            return None
        except (sr.UnknownValueError, sr.RequestError) as e:
            rospy.logdebug(f"Recognition error: {str(e)}")
            return None
        except Exception as e:
            rospy.logwarn(f"Unexpected error: {str(e)}")
            return None

    def process_command(self, text):
        """Ultra-fast command processing"""
        if not text or len(text) < self.min_command_length:
            return None
            
        text_words = set(text.split())
        
        for cmd, val in self.command_map.items():
            if cmd in text:
                return val
                
        for cmd, words in self.command_words.items():
            if words.issubset(text_words):
                return self.command_map[cmd]
                
        return None

    def run(self):
        """Main loop with performance optimizations"""
        rate = rospy.Rate(30)  
        consecutive_failures = 0
        
        while not rospy.is_shutdown():
            try:
                spoken_text = self.recognize_speech()
                
                if spoken_text:
                    command_val = self.process_command(spoken_text)
                    if command_val is not None:
                        self.pub.publish(command_val)
                        self.command_count += 1
                        current_time = time.time()
                        rospy.loginfo(
                            f"Command: {command_val} | "
                            f"Rate: {self.command_count/(current_time-self.last_command_time):.1f} cmd/s"
                        )
                        self.last_command_time = current_time
                    else:
                        rospy.logdebug(f"Ignored: '{spoken_text}'")
                        consecutive_failures += 1
                        if consecutive_failures > 3:
                            rate = rospy.Rate(30) 
                else:
                    consecutive_failures += 1
                    
                # Dynamic rate adjustment
                if consecutive_failures > 5:
                    rate = rospy.Rate(35)
                elif consecutive_failures == 0:
                    rate = rospy.Rate(30)  
                    
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in main loop: {str(e)}")
                rate.sleep()

if __name__ == '__main__':
    try:
        controller = VoiceControlPublisher()
        rospy.loginfo("Voice control node started with optimized performance")
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")