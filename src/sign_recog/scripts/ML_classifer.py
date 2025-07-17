#!/usr/bin/env python

import sys
import os
import subprocess
import rospy
import signal
from std_msgs.msg import String, Int32
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTabWidget, QGroupBox, QGridLayout, 
    QRadioButton, QTextEdit
)
from PyQt5.QtCore import QTimer, Qt, QProcess, pyqtSignal, QThread, QObject
from PyQt5.QtGui import QTextCursor

class Worker(QObject):
    output_signal = pyqtSignal(str)
    
    def __init__(self, process, name):
        super().__init__()
        self.process = process
        self.name = name
    
    def run(self):
        while True:
            output = self.process.stdout.readline()
            if output:
                self.output_signal.emit(f"[{self.name}] {output.strip()}")
            
            error = self.process.stderr.readline()
            if error:
                self.output_signal.emit(f"[{self.name} ERROR] {error.strip()}")
            
            if self.process.poll() is not None:
                break

class WaiterRobotGUI(QMainWindow):
    def __init__(self):
        super(WaiterRobotGUI, self).__init__()

        # ROS node initialization
        rospy.init_node('waiter_robot_gui', anonymous=True)

        # Dictionary to store running nodes
        self.ros_processes = {}
        self.worker_threads = []
        
        # ROS Publishers
        self.mode_pub = rospy.Publisher('/operation_mode', String, queue_size=10)
        self.teleop_pub = rospy.Publisher('/teleop_cmd', String, queue_size=10)
        self.autonomous_pub = rospy.Publisher('/autonomous_cmd', String, queue_size=10)
        self.scan_pub = rospy.Publisher('/scan_cmd', String, queue_size=10)
        
        # ROS Subscribers
        self.person_count_sub = rospy.Subscriber('/person_count', Int32, self.person_count_callback)
        
        # Current state variables
        self.current_speed = 1
        self.operation_mode = 0  # 0=Teleop, 1=Autonomous
        self.autonomous_running = False
        self.person_count = 0
        self.scanning_active = False
        self.roscore_running = False

        # Setup UI
        self.setWindowTitle("Waiter Robot Control Panel")
        self.setGeometry(100, 100, 1000, 800)
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QVBoxLayout(self.central_widget)

        self.create_status_bar()
        self.create_main_interface()
        self.create_node_management_section()

        # ROS timer to update GUI
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.update_ros_data)
        self.ros_timer.start(100)

    def person_count_callback(self, msg):
        """Handle person count updates"""
        self.person_count = msg.data
        if hasattr(self, 'person_count_label'):
            self.person_count_label.setText(f"Detected Persons: {self.person_count}")
        if hasattr(self, 'person_status'):
            self.person_status.setText(f"People: {self.person_count}")

    def create_status_bar(self):
        self.status_bar = self.statusBar()
        self.connection_status = QLabel("ROS: Disconnected")
        self.battery_status = QLabel("Battery: Unknown")
        self.current_task = QLabel("Task: Idle")
        self.mode_status = QLabel("Mode: Teleoperation")
        self.person_status = QLabel("People: 0")
        self.node_status = QLabel("Nodes: 0 running")

        self.status_bar.addWidget(self.connection_status, 1)
        self.status_bar.addWidget(self.battery_status, 1)
        self.status_bar.addWidget(self.current_task, 2)
        self.status_bar.addWidget(self.mode_status, 1)
        self.status_bar.addWidget(self.person_status, 1)
        self.status_bar.addWidget(self.node_status, 1)

    def create_main_interface(self):
        self.tab_widget = QTabWidget()
        self.main_layout.addWidget(self.tab_widget)

        # Main control tab
        self.control_tab = QWidget()
        self.tab_widget.addTab(self.control_tab, "Robot Control")
        self.setup_control_tab()

    def setup_control_tab(self):
        layout = QVBoxLayout()

        # Operation mode selection
        mode_group = QGroupBox("Operation Mode")
        mode_layout = QHBoxLayout()
        
        self.teleop_radio = QRadioButton("Teleoperation")
        self.teleop_radio.setChecked(True)
        self.teleop_radio.toggled.connect(lambda: self.set_operation_mode(0))
        
        self.autonomous_radio = QRadioButton("Autonomous")
        self.autonomous_radio.toggled.connect(lambda: self.set_operation_mode(1))
        
        mode_layout.addWidget(self.teleop_radio)
        mode_layout.addWidget(self.autonomous_radio)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        # Emergency stop
        self.emergency_stop_btn = QPushButton("EMERGENCY STOP")
        self.emergency_stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.emergency_stop_btn.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_stop_btn)

        # Teleoperation controls
        self.teleop_group = QGroupBox("Teleoperation Controls")
        teleop_layout = QVBoxLayout()

        # Keyboard diagram
        keyboard_group = QGroupBox("Keyboard Controls")
        keyboard_layout = QGridLayout()
        keyboard_layout.setSpacing(10)
        
        keyboard_layout.addWidget(QLabel("↑ (W)"), 0, 1)
        keyboard_layout.addWidget(QLabel("Forward"), 0, 2)
        
        keyboard_layout.addWidget(QLabel("← (A)"), 1, 0)
        keyboard_layout.addWidget(QLabel("Left"), 1, 1)
        keyboard_layout.addWidget(QLabel("→ (D)"), 1, 2)
        keyboard_layout.addWidget(QLabel("Right"), 1, 3)
        
        keyboard_layout.addWidget(QLabel("↓ (S)"), 2, 1)
        keyboard_layout.addWidget(QLabel("Backward"), 2, 2)
        
        keyboard_layout.addWidget(QLabel("(U)"), 3, 1)
        keyboard_layout.addWidget(QLabel("Speed Up"), 3, 2)
        
        keyboard_layout.addWidget(QLabel("(Space)"), 4, 1)
        keyboard_layout.addWidget(QLabel("Stop"), 4, 2)

        keyboard_group.setLayout(keyboard_layout)
        teleop_layout.addWidget(keyboard_group)

        # Button controls
        btn_group = QGroupBox("Button Controls")
        btn_layout = QGridLayout()
        
        self.move_forward_btn = QPushButton("Forward (W)")
        self.move_backward_btn = QPushButton("Backward (S)")
        self.turn_left_btn = QPushButton("Left (A)")
        self.turn_right_btn = QPushButton("Right (D)")
        self.stop_btn = QPushButton("Stop (Space)")
        self.speed_up_btn = QPushButton("Speed Up (U)")

        self.move_forward_btn.clicked.connect(lambda: self.send_teleop_command("forward"))
        self.move_backward_btn.clicked.connect(lambda: self.send_teleop_command("backward"))
        self.turn_left_btn.clicked.connect(lambda: self.send_teleop_command("left"))
        self.turn_right_btn.clicked.connect(lambda: self.send_teleop_command("right"))
        self.stop_btn.clicked.connect(lambda: self.send_teleop_command("stop"))
        self.speed_up_btn.clicked.connect(self.increase_speed)

        btn_layout.addWidget(self.move_forward_btn, 0, 1)
        btn_layout.addWidget(self.turn_left_btn, 1, 0)
        btn_layout.addWidget(self.stop_btn, 1, 1)
        btn_layout.addWidget(self.turn_right_btn, 1, 2)
        btn_layout.addWidget(self.move_backward_btn, 2, 1)
        btn_layout.addWidget(self.speed_up_btn, 3, 1)

        btn_group.setLayout(btn_layout)
        teleop_layout.addWidget(btn_group)
        
        # Person scanning section
        scan_group = QGroupBox("Person Scanning")
        scan_layout = QVBoxLayout()
        
        self.scan_btn = QPushButton("Start Scanning")
        self.scan_btn.setStyleSheet("background-color: blue; color: white;")
        self.scan_btn.clicked.connect(self.toggle_scanning)
        
        self.person_count_label = QLabel("Detected Persons: 0")
        self.person_count_label.setAlignment(Qt.AlignCenter)
        self.person_count_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        
        scan_layout.addWidget(self.scan_btn)
        scan_layout.addWidget(self.person_count_label)
        scan_group.setLayout(scan_layout)
        
        teleop_layout.addWidget(scan_group)
        
        self.teleop_group.setLayout(teleop_layout)
        layout.addWidget(self.teleop_group)

        # Autonomous delivery control
        self.autonomous_group = QGroupBox("Autonomous Delivery")
        autonomous_layout = QVBoxLayout()

        self.start_autonomous_btn = QPushButton("Start Delivery")
        self.start_autonomous_btn.setStyleSheet("background-color: green; color: white; font-weight: bold;")
        self.start_autonomous_btn.clicked.connect(self.start_autonomous_delivery)

        autonomous_layout.addWidget(self.start_autonomous_btn)
        self.autonomous_group.setLayout(autonomous_layout)
        layout.addWidget(self.autonomous_group)

        # Status display
        self.status_display = QLabel("Status: Ready")
        layout.addWidget(self.status_display)

        self.control_tab.setLayout(layout)
        
        # Set initial mode
        self.set_operation_mode(0)

    def create_node_management_section(self):
        """Create section for managing ROS nodes"""
        self.node_tab = QWidget()
        self.tab_widget.addTab(self.node_tab, "Node Management")
        
        layout = QVBoxLayout()

        # Node control buttons
        control_group = QGroupBox("Node Controls")
        control_layout = QVBoxLayout()
        
        # ROS Core controls
        core_group = QGroupBox("ROS Core")
        core_layout = QHBoxLayout()
        
        self.launch_core_btn = QPushButton("Launch ROS Core")
        self.launch_core_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        self.launch_core_btn.clicked.connect(self.launch_roscore)
        
        self.kill_core_btn = QPushButton("Kill ROS Core")
        self.kill_core_btn.setStyleSheet("background-color: #f44336; color: white;")
        self.kill_core_btn.clicked.connect(self.kill_roscore)
        
        core_layout.addWidget(self.launch_core_btn)
        core_layout.addWidget(self.kill_core_btn)
        core_group.setLayout(core_layout)
        control_layout.addWidget(core_group)

        # Node launch buttons
        node_btn_layout = QGridLayout()
        
        self.launch_robot_btn = QPushButton("Launch Robot Nodes")
        self.launch_robot_btn.setStyleSheet("background-color: #2196F3; color: white;")
        self.launch_robot_btn.clicked.connect(lambda: self.launch_nodes("robot"))
        
        self.kill_all_btn = QPushButton("Kill All Nodes")
        self.kill_all_btn.setStyleSheet("background-color: #f44336; color: white;")
        self.kill_all_btn.clicked.connect(self.kill_all_nodes)

        node_btn_layout.addWidget(self.launch_robot_btn, 0, 0)
        node_btn_layout.addWidget(self.kill_all_btn, 1, 1)
        
        control_layout.addLayout(node_btn_layout)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # Node status display
        status_group = QGroupBox("Node Status")
        status_layout = QVBoxLayout()
        
        self.node_status_display = QTextEdit()
        self.node_status_display.setReadOnly(True)
        self.node_status_display.setStyleSheet("background-color: black; color: white;")
        
        status_layout.addWidget(self.node_status_display)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # Node output console
        output_group = QGroupBox("Node Output")
        output_layout = QVBoxLayout()
        
        self.node_output = QTextEdit()
        self.node_output.setReadOnly(True)
        self.node_output.setStyleSheet("background-color: black; color: white;")
        
        output_layout.addWidget(self.node_output)
        output_group.setLayout(output_layout)
        layout.addWidget(output_group)

        self.node_tab.setLayout(layout)

    def log_node_status(self, message):
        """Add message to the node status display"""
        self.node_status_display.append(message)
        cursor = self.node_status_display.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.node_status_display.setTextCursor(cursor)

    def log_node_output(self, message):
        """Add message to the node output console"""
        self.node_output.append(message.strip())
        cursor = self.node_output.textCursor()
        cursor.movePosition(QTextCursor.End)
        self.node_output.setTextCursor(cursor)

    def launch_roscore(self):
        """Launch ROS core if not already running"""
        if self.roscore_running:
            self.log_node_status("ROS core is already running")
            return
            
        try:
            # Start roscore in a new process
            self.log_node_status("Launching ROS core...")
            roscore_process = subprocess.Popen(['roscore'], 
                                             stdout=subprocess.PIPE,
                                             stderr=subprocess.PIPE,
                                             universal_newlines=True,
                                             preexec_fn=os.setsid)
            
            self.ros_processes['roscore'] = roscore_process
            self.roscore_running = True
            
            # Create worker thread for output monitoring
            self.worker = Worker(roscore_process, "roscore")
            self.thread = QThread()
            self.worker.moveToThread(self.thread)
            
            # Connect signals
            self.worker.output_signal.connect(self.log_node_output)
            self.thread.started.connect(self.worker.run)
            
            # Store references
            self.worker_threads.append((self.worker, self.thread))
            
            # Start the thread
            self.thread.start()
            
            self.log_node_status("ROS core launched successfully")
            self.update_node_count()
            
        except Exception as e:
            self.log_node_status(f"Error launching ROS core: {str(e)}")

    def kill_roscore(self):
        """Kill ROS core if running"""
        if not self.roscore_running:
            self.log_node_status("ROS core is not running")
            return
            
        try:
            self.log_node_status("Shutting down ROS core...")
            if 'roscore' in self.ros_processes:
                process = self.ros_processes['roscore']
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.terminate()
                process.wait(timeout=2)
                del self.ros_processes['roscore']
            
            self.roscore_running = False
            self.log_node_status("ROS core terminated")
            self.update_node_count()
            
        except Exception as e:
            self.log_node_status(f"Error killing ROS core: {str(e)}")

    def launch_nodes(self, node_group):
        """Launch a group of ROS nodes"""
        launch_commands = {
            "robot": ["rosrun", "sign_recog", "Main.py"],
        }
        
        if node_group not in launch_commands:
            self.log_node_status(f"Unknown node group: {node_group}")
            return
            
        if node_group in self.ros_processes:
            self.log_node_status(f"{node_group} nodes are already running")
            return
            
        try:
            self.log_node_status(f"Launching {node_group} nodes...")
            
            # Start the process
            process = subprocess.Popen(
                launch_commands[node_group],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1,
                preexec_fn=os.setsid
            )
            
            self.ros_processes[node_group] = process
            
            # Create worker thread for output monitoring
            self.worker = Worker(process, node_group)
            self.thread = QThread()
            self.worker.moveToThread(self.thread)
            
            # Connect signals
            self.worker.output_signal.connect(self.log_node_output)
            self.thread.started.connect(self.worker.run)
            
            # Store references
            self.worker_threads.append((self.worker, self.thread))
            
            # Start the thread
            self.thread.start()
            
            self.log_node_status(f"{node_group} nodes launched successfully")
            self.update_node_count()
            
        except Exception as e:
            self.log_node_status(f"Error launching {node_group} nodes: {str(e)}")

    def kill_all_nodes(self):
        """Terminate all running ROS nodes"""
        self.log_node_status("Terminating all nodes...")
        
        # First try to kill nodes gracefully
        for name, process in list(self.ros_processes.items()):
            try:
                if process.poll() is None:  # Process is still running
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.terminate()
                    process.wait(timeout=2)
                    self.log_node_status(f"Terminated {name}")
            except Exception as e:
                self.log_node_status(f"Error terminating {name}: {str(e)}")
            finally:
                if name in self.ros_processes:
                    del self.ros_processes[name]
        
        # Clean up worker threads
        for worker, thread in self.worker_threads:
            thread.quit()
            thread.wait()
        self.worker_threads = []
        
        # Reset roscore flag
        self.roscore_running = False
        
        self.update_node_count()

    def toggle_scanning(self):
        self.scanning_active = not self.scanning_active
        if self.scanning_active:
            self.scan_btn.setText("Stop Scanning")
            self.scan_btn.setStyleSheet("background-color: orange; color: white;")
            scan_msg = String()
            scan_msg.data = "start_scan"
            self.scan_pub.publish(scan_msg)
            self.status_display.setText("Status: Scanning for people...")
        else:
            self.scan_btn.setText("Start Scanning")
            self.scan_btn.setStyleSheet("background-color: blue; color: white;")
            scan_msg = String()
            scan_msg.data = "stop_scan"
            self.scan_pub.publish(scan_msg)
            self.status_display.setText("Status: Scanning stopped")

    def set_operation_mode(self, mode):
        self.operation_mode = mode
        mode_msg = String()
        
        if mode == 0:  # Teleoperation
            self.mode_status.setText("Mode: Teleoperation")
            self.teleop_group.setEnabled(True)
            self.autonomous_group.setEnabled(False)
            mode_msg.data = "teleop"
            self.mode_pub.publish(mode_msg)
            rospy.loginfo("Switched to TELEOPERATION mode")
        else:  # Autonomous
            self.mode_status.setText("Mode: Autonomous")
            self.teleop_group.setEnabled(False)
            self.autonomous_group.setEnabled(True)
            mode_msg.data = "autonomous"
            self.mode_pub.publish(mode_msg)
            rospy.loginfo("Switched to AUTONOMOUS mode")

    def keyPressEvent(self, event):
        if self.operation_mode == 1:  # Ignore keys in autonomous mode
            return
            
        key = event.key()
        
        if key == Qt.Key_W:
            self.send_teleop_command("forward")
        elif key == Qt.Key_S:
            self.send_teleop_command("backward")
        elif key == Qt.Key_A:
            self.send_teleop_command("left")
        elif key == Qt.Key_D:
            self.send_teleop_command("right")
        elif key == Qt.Key_U:
            self.increase_speed()
        elif key == Qt.Key_Space:
            self.send_teleop_command("stop")
        else:
            super(WaiterRobotGUI, self).keyPressEvent(event)

    def increase_speed(self):
        if self.operation_mode == 1:
            return
            
        self.current_speed += 1
        rospy.loginfo("Speed increased")
        self.status_display.setText("Status: Speed increased")
        speed_msg = String()
        speed_msg.data = f"speed_{self.current_speed}"
        self.teleop_pub.publish(speed_msg)

    def send_teleop_command(self, command):
        if self.operation_mode == 1:
            return
            
        rospy.loginfo(f"Sending teleop command: {command}")
        self.status_display.setText(f"Status: {command.capitalize()}")
        cmd_msg = String()
        cmd_msg.data = command
        self.teleop_pub.publish(cmd_msg)

    def start_autonomous_delivery(self):
        if self.operation_mode == 0:
            return
            
        rospy.loginfo("Starting autonomous delivery")
        self.status_display.setText("Status: Starting Autonomous Delivery")
        self.autonomous_running = True
        cmd_msg = String()
        cmd_msg.data = "start"
        self.autonomous_pub.publish(cmd_msg)

    def emergency_stop(self):
        rospy.loginfo("Emergency stop activated")
        self.status_display.setText("Status: Emergency Stop Activated")
        
        # Publish emergency stop to both systems
        emergency_msg = String()
        emergency_msg.data = "emergency_stop"
        self.teleop_pub.publish(emergency_msg)
        self.autonomous_pub.publish(emergency_msg)

    def update_ros_data(self):
        if rospy.is_shutdown():
            self.connection_status.setText("ROS: Disconnected")
        else:
            self.connection_status.setText("ROS: Connected")

    def update_node_count(self):
        """Update the count of running nodes in status bar"""
        count = len(self.ros_processes)
        self.node_status.setText(f"Nodes: {count} running")

    def closeEvent(self, event):
        """Clean up when closing the application"""
        # Stop autonomous operation if running
        if self.autonomous_running:
            cmd_msg = String()
            cmd_msg.data = "stop"
            self.autonomous_pub.publish(cmd_msg)
            
        # Stop scanning if active
        if self.scanning_active:
            scan_msg = String()
            scan_msg.data = "stop_scan"
            self.scan_pub.publish(scan_msg)
            
        # Kill all running nodes
        self.kill_all_nodes()
        
        # Shutdown ROS
        rospy.signal_shutdown("GUI closed")
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)

    try:
        gui = WaiterRobotGUI()
        gui.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass