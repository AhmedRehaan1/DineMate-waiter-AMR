#!/usr/bin/env python
import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import os
import threading

class ROSControlPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("Waiter Robot")
        self.root.geometry("800x650")  # Increased height for ESP32 button
        self.root.configure(bg='#2c3e50')

        # Initialize nodes configuration
        self.nodes = {
            "teleop": {
                "package": "Robot",
                "node": "RosNode.py",
                "path": "/home/g15/catkin_ws/src/Robot/scripts",
                "description": "Manual robot control mode\nwith Voice commands/keyboard input"
            },
            "autonomous": [
                {
                    "package": "sign_recog",
                    "node": "Main.py",
                    "path": "/home/g15/catkin_ws/src/sign_recog/scripts",
                    "description": "Sign recognition system\nfor autonomous navigation"
                },
                {
                    "package": "Nav_basics",
                    "node": "Go_To_Goal_ROS.py",
                    "path": "/home/g15/catkin_ws/src/Nav_basics/scripts",
                    "description": "Path planning and\ngoal-oriented navigation"
                },
                {
                    "package": "Nav_basics",
                    "node": "scaning.py",
                    "path": "/home/g15/catkin_ws/src/Nav_basics/scripts",
                    "description": "Environment scaning\nand obstacle detection"
                },
            ]
        }

        # ROS configuration
        self.ros_distro = "noetic"
        self.setup_file = f"/opt/ros/{self.ros_distro}/setup.bash"
        self.workspace = "/home/g15/catkin_ws"
        self.workspace_file = f"{self.workspace}/devel/setup.bash"

        self.setup_styles()
        self.create_widgets()

    def setup_styles(self):
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('.', background='#2c3e50', foreground='white')
        
        # Button styles
        self.style.configure('Teleop.TButton',
                           font=('Arial', 14, 'bold'),
                           padding=15,
                           background='#3498db',
                           foreground='white')
        self.style.map('Teleop.TButton',
                      background=[('active', '#2980b9')])
        
        self.style.configure('Auto.TButton',
                           font=('Arial', 14, 'bold'),
                           padding=15,
                           background='#2ecc71',
                           foreground='white')
        self.style.map('Auto.TButton',
                      background=[('active', '#27ae60')])
        
        self.style.configure('Esp32.TButton',
                           font=('Arial', 14, 'bold'),
                           padding=10,
                           background='#e67e22',
                           foreground='white')
        self.style.map('Esp32.TButton',
                      background=[('active', '#d35400')])
        
        # Frame styles
        self.style.configure('TFrame', background='#2c3e50')
        self.style.configure('Header.TFrame', background='#34495e')
        self.style.configure('Content.TFrame', background='#34495e')

    def create_widgets(self):
        # Header frame
        header_frame = ttk.Frame(self.root, style='Header.TFrame')
        header_frame.pack(fill='x', padx=10, pady=10)
        
        # Title
        title_frame = ttk.Frame(header_frame, style='Header.TFrame')
        title_frame.pack(side='left', fill='y')
        
        tk.Label(title_frame, 
               text="Waiter Robot", 
               font=('Arial', 24, 'bold'),
               bg='#34495e',
               fg='white').pack(anchor='w')
        
        # Content frame
        content_frame = ttk.Frame(self.root, style='Content.TFrame')
        content_frame.pack(fill='both', expand=True, padx=20, pady=20)
        
        # Mode selection
        mode_frame = ttk.Frame(content_frame, style='Content.TFrame')
        mode_frame.pack(fill='both', expand=True, pady=20)
        
        # Teleoperation card
        teleop_card = ttk.Frame(mode_frame, style='Content.TFrame')
        teleop_card.pack(side='left', fill='both', expand=True, padx=10)
        
        ttk.Button(teleop_card,
                 text="TELEOPERATION MODE",
                 style='Teleop.TButton',
                 command=lambda: self.run_node("teleop")).pack(fill='both', expand=True)
        
        tk.Label(teleop_card, 
               text=self.nodes["teleop"]["description"],
               font=('Arial', 12),
               bg='#34495e',
               fg='#bdc3c7').pack(pady=10)
        
        # Autonomous card
        auto_card = ttk.Frame(mode_frame, style='Content.TFrame')
        auto_card.pack(side='left', fill='both', expand=True, padx=10)
        
        ttk.Button(auto_card,
                 text="AUTONOMOUS MODE",
                 style='Auto.TButton',
                 command=lambda: self.run_node("autonomous")).pack(fill='both', expand=True)
        
        tk.Label(auto_card, 
               text="Sign recognition + Navigation system",
               font=('Arial', 12),
               bg='#34495e',
               fg='#bdc3c7').pack(pady=10)
        
        # ESP32 connection button
        esp32_frame = ttk.Frame(content_frame, style='Content.TFrame')
        esp32_frame.pack(fill='x', pady=10)
        
        ttk.Button(esp32_frame,
                 text="ESP32 WiFi CONNECTION",
                 style='Esp32.TButton',
                 command=self.connect_esp32).pack(fill='x')
        
        tk.Label(esp32_frame, 
               text="Connect to ESP32 via WiFi\nusing rosserial over TCP",
               font=('Arial', 12),
               bg='#34495e',
               fg='#bdc3c7').pack(pady=5)
        
        # Status bar
        status_frame = ttk.Frame(self.root, style='Header.TFrame')
        status_frame.pack(fill='x', padx=10, pady=10)
        
        self.status_label = tk.Label(status_frame,
                                   text="System ready | Select operation mode",
                                   font=('Arial', 10),
                                   bg='#34495e',
                                   fg='#95a5a6')
        self.status_label.pack(side='left', padx=10)

    def run_node(self, mode):
        self.status_label.config(text=f"Initializing {mode.upper()} mode...")
        
        try:
            if mode == "teleop":
                config = self.nodes["teleop"]
                self.launch_single_node(config, "Teleoperation")
                
            elif mode == "autonomous":
                for idx, config in enumerate(self.nodes["autonomous"]):
                    thread = threading.Thread(
                        target=self.launch_single_node,
                        args=(config, f"Autonomous-{idx+1}")
                    )
                    thread.start()
                
        except Exception as e:
            pass

    def connect_esp32(self):
        """Simple method to run the ESP32 connection command in a terminal"""
        try:
            command = f"""
            source {self.setup_file}
            source {self.workspace_file}
            rosrun rosserial_python serial_node.py tcp 11411
            """
            
            terminal = "gnome-terminal"
            subprocess.Popen([terminal, "--", "bash", "-c", command])
            
            self.status_label.config(text="ESP32 WiFi connection established")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect to ESP32:\n{e}")

    def launch_single_node(self, config, mode_name):
        try:
            if not os.path.exists(config["path"]):
                self.root.after(0, messagebox.showerror, 
                              "Error", f"Path not found: {config['path']}")
                return
                
            node_path = os.path.join(config["path"], config["node"])
            if not os.path.exists(node_path):
                self.root.after(0, messagebox.showerror,
                              "Error", f"Node not found: {node_path}")
                return
                
            command = f"""
            cd {self.workspace}
            source {self.setup_file}
            source {self.workspace_file}
            cd {config["path"]}
            rosrun {config['package']} {config['node']}
            if [ $? -ne 0 ]; then
                echo "[ERROR] Node failed!"
                read
            fi
            """
            
            terminal = "gnome-terminal"
            subprocess.Popen([terminal, "--", "bash", "-c", command])
            
            self.root.after(0, self.status_label.config,
                          text=f"{mode_name} node running: {config['node']}")
            
        except Exception as e:
            pass

if __name__ == "__main__":
    root = tk.Tk()
    app = ROSControlPanel(root)
    root.mainloop()