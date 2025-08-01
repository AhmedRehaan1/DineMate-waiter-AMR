#  Autonomous Waiter Mobile Robot

A hygienic, AI-powered, autonomous 4-wheeled mobile robot for smart restaurant service — integrating ROS, YOLOv8, ESP32, MediaPipe, voice commands, and more.



Supervised by: **DR Hossam Ammar**

---

##  Project Overview
![Image](https://github.com/user-attachments/assets/7b5050a0-cdce-46e0-90a8-8f4cf50ca321)
This project presents a smart autonomous robot that acts as a **restaurant waiter**, combining AI-based gesture/voice interaction, real-time obstacle detection using **YOLOv8**, and precise control powered by **PID tuning** and **ESP32** integration.

---

##  Technical Specifications
<img width="581" height="300" alt="Image" src="https://github.com/user-attachments/assets/059b9548-049e-49a1-810c-2bc0dd542222" />
- **Controller**: ESP32  
- **Motors**: JGY-370 + Encoders  
- **Sensors**: Load Cell, MPU6050  
- **Power**: 6x Lithium-Ion Batteries  
- **Software**: ROS Noetic, Python, OpenCV, YOLOv8, MediaPipe

PID Tuning:
- `Kp = 1.2`, `Ki = 0.5`, `Kd = 0.01`

---

##  Robot Kinematics & Control

- **Inverse Kinematics**: Maps velocity commands to wheel RPMs  
- **Forward Kinematics**: Estimates position using encoder data  
- **PID Controller**: Ensures smooth and accurate movement

---

##  Go-To-Goal Navigation
*Insert Image Here (e.g., trajectory plot)*

A Euclidean distance-based algorithm lets the robot move to a designated goal table autonomously while tracking heading and distance errors in real-time.

---

##  AI Systems

###  Sign Language Detection
*Insert Image Here (e.g., dataset samples or gestures)*

- **MediaPipe Hands** for landmark extraction  
- **Random Forest Classifier** for real-time gesture recognition (A, B, L)  
- Used for **contactless table selection**

###  YOLOv8 Obstacle Detection
*Insert Image Here (e.g., object detection frame)*

- Custom-trained on 2,500+ images  
- Detects wet floors & fallen knives  
- Dynamic steering based on obstacle position in frame  
- Proximity safety threshold: **40 cm**

---

##  Teleoperation
*Insert Image Here (e.g., teleop_keyboard usage)*

### Keyboard Control
- `W` = Forward  
- `S` = Backward  
- `A/D` = Rotate  
- `U` = Speed Up

### Voice Control (ROS Node)
*Insert Image Here (e.g., microphone/voice diagram)*

- Uses Google Speech Recognition API  
- Understands: forward, stop, turn left/right, speed up

---

##  Operational Workflow
*Insert Image Here (e.g., robot with tray in kitchen)*

1. Gesture Command Issued  
2. Tray Loaded Manually  
3. Navigates While Avoiding Obstacles  
4. Verifies Pickup via Load Cell  
5. Returns to Kitchen Automatically

---

##  Gesture-Based Scanning & Analytics
*Insert Image Here (e.g., customer scanning scene)*

- Gesture 'L' triggers 180° scanning  
- YOLO-based camera scan for customer presence  
- Gathers analytics and confirms service delivery

---

##  Mechanical Design
*Insert Image Here (e.g., chassis photos)*

- Built using **3mm MDF**  
- Includes custom-designed tray, shaft, and camera mount  
- Sturdy and easy to maintain

---

##  Electronics Hardware
*Insert Image Here (e.g., component diagram or wiring)*

| Component        | Qty |  
|------------------|-----|  
| ESP32            | 1   |  
| L298N Driver     | 2   |  
| JGY-370 Motors   | 4   |  
| MPU6050          | 1   |  
| Load Cell + HX711| 1   |  
| MG995 Servo      | 1   |  

---

##  Cost Analysis
*Insert Image Here (e.g., parts table or cost chart)*

**Total Cost**: ~7,235 EGP  
All parts sourced locally, including 3D-printed shaft and chassis.

---

##  References

1. MediaPipe Framework – Google  
2. Programming Robots with ROS – O'Reilly  
3. Google Speech-to-Text API  
4. ESP32 Technical Docs – Espressif  
5. YOLOv8 – [Ultralytics GitHub](https://github.com/ultralytics/ultralytics)

---

## 🧠 Future Improvements

- Dynamic map-based navigation (SLAM)  
- Cloud-based analytics dashboard  
- Multi-robot coordination in a smart restaurant

---

## 📎 License

This project is licensed under the [MIT License](LICENSE).

---

## 🌐 Connect With Us

For inquiries, collaborations, or feedback:  
📧 [ahmedrehan2214@gmail.com](mailto:ahmedrehan2214@gmail.com)  
🔗 [LinkedIn – Ahmed Rehan](https://www.linkedin.com/in/ahmed-rehan-080604267/)

---

