#!/usr/bin/env python3
import pickle
import cv2
import mediapipe as mp
import numpy as np
import time
import rospy
from std_msgs.msg import String

# Initialize ROS node
rospy.init_node('sign_language_detector', anonymous=True)
pub = rospy.Publisher('chatter', String, queue_size=10)
rate = rospy.Rate(200)  # 200Hz

# Load trained model
model_dict = pickle.load(open('/home/g15/catkin_ws/src/sign_recog/scripts/model.p', 'rb'))
model = model_dict['model']

# Mediapipe setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, min_detection_confidence=0.5)

# Labels for prediction
labels_dict = {0: 'A', 1: 'B', 2: 'L'}

# Define fixed rectangle for detection
rect_x1, rect_y1, rect_x2, rect_y2 = 50, 50, 400, 400  

# Variables for sign stability check
last_prediction = None
current_prediction = None
prediction_start_time = None
stability_threshold = 1.5  # seconds to consider prediction stable

cap = cv2.VideoCapture(0)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        break

    H, W, _ = frame.shape
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)

    # Draw fixed rectangle
    cv2.rectangle(frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 255, 0), 2)

    current_prediction = None  # Reset current prediction each frame

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            wrist_x = int(hand_landmarks.landmark[0].x * W)
            wrist_y = int(hand_landmarks.landmark[0].y * H)

            if rect_x1 < wrist_x < rect_x2 and rect_y1 < wrist_y < rect_y2:
                data_aux = []
                x_ = []
                y_ = []

                for landmark in hand_landmarks.landmark:
                    x_.append(landmark.x)
                    y_.append(landmark.y)

                for landmark in hand_landmarks.landmark:
                    data_aux.append(landmark.x - min(x_))
                    data_aux.append(landmark.y - min(y_))

                prediction = model.predict([np.asarray(data_aux)])
                current_prediction = labels_dict[int(prediction[0])]

                # Draw bounding box and text
                x1 = int(min(x_) * W) - 10
                y1 = int(min(y_) * H) - 10
                x2 = int(max(x_) * W) - 10
                y2 = int(max(y_) * H) - 10
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 4)
                cv2.putText(frame, current_prediction, (x1, y1 - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 3, cv2.LINE_AA)

    # Check if prediction is stable
    if current_prediction == last_prediction:
        if prediction_start_time is None:
            prediction_start_time = time.time()
        elif time.time() - prediction_start_time >= stability_threshold:
            # Prediction is stable, publish the command
            if current_prediction == 'A':
                sign = "1"
            elif current_prediction == 'B':
                sign = "2"
            elif current_prediction == 'L':
                sign = "3"
            else:
                sign = "0" 
            
            pub.publish(sign)
            rospy.loginfo(f"Published: {sign}")
    else:
        prediction_start_time = None

    last_prediction = current_prediction if current_prediction else last_prediction

    cv2.imshow('Sign Language Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    rate.sleep()

cap.release()
cv2.destroyAllWindows()
