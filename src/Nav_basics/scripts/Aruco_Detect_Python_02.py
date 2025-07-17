#!/usr/bin/env python3
import cv2
import numpy as np

# ArUco dictionary and parameters
aruco_dict =  cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()

# These should be obtained via camera calibration
camera_matrix = np.array([
    [1.07139173e+03, 0.00000000e+00, 9.72592860e+02],
    [0.00000000e+00, 1.05703915e+03, 6.36955649e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float32)

dist_coeffs = np.array([[ 0.31466559, -1.09088039,  0.03388564, -0.00394274,  2.31947725]])

# Define 3D points of the cube (relative to the marker)
marker_size = 0.05  # Marker size in meters
half_size = marker_size / 2
cube_points = np.array([
    [half_size, half_size, 0],          # Bottom-right-front
    [half_size, -half_size, 0],         # Bottom-left-front
    [-half_size, -half_size, 0],        # Top-left-front
    [-half_size, half_size, 0],         # Top-right-front
    [half_size, half_size, -marker_size],    # Bottom-right-back
    [half_size, -half_size, -marker_size],   # Bottom-left-back
    [-half_size, -half_size, -marker_size],  # Top-left-back
    [-half_size, half_size, -marker_size],   # Top-right-back
], dtype=np.float32)

# Start video capture (replace with IP camera URL if needed)
IP_Cam="http://172.20.10.2:8080/video"
cap = cv2.VideoCapture(IP_Cam)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    if ids is not None:
        # Draw detected markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Estimate pose for each detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        for rvec, tvec in zip(rvecs, tvecs):
            # Draw pose axes on the marker
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

            # Project cube points onto the image plane
            img_points, _ = cv2.projectPoints(cube_points, rvec, tvec, camera_matrix, dist_coeffs)

            # Convert to integer for drawing
            img_points = np.int32(img_points).reshape(-1, 2)

            # Draw the cube edges
            # Base of the cube
            cv2.drawContours(frame, [img_points[:4]], -1, (0, 255, 0), 2)

            # Top of the cube
            cv2.drawContours(frame, [img_points[4:]], -1, (0, 255, 0), 2)

            # Vertical edges
            for i in range(4):
                cv2.line(frame, tuple(img_points[i]), tuple(img_points[i + 4]), (255, 0, 0), 2)

    # Display the frame
    cv2.imshow("Aruco Cube", frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()