### Camera Claiberation 
from __future__ import print_function # Python 2/3 compatibility 
import cv2 # Import the OpenCV library to enable computer vision
import numpy as np # Import the NumPy scientific computing library
# Chessboard dimensions
number_of_squares_X = 10 # Number of chessboard squares along the x-axis
number_of_squares_Y = 7  # Number of chessboard squares along the y-axis
nX = number_of_squares_X - 1 # Number of interior corners along x-axis
nY = number_of_squares_Y - 1 # Number of interior corners along y-axis
square_size = 0.025 # Size, in meters, of a square side 
# Set termination criteria. We stop either when an accuracy is reached or when
# we have finished a certain number of iterations.
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 
# Define real world coordinates for points in the 3D coordinate frame
# Object points are (0,0,0), (1,0,0), (2,0,0) ...., (5,8,0)
object_points_3D = np.zeros((nX * nY, 3), np.float32)  
# These are the x and y coordinates                                              
object_points_3D[:,:2] = np.mgrid[0:nY, 0:nX].T.reshape(-1, 2) 
object_points_3D = object_points_3D * square_size
# Store vectors of 3D points for all chessboard images (world coordinate frame)
object_points = []
# Store vectors of 2D points for all chessboard images (camera coordinate frame)
image_points = []
def main():
    IP_Cam="http://172.20.10.2:8080/video"
    cap = cv2.VideoCapture(IP_Cam)
    ret, image = cap.read()
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
    # Find the corners on the chessboard
    success, corners = cv2.findChessboardCorners(gray, (nY, nX), None)
    # If the corners are found by the algorithm, draw them
    if success == True:
      # Append object points
        object_points.append(object_points_3D)
      # Find more exact corner pixels       
        corners_2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)       
      # Append image points
        image_points.append(corners_2)
      # Draw the corners
        cv2.drawChessboardCorners(image, (nY, nX), corners_2, success)
      # Display the image. Used for testing.
        cv2.imshow("Image", image) 
        cv2.waitKey(2000)
      # Display the window for a short period. Used for testing.
                                                                                                                      
  # Perform camera calibration to return the camera matrix, distortion coefficients, rotation and translation vectors etc 
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, 
                                                    image_points, 
                                                    gray.shape[::-1], 
                                                    None, 
                                                    None)
  # Display key parameter outputs of the camera calibration process
    print("Camera matrix:") 
    print(mtx) 
    print("\n Distortion coefficient:") 
    print(dist) 
  # Close all windows
    cv2.destroyAllWindows() 
      
if __name__ == '__main__':
  print(__doc__)
  main()
