import cv2 as cv
from cv2 import aruco
import numpy as np
import serial
import struct
from icecream import ic
from ctypes import *
import time
import can
import threading
import os
import sys
import csv

file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)
from check_sum import *



# Define camera intrinsic parameters (example values, adjust as per your camera)
fx = 962.6 # Focal length in pixels (along x-axis)
fy = 1085.46 # Focal length in pixels (along y-axis)
cx = 997.89  # Principal point (x-coordinate) in pixels
cy = 505.60  # Principal point (y-coordinate) in pixels

# Define distortion coefficients (typically obtained from camera calibration)
k1 = -0.813251508
k2 = 16.1669366
p1 = -0.0218294859
p2 = 0.0727941984
k3 = -92.6641208




# Define camera matrix and distortion coefficients
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])

dist_coeffs = np.array([k1, k2, p1, p2, k3])

# Dictionary to specify the type of the marker
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
param_markers = aruco.DetectorParameters()

# Marker size in meters (adjust according to the actual size of your marker)
marker_size = 0.1  # 10 cm in this example

# Utilize the default camera/webcam driver
# Setup camera
print("Setting up camera...")
cap = cv.VideoCapture(0)  # Adjust the index as needed

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()
else:
    print("Camera connection established.")



while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        continue

    try:
        
        # Get the frame dimensions
        height, width, _ = frame.shape

        # Calculate the center of the frame
        frame_center_x = width // 2
        frame_center_y = height // 2

        # Draw a red dot at the center of the frame
        cv.circle(frame, (frame_center_x, frame_center_y), 5, (0, 0, 255), -1)

        # Turn the frame to grayscale-only (for efficiency)
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Detect ArUco markers
        marker_corners, marker_IDs, _ = aruco.detectMarkers(gray_frame, marker_dict, parameters=param_markers)

        if marker_corners:
            for ids, corners in zip(marker_IDs, marker_corners):
                # Draw the detected markers
                cv.polylines(frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA)
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)

                # Calculate center of the marker
                center = np.mean(corners, axis=0).astype(int)
                aruco_center_x, aruco_center_y = center

                # Calculate the difference between the frame center and the marker center
                dy = aruco_center_y - frame_center_y
                dx = aruco_center_x - frame_center_x
                distance = np.sqrt(dx**2 + dy**2)

                # # Incrementally adjust pitch and yaw based on the distance
                # if distance > distance_threshold:
                #     distance_within_threshold_start_time = None
                #     if dy != 0:
                #         pitch_increment += np.degrees(np.arctan2(-dy, fy)) * 0.03# Adjust the scaling factor as needed
                #     if dx != 0:
                #         yaw_increment += np.degrees(np.arctan2(dx, fx)) * 0.03 # Adjust the scaling factor as needed

                #     pitch, yaw = set_gimbal_orientation(pitch_increment, 0, yaw_increment)
                #     print(pitch_increment)

                    
                # else:
                    
                #     if distance_within_threshold_start_time is None:
                #         distance_within_threshold_start_time = time.time()
                #     elif time.time() - distance_within_threshold_start_time > 0.5:
                #         # Calculate X and Y using the provided formula
                #         d = 500  # Example distance value; replace as needed
                #         B = np.degrees(np.arctan2(d * np.cos(np.radians(yaw_increment)), d * np.sin(np.radians(yaw_increment)) - 80))
                #         C = np.degrees(np.arctan2(d * np.cos(np.radians(yaw_increment)), d * np.sin(np.radians(yaw_increment)) + 160))
                #         X = (90 - B) % 360
                #         Y = (90 - C) % 360
                #         Z = -pitch_increment
                        

                #         message_1 = f"1 {X}\n"
                #         message_2 = f"2 {Y}\n"
                #         message3=f"{Z}\n"
                #         # try:
                #         ser.write(message_1.encode())
                #         print(f"Sent message to ESP32: {message_1.strip()}")
                #         ser.write(message_2.encode())
                #         print(f"Sent message to ESP32: {message_2.strip()}")
                        
                #         # Send message to Arduino Mega
                #         esp2.write(message3.encode())
                #         print("Sent testing message to Arduino Mega: 45")

                #         # # Reset variables to ensure re-entering the loop correctly
                #         # distance_within_threshold_start_time = None
                #         # pitch_increment = 0
                #         # yaw_increment = 0
                #         # except Exception as e:
                #         #     print(f"Failed to send message: {e}")

        # Show the frame with markers and tracking info
        cv.imshow("Frame", frame)

    except Exception as e:
        print(f"Error in processing frame: {e}")

    key = cv.waitKey(1)
    if key == ord("q"):
        break

# Release camera and close all windows
cap.release()
cv.destroyAllWindows()
print("Released the camera and destroyed all windows.")