import cv2
import numpy as np
import glob
import os

# Define the dimensions of the checkerboard (8x6 inner corners)
CHECKERBOARD = (8, 6)

# Stop the iteration when specified accuracy (epsilon) is reached or
# when the specified number of iterations is completed.
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Vector to store 3D points for each checkerboard image
threedpoints = []

# Vector to store 2D points for each checkerboard image
twodpoints = []

# Prepare 3D points for the checkerboard (real-world coordinates)
objectp3d = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Directory containing the frames
frame_dir = 'frames'  # Replace with the path to your frames directory
images = glob.glob(os.path.join(frame_dir, '*.jpg'))  # Load all .jpg images from the directory

# Loop through each image (frame)
for filename in images:
    # Read the image
    image = cv2.imread(filename)
    if image is None:
        print(f"Error: Unable to read image {filename}. Skipping...")
        continue

    # Convert the image to grayscale
    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(
        grayColor, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    # If corners are found, refine and store them
    if ret:
        threedpoints.append(objectp3d)  # Add 3D points for this image

        # Refine the corner locations
        corners2 = cv2.cornerSubPix(grayColor, corners, (11, 11), (-1, -1), criteria)
        twodpoints.append(corners2)  # Add refined 2D points for this image

        # Draw and display the corners on the image
        image = cv2.drawChessboardCorners(image, CHECKERBOARD, corners2, ret)
        cv2.imshow('Chessboard Corners', image)
        cv2.waitKey(500)  # Wait for 500ms

# Release the window
cv2.destroyAllWindows()

# Check if we have enough points to perform calibration
if len(threedpoints) < 1:
    print("Error: Not enough valid images with detected checkerboard corners.")
    exit()

# Perform camera calibration
ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
    threedpoints, twodpoints, grayColor.shape[::-1], None, None
)

# Display the calibration results
if ret:
    print("Camera matrix (intrinsic parameters):")
    print(matrix)

    print("\nDistortion coefficients:")
    print(distortion)

    print("\nRotation Vectors:")
    print(r_vecs)

    print("\nTranslation Vectors:")
    print(t_vecs)

    # Save the calibration results to a file (optional)
    np.savez('calibration_data.npz', camera_matrix=matrix, dist_coeffs=distortion, rvecs=r_vecs, tvecs=t_vecs)
    print("\nCalibration data saved to 'calibration_data.npz'.")
else:
    print("Error: Calibration failed.")