import cv2
import os

# Create a directory to save the frames
output_dir = "frames"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Initialize video capture
cap = cv2.VideoCapture(0)  # 0 is the default camera, you can change it if you have multiple cameras

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video capture.")
    exit()

frame_count = 0

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Display the resulting frame
    cv2.imshow('Frame', frame)

    # Save the frame as an image file
    frame_filename = os.path.join(output_dir, f"frame_{frame_count:04d}.jpg")
    cv2.imwrite(frame_filename, frame)
    print(f"Saved {frame_filename}")

    frame_count += 1

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

print(f"Finished recording. Total frames saved: {frame_count}")