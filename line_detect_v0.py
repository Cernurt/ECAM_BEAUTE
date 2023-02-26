import cv2
import numpy as np

# Capture video from default camera
cap = cv2.VideoCapture(0)

# Define the parameters for the edge detection algorithm
canny_thresh1 = 80
canny_thresh2 = 100

# Define the parameters for the line detection algorithm
rho = 1
theta = np.pi/180
threshold = 80
min_line_length = 100
max_line_gap = 10

# Loop through the frames of the video
while True:
    # Read a frame from the video stream
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply edge detection to the grayscale frame
    edges = cv2.Canny(gray, canny_thresh1, canny_thresh2)

    # Apply line detection to the edge map
    lines = cv2.HoughLinesP(edges, rho, theta, threshold, minLineLength=min_line_length, maxLineGap=max_line_gap)

    # If at least one line was found
    if lines is not None:
        # Loop through all the lines found
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # Draw the line on the original frame
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # Show the original frame and the edge map
    cv2.imshow('frame', frame)
    cv2.imshow('edges', edges)

    # Exit if the user presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
