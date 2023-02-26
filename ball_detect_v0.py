import cv2
import numpy as np

# Capture video from default camera
cap = cv2.VideoCapture(0)

# Define the parameters for the edge detection algorithm
canny_thresh1 = 80
canny_thresh2 = 100

# Define the parameters for the circle detection algorithm
dp = 1
minDist = 200
param1 = 100
param2 = 50
minRadius = 1
maxRadius = 200

# Loop through the frames of the video
while True:
    # Read a frame from the video stream
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply edge detection to the grayscale frame
    edges = cv2.Canny(gray, canny_thresh1, canny_thresh2)

    # Apply the Hough Circle Transform to the edge map
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp, minDist,
                               param1=param1, param2=param2,
                               minRadius=minRadius, maxRadius=maxRadius)

    # If at least one circle was found
    if circles is not None:
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")

        # Loop through all the circles found
        for (x, y, r) in circles:
            # Draw the circle on the original frame
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)

    # Show the original frame and the edge map
    cv2.imshow('frame', frame)
    cv2.imshow('edges', edges)

    # Exit if the user presses the 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close all windows
cap.release()
cv2.destroyAllWindows()
