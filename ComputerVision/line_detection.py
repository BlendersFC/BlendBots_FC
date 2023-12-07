#!/usr/bin/env python2.7

import numpy as np
import cv2

def erase_lines(frame):

    green_lower = (23, 74, 33)
    green_upper = (61, 255, 255)
    white_lower = (0, 0, 88)
    white_upper = (88, 90, 234)

    frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_CUBIC)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dummy_mask = np.zeros_like(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

    hsv = cv2.inRange(hsv_frame, white_lower, white_upper)
    mask = hsv

    _, umbral = cv2.threshold(hsv, 250, 255, cv2.THRESH_BINARY)
    # Additional processing...

    # LINEA ENTERA####
    img = cv2.GaussianBlur(mask, (7, 7), 0)
    edges = cv2.Canny(img, 50, 150, apertureSize=5)
    lines2 = cv2.HoughLines(edges, 1, 3.141592 / 180, 90)

    if lines2 is not None:
        for line in lines2:
            rho, theta = line[0]
            
            # Filter out horizontal lines (near horizontal)
            if np.abs(theta) > np.pi / 4 and np.abs(theta - np.pi) > np.pi / 4:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(dummy_mask, (x1, y1), (x2, y2), (255, 255, 255), 10)

    ####3GREEN DETECTION #####
    hsv = cv2.inRange(hsv_frame, green_lower, green_upper)
    mask2 = cv2.erode(hsv, np.ones((7, 7), np.uint8))
    mask2 = cv2.dilate(mask2, np.ones((4, 4), np.uint8))
    _, umbral = cv2.threshold(hsv, 250, 255, cv2.THRESH_BINARY_INV)
    mask2 = cv2.erode(umbral, np.ones((7, 7), np.uint8))
    mask2 = cv2.dilate(mask2, np.ones((3, 3), np.uint8))

    final = mask2-dummy_mask  # White reduction

    # Display images
    cv2.imshow("Original Frame", mask)
    cv2.imshow("Dummy Mask", dummy_mask)
    cv2.imshow("Final Result", final)

# ... (the rest of the code remains the same)
# Initialize the camera
vc = cv2.VideoCapture(0)

while True:
    ret, frame = vc.read()

    if not ret:
        print("Failed to capture frame. Exiting...")
        break

    erase_lines(frame)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close OpenCV windows
vc.release()
cv2.destroyAllWindows()