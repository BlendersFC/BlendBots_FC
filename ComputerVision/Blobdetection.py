#!/usr/bin/env python2.7


import numpy as np
import cv2
import math
import os

def erase_lines(frame):

    green_lower = (23, 40, 33)
    green_upper = (61, 255, 255)
    white_lower = (0, 0, 150)
    white_upper = (88, 90, 234)

    frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_CUBIC)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    dummy_mask = np.zeros_like(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

    hsv = cv2.inRange(hsv_frame, white_lower, white_upper) #mascara donde solo quedará en blanco lo blanco
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
    cv2.imshow("White mask", mask)
    cv2.imshow("Green detection", mask2)
    cv2.imshow("Final Result", final)
    return mask2


def blob(green):
    
    params = cv2.SimpleBlobDetector_Params()
    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200
    
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 1500
    
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.5
    
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87
    
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else : 
        detector = cv2.SimpleBlobDetector_create(params)
 
    # Detect blobs.
    #im = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
    # Detect blobs.
    reversemask = 255 - green
    keypoints = detector.detect(reversemask)
    
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    im_with_keypoints = cv2.drawKeypoints(green, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Show keypoints
    cv2.imshow("Keypoints", im_with_keypoints)
    #return keypoints, reversemask

def find_top_border(image):
    # Convert the image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the green color
    lower_green = (23, 40, 33)
    upper_green = (61, 255, 255)

    # Threshold the image to extract the green color
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

     # Display the green mask for visualization
    cv2.imshow("Green Mask", green_mask)

    # Find contours in the green mask
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the contour with the minimum y-coordinate (top border)
        min_y = min(contours, key=lambda c: cv2.boundingRect(c)[1])[0][0][1]
        return min_y
    else:
        return 0  # Default to 0 if no contours are foun
    
def main():
    # Open a connection to the webcam (use 0 for default webcam)
    cap = cv2.VideoCapture("C:/Users/lizet/Downloads/VIDPRUEBA.mp4")
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        frame = cv2.resize(frame, dsize=(800, 480), interpolation=cv2.INTER_CUBIC)

        # Find the top border of the grass
        top_border = find_top_border(frame)

        # Display the top border for visualization
        cv2.line(frame, (0, top_border), (frame.shape[1], top_border), (0, 255, 255), 2)
        cv2.imshow("border", frame)
 
        green= erase_lines(frame)
        blob(green)
    
        if not ret:
            break
        # Exit when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()