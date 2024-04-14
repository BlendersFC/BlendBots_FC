#!/usr/bin/env python2.7

import numpy as np
import cv2
import math
import os
import goal_detection as gd
import line_detection as ld


def main():
    # Open a connection to the webcam (use 0 for default webcam)
    cap = cv2.VideoCapture(0)
    # cap= cv2.VideoCapture("C:/Users/lizet/Downloads/Crop.mp4")

    # Number of rows and columns to divide the image into
    num_rows = 10
    num_cols = 10

    # Number of top regions to select
    num_top_regions = 12

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            break

        # Find the top border of the grass
        top_border = gd.find_top_border(frame)

        # Display the top border for visualization
        # cv2.line(frame, (0, top_border), (frame.shape[1], top_border), (0, 255, 255), 2)

        frame=ld.line_elimination(frame,top_border)

        # Find the top N regions with the highest light pixel concentration, discarding the part above the top border
        top_indices, top_regions = gd.find_light_interest_regions(frame, num_rows, num_cols, num_top_regions, top_border)

        # cv2.imshow("Before contours", frame)

        # Calculate the center of the goal posts
        goal_centers = []
        for index, region in zip(top_indices, top_regions):
            gray_region = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
            _, binary_mask = cv2.threshold(gray_region, 150, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            

            # Draw the contours on the original image with correct offset
            gd.draw_contours(frame, contours, index, num_cols, num_rows, top_border)
            goal_centers = gd.calculate_centers(contours, index, num_cols, num_rows, top_border, goal_centers, frame)

        # Calculate the average center of the goal posts
        if goal_centers:
            avg_center = tuple(np.mean(goal_centers, axis=0, dtype=int))

            # Print the average center coordinates
            print(f"Avg Center Coordinates: {avg_center}")

            # Draw a marker at the average center
            cv2.drawMarker(frame, avg_center, (255, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # # Display the result
        cv2.imshow("Goal contours and center", frame)

        # Exit when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()