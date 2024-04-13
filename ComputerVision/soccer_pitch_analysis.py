#!/usr/bin/env python2.7

import os
import math
import rospy
import sensor_msgs
import numpy as np
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs
from geometry_msgs.msg import Point
import std_msgs
from std_msgs.msg import Bool
import goal_detection as gd
import line_detection as ld


def image_callback(img_msg):

    rospy.loginfo(img_msg.header)

    try:
        frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #cv_img = cv2.flip(cv_img,1)
    except CvBridgeError:
        rospy.logerr("CvBridge Error")
       # continue

    img = goal_fusion(frame) ##como le meto mi imagen y saco lo de cv2

    final_img = bridge.cv2_to_imgmsg(img, "rgb8")
    #mask_img = bridge.cv2_to_imgmsg(dummy_mask)
    #frame_img = bridge.cv2_to_imgmsg(frame, "rgb8") # Si es imagen binaria, quitar "rgb8"

    pub_final.publish(final_img)
    #pub_mask.publish(mask_img)
    #pub_frame.publish(frame_img)


def goal_fusion(cap):
    # Open a connection to the webcam (use 0 for default webcam)
    # cap = cv2.VideoCapture(0)

    # Number of rows and columns to divide the image into
    num_rows = 10
    num_cols = 10

    # Number of top regions to select
    num_top_regions = 12

    # Find the top border of the grass
    top_border = gd.find_top_border(frame)

    # Display the top border for visualization
    #cv2.line(frame, (0, top_border), (frame.shape[1], top_border), (0, 255, 255), 2)

    frame=ld.line_elimination(frame, top_border)

    # Find the top N regions with the highest light pixel concentration, discarding the part above the top border
    top_indices, top_regions = gd.find_light_interest_regions(frame, num_rows, num_cols, num_top_regions, top_border)

    #cv2.imshow("Before contours", frame)

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
    
    return frame

    # Release the webcam and close all windows
    #cap.release()
    #cv2.destroyAllWindows()



if __name__ == "__main__":
    rospy.init_node('image')
    pub_final = rospy.Publisher('/Final', sensor_msgs.msg.Image, queue_size=1)
    #pub_mask = rospy.Publisher('/Mask', sensor_msgs.msg.Image, queue_size=1)
    #pub_frame = rospy.Publisher('/Frame', sensor_msgs.msg.Image, queue_size=1)

    rospy.loginfo("Hello ros")

    bridge = CvBridge()

    subimg = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    while not rospy.is_shutdown():
        pass
