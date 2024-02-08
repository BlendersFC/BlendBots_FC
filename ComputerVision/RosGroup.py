import rospy
import sensor_msgs
import numpy as np
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs
from geometry_msgs.msg import Point
import math
import std_msgs
from std_msgs.msg import Bool
import os

def imageCallback(img_msg):

    rospy.loginfo(img_msg.header)

    try:
        frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #cv_img = cv2.flip(cv_img,1)
    except CvBridgeError:
        rospy.logerr("CvBridge Error")
       # continue

    img = white_ball(frame) ##como le meto mi imagen y saco lo de cv2

    final_img = bridge.cv2_to_imgmsg(img, "rgb8")
    #frame_img = bridge.cv2_to_imgmsg(frame, "rgb8") # Si es imagen binaria, quitar "rgb8"

    pub_final.publish(final_img)
    #pub_mask.publish(mask_img)
    #pub_frame.publish(frame_img)

def white_ball():
    frame_HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    frame_threshold = cv.inRange(frame_HSV, (0, 0, 171), (120, 23, 255))
    result = cv.bitwise_and(img, img, mask=frame_threshold)
    cimg = cv.cvtColor(result, cv.COLOR_HSV2BGR)
    img = cv.GaussianBlur(cimg, (7, 7), 1.5)

    cimg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    circles = cv.HoughCircles(cimg, cv.HOUGH_GRADIENT, 1.363, 750,
                           param1=150, param2=48, minRadius=0, maxRadius=250)

    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv.circle(cimg, (i[0], i[1]), i[2], (255, 0, 0), 2)
            # draw the center of the circle
            cv.circle(cimg, (i[0], i[1]), 2, (255, 255, 0), 3)    
    cimg = cv.cvtColor(img,cv.COLOR_GRAY2BGR)
    return cimg

if __name__ == "__main__":
    rospy.init_node('image')
    pub_final = rospy.Publisher('/Final', sensor_msgs.msg.Image, queue_size=1)
    #pub_mask = rospy.Publisher('/Mask', sensor_msgs.msg.Image, queue_size=1)
    #pub_frame = rospy.Publisher('/Frame', sensor_msgs.msg.Image, queue_size=1)

    rospy.loginfo("Hello ros")

    bridge = CvBridge()

    subimg = rospy.Subscriber("/usb_cam/image_raw", Image, imageCallback)

    while not rospy.is_shutdown():
        pass