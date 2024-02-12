#!/usr/bin/env python2.7
#coding=utf-8

import rospy
import sensor_msgs
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs
from geometry_msgs.msg import Point
import math
import std_msgs
from std_msgs.msg import Bool

gradient = lambda x1,y1,x2,y2: (y2-y1)/(x2-x1)

ux = 0
uy = 0

ux1 = 0
uy1 = 0

Kpx = 0.002  # 0.0175
Kix = 5
Kdx = 0.0001

Kpy = 0.002  # 0.0175
Kiy = 5
Kdy = 0.0001

Tm = 0.1 #100ms

t = 0

case = True #True -> Pelota
            #False -> Portería

old_x1 = 0
old_y1 = 0
old_x2 = 0
old_y2 = 0

old_centerx = 0
old_centery = 0

center = Point()


def callbackCenter(ball_center):
    global center

    center.x = ball_center.x
    center.y = ball_center.y

    error(center.x, center.y)


def error(x, y):
    x_center = 320
    y_center = 240

    errorx = x_center - x
    errory = y_center - y

    errorx = errorx * 70/x_center
    errory = errory * 70/y_center

    point = Point()
    point.x = errorx
    point.y = errory

    error_point = Point()
    error_point.x = errorx
    error_point.y = errory

    error_pub.publish(error_point)

    control(point)


def control(point):
    global ux
    global uy
    global ux1
    global uy1
    global Kpx
    global Kpy
    global Kdx
    global Kdy
    global Tm
    global t

    errx0 = point.x
    erry0 = point.y

    if errx0 != 70 and erry0 != 70:

        errx1 = 0
        erry1 = 0

        errx2 = 0
        erry2 = 0

        ux = ux1 + (Kpx + Kdx/Tm)*errx0 + (-Kpx + Kix*Tm - 2*Kdx/Tm)*errx1 + (Kdx/Tm)*errx2
        #ux = ux1 + (Kpx + Kdx/Tm)*errx0 + (-2*Kdx/Tm)*errx1 + (-Kpx + Kdx/Tm)*errx2
        uy = uy1 + (Kpy + Kdy/Tm)*erry0 + (-Kpy + Kiy*Tm - 2*Kdy/Tm)*erry1 + (Kdy/Tm)*erry2
        #uy = uy1 + (Kpy + Kdy/Tm)*erry0 + (-2*Kdy/Tm)*erry1 + (-Kpy + Kdy/Tm)*erry2

        if ux >= 70: ux = 70
        elif ux <= -70: ux = 70

        if uy >= 0: uy = -5
        elif uy <= -70: uy = -70

        ux1 = ux
        errx1 = errx0
        errx2 = errx1

        uy1 = uy
        erry2 = erry1
        erry1 = erry0

        if (errx0 < -5 or errx0 > 5) or (erry0 < -5 or erry0 > 5):

            ux = (ux*math.pi)/180
            uy = (uy*math.pi)/180

            position_point = Point()
            position_point.x = ux
            position_point.y = uy

            position_pub.publish(position_point)
    """else:
        t += 1

        if (t>360):
            t=-360

        ux_noball = 70*math.sin(0.1*t) #(t/(180/3.14159))

        uy_noball = 20*math.cos(0.1*t) - 20

        ux_noball = (ux_noball*math.pi)/180
        uy_noball = (uy_noball*math.pi)/180

        position_point = Point()

        position_point.x = ux_noball
        position_point.y = uy_noball

        position_pub.publish(position_point)"""


rospy.init_node('Head_Control_Node')
error_pub = rospy.Publisher('/error', geometry_msgs.msg.Point, queue_size=1)
position_pub = rospy.Publisher('/position', geometry_msgs.msg.Point, queue_size=1)
mov_pub = rospy.Publisher('/following', std_msgs.msg.Bool, queue_size=1)
ball_pub = rospy.Publisher('/find_ball', std_msgs.msg.Bool, queue_size=1)

kernel = np.ones((5,5),np.uint8)

rospy.loginfo("Hello ros")

bridge = CvBridge()

sub_center = rospy.Subscriber("/BallCenter", Point, callbackCenter)

kernel = np.ones((5,5),np.uint8)

while not rospy.is_shutdown():
    pass
