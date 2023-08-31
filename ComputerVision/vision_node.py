#!/usr/bin/env python2.7
#coding=utf-8

import rospy
import sensor_msgs
import numpy as np
from sensor_msgs.msg import Image
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

Kpx = 0.0175
#Kix = 1.85
Kdx = 0.0001

Kpy = 0.01
Kdy = 0.0001
Tm = 0.1 #100ms

t = 0

case = True #True -> Pelota
            #False -> Portería

cnt = 0
centro = 0
centro_sum = 0

old_x1 = 0
old_y1 = 0
old_x2 = 0
old_y2 = 0

old_centerx = 0
old_centery = 0

goal = True

"""def caseCallback(objective):
    global case

    case = objective.data"""


def imageCallback(img_msg):
    global cnt
    global centro
    global centro_sum
    global old_x1
    global old_y1
    global old_x2
    global old_y2
    global old_centerx
    global old_centery
    global goal
    global case

    rospy.loginfo(img_msg.header)

    try:
        frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #cv_img = cv2.flip(cv_img,1)
    except CvBridgeError:
        rospy.logerr("CvBridge Error")

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)

    #print(case)
    
    if case:
        #################### Ball tracking ######################
        #hsv_video = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        find_ball = Bool()
        find_ball.data = True
        ball_pub.publish(find_ball)

        """limit_low = np.array([16,0,122])
        limit_high = np.array([66,42,255])"""

        """limit_low = np.array([115,100,125])
        limit_high = np.array([130,255,255])
        
        mask = cv2.inRange(hsv_video, limit_low, limit_high)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        #mask = cv2.inRange(cv_img, limit_low, limit_high)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)"""


        nR = np.matrix(blurred[:,:,0])
        nG = np.matrix(blurred[:,:,1])
        nB = np.matrix(blurred[:,:,2])

        color = cv2.absdiff(nG,nR)

        _, umbral = cv2.threshold(color,50,255,cv2.THRESH_BINARY)

        mask = cv2.erode(umbral,np.ones((35,35), np.uint8))
        mask = cv2.dilate(mask, None, iterations=2)
        
        """contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for idx, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > 100 and area < 1000:
                contour = cv2.approxPolyDP(cnt, 0.001*cv2.arcLength(cnt, True), True)
                
                xb,yb,wb,hb = cv2.boundingRect(contour)
                xball = int(xb+(wb/2))
                yball = int(yb+(hb/2))

                cv2.rectangle(frame, (xb,yb), (xb+wb,yb+hb), (255,0,0), 8)
                cv2.circle(frame, (xball, yball), 5,  (255,255,255), 5)"""
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        xb,yb,wb,hb = cv2.boundingRect(opening)
        xball = int(xb+(wb/2))
        yball = int(yb+(hb/2))

        cv2.rectangle(frame, (xb,yb), (xb+wb,yb+hb), (255,0,0), 8)
        cv2.circle(frame, (xball, yball), 5,  (255,255,255), 5)

        area = wb*hb
        print(area)

        final_img = bridge.cv2_to_imgmsg(frame, "rgb8")
        pub.publish(final_img)

        error(xball, yball, area)
    
    """else:
        #################### Goal detection #####################

        find_ball = Bool()
        find_ball.data = False
        print(find_ball.data)
        ball_pub.publish(find_ball)


        y_frame, x_frame, _ = frame.shape

        #print("PORTERÍAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

        gray_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        _, binarie = cv2.threshold(gray_frame, 85, 255, cv2.THRESH_BINARY)
        canny = cv2.Canny(gray_frame, 0, 100)
        dst = cv2.dilate(canny, (15,15), iterations=5) 

        out1=np.zeros_like(dst)
        out2=np.zeros_like(dst)

        lines = cv2.HoughLinesP(dst, 1, np.pi/180, 150, 10, 100)

        try:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                m2 = gradient(x1, y1, x2, y2)
                m1 = gradient(x2, y2, old_x1, old_y1)
                angleR = math.atan((m2-m1)/(1+(m2*m1)))
                angleD = round(math.degrees(angleR))

                if (angleD > -50 and angleD < 50) and binarie[y1][x1] and binarie[y2][x2]:

                    cv2.line(out1,(x1,y1),(x2,y2),(255,255,255),8)
                    cv2.line(frame,(x1,y1),(x2,y2),(255,0,255),8)

                    centro = x2 + (x1-x2)/2
                    centro_sum = centro_sum + centro
                        
                    if cnt == 1:
                        centro_promedio = centro_sum/cnt
                        cv2.line(out2,(centro_promedio,y_frame),(centro_promedio,0),(255,255,255),3)
                        #cv2.putText(frame,str(angleD),(x1-40,y1-20),cv2.FONT_HERSHEY_COMPLEX,1.5,(0,0,255),2)
                        cnt = 0
                        centro_sum = 0
                        goal = True
                        break
                        
                    cnt = cnt + 1
                else:
                    goal = False
                
                old_x1 = x1
                old_y1 = y1
                old_x2 = x2
                old_y2 = y2
            
            mask = cv2.bitwise_and(out1, out2)
            opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (5, 5))

            x_center,y_center,w_center,h_center = cv2.boundingRect(opening)
            print(x_center, y_center, w_center, h_center)

            centerx = int(x_center+(w_center/2))
            centery = int(y_center+(h_center/2))
            if centerx != 0 and centery != 0:
                cv2.circle(frame, (centerx, centery), 5,  (255,255,255), 2)
                old_centerx = centerx
                old_centery = centery
            elif centerx == 0 and centery == 0 and goal:
                #print(centerx, centery)
                cv2.circle(frame, (old_centerx, old_centery), 5,  (255,255,255), 2)

            final_img = bridge.cv2_to_imgmsg(frame, "rgb8")
            pub.publish(final_img)

            error(centerx, centery, 0)

        except:
            final_img = bridge.cv2_to_imgmsg(frame, "rgb8")
            pub.publish(final_img)
            error(70, 70, 0)"""

    """xball = int(xb+(wb/2))
    yball = int(yb+(hb/2))

    cv2.rectangle(frame, (xb,yb), (xb+wb,yb+hb), (255,0,0), 8)
    cv2.circle(frame, (xball, yball), 5,  (255,255,255), 5)

    area = wb*hb
    #print(area)
    
    ################ Dibujar línea pelota-portería ##################
    try:
        if centerx > 0 and centery > 0 and area > 0:
            cv2.line(frame,(centerx,centery),(xball,yball),(0,0,255),3)
    except:
        pass

    cv2.line(frame,(int(x_frame/2),int(y_frame)),(int(x_frame/2),0),(50,130,255),3)

    final_img = bridge.cv2_to_imgmsg(frame, "rgb8")
    pub.publish(final_img)

    error(xball, yball, area)"""


def error(x, y, a):
    x_center = 320
    y_center = 240

    errorx = x_center - x
    errory = y_center - y

    errorx = errorx * 70/x_center
    errory = errory * 70/y_center

    #print(errorx, errory)

    point = Point()
    point.x = errorx
    point.y = errory

    error_point = Point()
    error_point.x = errorx
    error_point.y = errory

    error_pub.publish(error_point)

    control(point, a)

def control(point, a):
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

        #ux = ux1 + ( Kpx + Kdx/Tm)*errx0 + (-Kpx + Kix*Tm - 2*Kdx/Tm)*errx1 + (Kdx/Tm)*errx2
        ux = ux1 + (Kpx + Kdx/Tm)*errx0 + (-2*Kdx/Tm)*errx1 + (-Kpx + Kdx/Tm)*errx2
        #uy = uy1 + (Kpy + Kdy/Tm)*erry0 + (-Kpy + Kiy*Tm - 2*Kdy/Tm)*erry1 + (Kdy/Tm)*erry2
        uy = uy1 + (Kpy + Kdy/Tm)*erry0 + (-2*Kdy/Tm)*erry1 + (-Kpy + Kdy/Tm)*erry2

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

        #if ux > 3 or ux < -3:

        if errx0 < -5 or errx0 > 5:

            ux = (ux*math.pi)/180
            uy = (uy*math.pi)/180

            position_point = Point()
            position_point.x = ux
            position_point.y = uy
            position_point.z = a

            position_pub.publish(position_point)
    else:
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

        position_pub.publish(position_point)


rospy.init_node('image')
pub = rospy.Publisher('/Vision_node', sensor_msgs.msg.Image, queue_size=1)
error_pub = rospy.Publisher('/error', geometry_msgs.msg.Point, queue_size=1)
position_pub = rospy.Publisher('/position', geometry_msgs.msg.Point, queue_size=1)
mov_pub = rospy.Publisher('/following', std_msgs.msg.Bool, queue_size=1)
ball_pub = rospy.Publisher('/find_ball', std_msgs.msg.Bool, queue_size=1)

kernel = np.ones((5,5),np.uint8)

rospy.loginfo("Hello ros")

bridge = CvBridge()

subimg = rospy.Subscriber("/usb_cam/image_raw", Image, imageCallback)
#subcase = rospy.Subscriber("/vision_case", Bool, caseCallback)

kernel = np.ones((5,5),np.uint8)

while not rospy.is_shutdown():
    pass