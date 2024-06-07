#!/usr/bin/env python2.7
#coding=utf-8

import rospy
import sensor_msgs
import numpy as np
from sensor_msgs.msg import Image
import geometry_msgs
from geometry_msgs.msg import Point
import math
import std_msgs
from std_msgs.msg import Bool
import time

class HeadControl:
    def __init__(self):
        rospy.init_node('PD_Head_Control')
        self.error_pub = rospy.Publisher('/error', Point, queue_size=1)
        self.position_pub = rospy.Publisher('/position', Point, queue_size=1)
        self.search_ball_pub = rospy.Publisher('/search_ball', Bool, queue_size=1)
        self.ball_pub = rospy.Publisher('/find_ball', Bool, queue_size=1)
        self.turnNsearch_pub = rospy.Publisher('/turnNsearch', Bool, queue_size=1)

        sub_center = rospy.Subscriber('/BallCenter', Point, self.callbackCenter)

        self.find_ball = Bool()
        self.search_ball = Bool()
        self.turn_search = Bool()
        self.center = Point()

        self.search_ball.data = False

        self.find_ball.data = False

        self.ux0 = 0
        self.ux0 = 0

        self.ux1 = 0
        self.uy1 = 0

        self.kpx = 0.00018
        self.kpy = 0.00018

        #self.kpx = 0.00018
        #self.kpy = 0.00018

        self.errx0 = 0
        self.errx1 = 0
        self.erry0 = 0
        self.erry1 = 0

        self.now = 0
        self.start_search = 0
        self.end_search = False

        self.noball_start_time = 0
        self.noball_now_time = 0
        self.noball_end_time = False

        self.t = 0
        self.head_direction = 1
    
    def callbackCenter(self, ball_center):

        self.center.x = ball_center.x
        self.center.y = ball_center.y

    def cal_err(self):
        x_center = 320
        y_center = 240

        if self.center.x != 999 and self.center.y != 999 and self.center.x != None and self.center.y != None:
            errorx = x_center - self.center.x
            errory = y_center - self.center.y

            errorx = errorx * 70/x_center
            errory = errory * 70/y_center

            point = Point()
            point.x = errorx
            point.y = errory

            self.error_pub.publish(point)

            return point
        else:
            return None

    def control(self):
        self.turn_search.data = False
        self.turnNsearch_pub.publish(self.turn_search)

        point = self.cal_err()

        if (self.errx0 < -35 or self.errx0 > 35) or (self.erry0 < -35 or self.erry0 > 35) or point == None:
            self.find_ball.data = False

        if point == None:

            if not self.noball_start_time:
                self.noball_start_time =  time.time()
            if not self.noball_end_time:
                if (self.noball_now_time - self.noball_start_time) > 3:
                    self.noball_end_time = True
                
                self.noball_now_time = time.time()
            else:
                self.noball_start_time = False
                self.noball_end_time = False
                self.noball_now_time = 0

                ##self.find_ball.data = False
                ##self.ball_pub.publish(self.find_ball)
            
                self.ux1 = 0
                self.uy1 = 0

                self.search_ball.data = True
                self.search_ball_pub.publish(self.search_ball)

                self.t += 1 * self.head_direction

                if (self.t >= 360):
                    self.head_direction = -1
                elif (self <= -360):
                    self.head_direction = 1

                ux_noball = 60*math.sin(1*self.t) #(t/(180/3.14159))
                uy_noball = 15*math.cos(1*-self.t*self.head_direction) - 30

                ux_noball = (ux_noball*math.pi)/180
                uy_noball = (uy_noball*math.pi)/180

                position_point = Point()

                position_point.x = ux_noball
                position_point.y = -0.5
                #position_point.y = uy_noball

                self.position_pub.publish(position_point)

                #time.sleep(0.05)

                self.errx1 = 0
                self.erry1 = 0

                if not self.start_search:
                    self.start_search =  time.time()
                if not self.end_search:
                    self.turn_search.data = False
                    self.turnNsearch_pub.publish(self.turn_search)
                    if (self.now - self.start_search) > 15:
                        self.end_search = True
                    
                    self.now = time.time()
                else:
                    self.turn_search.data = True
                    self.turnNsearch_pub.publish(self.turn_search)
                    time.sleep(1.0)
                    self.start_search = False
                    self.end_search = False
                    self.now = 0
            
        elif not self.find_ball.data:

            self.search_ball.data = False
            self.search_ball_pub.publish(self.search_ball)
            
            self.noball_start_time = 0
            self.noball_now_time = 0
            self.noball_end_time = False

            self.errx0 = point.x
            self.erry0 = point.y

            self.start_search = 0

            ux = self.ux1 + self.kpx*self.errx0 #- self.kp*self.errx1
            uy = self.uy1 + self.kpy*self.erry0 #- self.kp*self.erry1

            if ux >= 70: ux = 70
            elif ux <= -70: ux = -70

            if uy >= 20: uy = 20
            elif uy <= -70: uy = -70

            self.ux1 = ux
            self.errx1 = self.errx0

            self.uy1 = uy
            self.erry2 = self.erry1

            if (self.errx0 < -16 or self.errx0 > 16) or (self.erry0 < -16 or self.erry0 > 16):

                ux = (ux*math.pi)/180
                uy = (uy*math.pi)/180

                position_point = Point()
                position_point.x = ux
                position_point.y = -0.5

                self.position_pub.publish(position_point)

                self.find_ball.data = False
                self.ball_pub.publish(self.find_ball)
            else:
                self.find_ball.data = True
                self.ball_pub.publish(self.find_ball)

            self.center.x = None
            self.center.y = None


if __name__ == "__main__":
    headctrl = HeadControl()
    while not rospy.is_shutdown():
        try:
            headctrl.control()
        except AttributeError:
            continue
