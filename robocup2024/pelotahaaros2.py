#!/usr/bin/env python3

import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

get_distance = lambda p1, p2: math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)

def angleCallback(angle_msg):
    global half_boolean
    angle = angle_msg.position[1]
    if angle < -0.7:
        half_boolean = True
    else:
        half_boolean = False

def imageCallback(img_msg):
    global center, old_center

    try:
        frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    img, center = ball_detect(frame)
    old_center.x = center.x
    old_center.y = center.y

    final_img = bridge.cv2_to_imgmsg(img, "rgb8")
    pub_img.publish(final_img)

def ball_detect(frame):
    global center, half_boolean

    # Make a writable copy of the frame
    frame_writable = frame.copy()

    pelota_clas = cv2.CascadeClassifier('/home/robotis/blenders_ws/src/soccer_pkg/data/cascade_manual.xml')
    half_clas = cv2.CascadeClassifier('/home/robotis/blenders_ws/src/soccer_pkg/data/half_cascade.xml')

    gray = cv2.cvtColor(frame_writable, cv2.COLOR_BGR2GRAY)

    if half_boolean:
        ball = half_clas.detectMultiScale(gray, scaleFactor=1.25, minNeighbors=5)
    else:
        ball = pelota_clas.detectMultiScale(gray, scaleFactor=1.25, minNeighbors=10, minSize=(50, 50))

    if len(ball) > 0:
        x, y, w, h = ball[0]
        cv2.rectangle(frame_writable, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cx = int((x + (x + w)) / 2)
        cy = int((y + (y + h)) / 2)
        a = w * h
        center.x = cx
        center.y = cy
        center.z = a
        cv2.circle(frame_writable, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)
        pub_center.publish(center)
    else:
        center.x = 999
        center.y = 999
        pub_center.publish(center)

    return frame_writable, center


def get_predictions(frame, center, predict_num=4):
    for p in range(predict_num):
        if p == 0:
            prediction = _predict(np.array((center.x, center.y), np.float32))
            first_prediction = prediction
        else:
            prediction = _predict(np.array((old_prediction[0], old_prediction[1]), np.float32))
        cv2.circle(frame, (prediction[0], prediction[1]), 5, color=(255, 0, 0), thickness=3)
        old_prediction = prediction

    try:
        slope = (prediction[1] - first_prediction[1]) / (prediction[0] - first_prediction[0])
        b = first_prediction[1] - (slope * first_prediction[0])
        cv2.line(frame, (0, int(b)), (1280, int((slope * 1280) + b)), color=(255, 255, 255), thickness=2)
        cv2.line(frame, (first_prediction[0], first_prediction[1]), (first_prediction[0], prediction[1]), color=(255, 255, 255), thickness=2)
        return frame, slope
    except ZeroDivisionError:
        pass

def _predict(value):
    global kalman
    kalman.correct(value)
    tp = kalman.predict()
    return int(tp[0]), int(tp[1])

if __name__ == "__main__":
    bridge = CvBridge()
    slope = Float64()
    center = Point()
    old_center = Point()
    half_boolean = None

    old_center.x = 0.0
    old_center.y = 0.0
    n_states = 4
    n_measures = 2
    kalman = cv2.KalmanFilter(n_states, n_measures)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.03

    rospy.init_node('image')
    robot_id = rospy.get_param('robot_id', 0)
    pub_img = rospy.Publisher(f'robotis_{robot_id}/ImgFinal', Image, queue_size=1)
    pub_center = rospy.Publisher(f'robotis_{robot_id}/ball_center', Point, queue_size=1)
    pub_slope = rospy.Publisher(f'robotis_{robot_id}/Slope', Float64, queue_size=1)

    rospy.loginfo("Hello ROS")

    subimg = rospy.Subscriber("/usb_cam_node/image_raw", Image, imageCallback)
    subangle_head = rospy.Subscriber(f'robotis_{robot_id}/present_joint_states', JointState, angleCallback)

    rospy.spin()

