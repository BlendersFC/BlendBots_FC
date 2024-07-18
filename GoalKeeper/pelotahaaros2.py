#!/usr/bin/env python2.7

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


def apply_mask(matrix, mask, fill_value):
    masked = np.ma.array(matrix, mask=mask, fill_value=fill_value)
    return masked.filled()

def apply_threshold(matrix, low_value, high_value):
    low_mask = matrix < low_value
    matrix = apply_mask(matrix, low_mask, low_value)

    high_mask = matrix > high_value
    matrix = apply_mask(matrix, high_mask, high_value)

    return matrix

def simplest_cb(img, percent):
    assert img.shape[2] == 3
    assert percent > 0 and percent < 100

    half_percent = percent / 200.0

    channels = cv2.split(img)

    out_channels = []
    for channel in channels:
        assert len(channel.shape) == 2
        # find the low and high precentile values (based on the input percentile)
        height, width = channel.shape
        vec_size = width * height
        flat = channel.reshape(vec_size)

        assert len(flat.shape) == 1

        flat = np.sort(flat)

        n_cols = flat.shape[0]

        low_val  = flat[math.floor(n_cols * half_percent)]
        high_val = flat[math.ceil( n_cols * (1.0 - half_percent))]


        # saturate below the low percentile and above the high percentile
        thresholded = apply_threshold(channel, low_val, high_val)
        # scale the channel
        normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
        out_channels.append(normalized)

    return cv2.merge(out_channels)







get_distance = lambda p1, p2: math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)



def angleCallback(angle_msg):
    global half_boolean
    rospy.loginfo(angle_msg.position)
    angle_msg=angle_msg.position[1]
    if angle_msg<-0.7:
        half_boolean=True
    else:
        half_boolean=False

def imageCallback(img_msg):
    global center, old_center

    rospy.loginfo(img_msg.header)

    try:
        frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        # cv_img = cv2.flip(cv_img,1)
    except CvBridgeError:
        rospy.logerr("CvBridge Error")
    # continue

	slope.data = 0
    img, center = ball_detect(frame)  ##como le meto mi imagen y saco lo de cv2
    if center.x != None and center.y != None and get_distance(center, old_center) > 5:
        print(center.x, center.y)
        img, slope.data = get_predictions(img, center)
    
    old_center.x = center.x; old_center.y = center.y

    final_img = bridge.cv2_to_imgmsg(img, "rgb8")
    # mask_img = bridge.cv2_to_imgmsg(dummy_mask)
    # frame_img = bridge.cv2_to_imgmsg(frame, "rgb8") # Si es imagen binaria, quitar "rgb8"

    pub_slope.publish(slope)
    pub_img.publish(final_img)
    # pub_mask.publish(mask_img)
    # pub_frame.publish(frame_img)


# cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
def ball_detect(frame):
    global center, half_boolean

    pelota_clas = cv2.CascadeClassifier('/home/robotis/blenders_ws/src/soccer_pkg/data/cascade_manual.xml')
    half_clas = cv2.CascadeClassifier('/home/robotis/blenders_ws/src/soccer_pkg/data/half_cascade.xml')
    

    
    # ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if half_boolean:

        ball = half_clas.detectMultiScale(gray,
                                           scaleFactor=1.25,
                                           minNeighbors=5)
    else:
        ball = pelota_clas.detectMultiScale(gray,
                                           scaleFactor=1.25,
                                           minNeighbors=3, minSize=(50, 50))

    if len(ball)>0:
        x,y,w,h = ball[0]
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cx = int((x + (x + w))/ 2)
        cy = int((y + (y + h))/ 2)
        a = w * h
        center.x = cx
        center.y = cy
        center.z = a
        cv2.circle(frame, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)
        pub_center.publish(center)
    else:
        center.x = 999
        center.y = 999
        pub_center.publish(center)

    return frame, center

def get_predictions(frame, center, predict_num=4):
    for p in range(predict_num):
        if p == 0:
            prediction = _predict(np.array((center.x, center.y), np.float32))
            first_prediction = prediction
        else:
            prediction = _predict(np.array((old_prediction[0], old_prediction[1]), np.float32))
        cv2.circle(frame, (prediction[0], prediction[1]),5, color=(255,0,0), thickness=3)
        old_prediction = prediction
    
    try:
        slope = (prediction[1] - first_prediction[1]) / (prediction[0] - first_prediction[0])

        # calculate y-intercept
        b = first_prediction[1]- (slope * first_prediction[0])

        # draw infinite line
        cv2.line(frame, (0, int(b)), (1280, int((slope * 1280) + b)), color=(255,255,255), thickness=2)
        cv2.line(frame, (first_prediction[0], first_prediction[1]), (first_prediction[0],prediction[1]), color=(255,255,255), thickness=2)
        
        return frame, slope
    except:
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
    kalman.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.003
    kalman.measurementNoiseCov = np.eye(n_measures, dtype = np.float32)*0.1


    rospy.init_node('image')
    robot_id = rospy.get_param('robot_id', 0)
    pub_img = rospy.Publisher('robotis_' + str(robot_id) + '/ImgFinal', Image, queue_size=1)
    pub_center = rospy.Publisher('robotis_' + str(robot_id) + '/ball_center', Point, queue_size=1)
    pub_slope = rospy.Publisher('robotis_' + str(robot_id) + '/Slope', Float64, queue_size=1)
    #self.subimg = rospy.Subscriber("/robotis_" + str(self.robot_id) +"/usb_cam/image_raw", Image, self.image_callback)

    rospy.loginfo("Hello ros")

    subimg = rospy.Subscriber("/usb_cam_node/image_raw", Image, imageCallback)
    #subscriptor del angulo de la cabeza
    subangle_head = rospy.Subscriber('robotis_' + str(robot_id) + '/present_joint_states',JointState, angleCallback)

    while not rospy.is_shutdown():
        pass
