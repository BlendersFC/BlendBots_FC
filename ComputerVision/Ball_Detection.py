import cv2
import numpy as np
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

prevCircle = None
dist = lambda x1,y1,x2,y2: (x1-x2)**2*(y1-y2)**2
old_values = 0
kernel = np.ones((5,5),np.uint8)

shot = cv2.VideoCapture(0)

green_lower = (10, 0, 0)
green_upper = (50, 255, 180)

def imageCallback(img_msg):

    rospy.loginfo(img_msg.header)

    try:
        frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #cv_img = cv2.flip(cv_img,1)
    except CvBridgeError:
        rospy.logerr("CvBridge Error")

    hsv_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    dummy_mask = np.zeros_like(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
    
    hsv = cv2.inRange(hsv_frame, green_lower, green_upper)
    mask = cv2.erode(hsv,np.ones((3, 3), np.uint8))
    mask = cv2.dilate(mask, np.ones((15,15), np.uint8))
    _, umbral = cv2.threshold(hsv, 250, 255,cv2.THRESH_BINARY_INV)
    mask = cv2.erode(umbral, np.ones((7, 7), np.uint8))
    mask = cv2.dilate(mask, np.ones((3,3), np.uint8))

    #the 1.2 value (dp) merges circles as much as the value be
    #MinDist (100) defines the min dist between circles fund (pixels). If you're looking for just one cirle, use a higher number than 100
    #param1: if it is too low, will find too many circles, if it is too high, will not find any circles
    #param2: the minimum number of point to determ it is a circle
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.2, 1500, param1=1500, param2=30, minRadius=50, maxRadius=1000)

    if circles is not None:
        circles = np.uint16(np.around(circles)) #convert to a np array
        
        #Now we take de prevCircle to compare with the new circle, in order to reduce the random circles number
        chosen = None
        for i in circles[0, :]:
            if chosen is None: chosen = i
            if prevCircle is not None:
                if dist(chosen[0], chosen[1], prevCircle[0], prevCircle[1]) <= dist(i[0], i[1], prevCircle[0], prevCircle[1]):
                    chosen = i
        
        cv2.circle(dummy_mask, (chosen[0], chosen[1]), 1, (0,100,100), 3)
        cv2.circle(dummy_mask, (chosen[0], chosen[1]), chosen[2], (255,0,0), 8)
        prevCircle = chosen
    else:
        print("None")
    
    final_mask = cv2.bitwise_and(mask, dummy_mask)

    contours, _ = cv2.findContours(final_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for idx, cnt in enumerate(contours):
        
        opening = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)

        x,y,w,h = cv2.boundingRect(opening)
        #print(x,y,w,h)
        
        cv2.rectangle(frame, (x,y), (x+w,y+h), color=(255,255,0), thickness=3)
        cv2.circle(frame, (int(x+(w/2)), int(y+(h/2))),5, color=(255,255,255), thickness=3)

    final_img = bridge.cv2_to_imgmsg(final_mask)#frame, "rgb8")
    pub.publish(final_img)

    #error(xball, yball, area)

"""def error(x, y, a):
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

    control(point, a)"""

rospy.init_node('image')
pub = rospy.Publisher('/Vision_node', sensor_msgs.msg.Image, queue_size=1)
"""error_pub = rospy.Publisher('/error', geometry_msgs.msg.Point, queue_size=1)
position_pub = rospy.Publisher('/position', geometry_msgs.msg.Point, queue_size=1)
mov_pub = rospy.Publisher('/following', std_msgs.msg.Bool, queue_size=1)
ball_pub = rospy.Publisher('/find_ball', std_msgs.msg.Bool, queue_size=1)"""

rospy.loginfo("Hello ros")

bridge = CvBridge()

subimg = rospy.Subscriber("/usb_cam/image_raw", Image, imageCallback)
#subcase = rospy.Subscriber("/vision_case", Bool, caseCallback)

while not rospy.is_shutdown():
    pass