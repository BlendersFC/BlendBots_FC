"""Run this program to try the model with robot webcam"""
import rospy
import sensor_msgs
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import goal_detection as gd
import line_detection as ld
import math
import cv2
import cProfile
from ultralytics import YOLO
from ultralytics.models.yolo.detect.predict import DetectionPredictor

def imageCallback(img_msg):

    rospy.loginfo(img_msg.header)

    try:
        frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError:
        rospy.logerr("CvBridge Error")
        return

    results = ov_model(frame, stream=True, conf=0.5)

    class_names = ["robot", "goal post", "ball"]

    for r in results:
        boxes = r.boxes

        for box in boxes:
            # bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # convert to int values

            # put box in cam
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

            # confidence
            confidence = math.ceil((box.conf[0] * 100)) / 100

            # class name
            cls = int(box.cls[0])
            text = f"{class_names[cls]} {confidence:.2f}"

            # object details
            org = [x1, y1]
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            color = (255, 0, 0)
            thickness = 2

            cv2.putText(frame, text, org, font, font_scale, color, thickness)

    final_img_msg = bridge.cv2_to_imgmsg(frame, "rgb8")
    pub_final.publish(final_img_msg)



if __name__ == "__main__":
    rospy.init_node('image')
    pub_final = rospy.Publisher('/Final', sensor_msgs.msg.Image, queue_size=1)

    rospy.loginfo("Hello ros")

    bridge = CvBridge()

    # Load model
    model = YOLO("full_soccer_model.pt")
    model.export(format="openvino")
    ov_model = YOLO('full_soccer_model_openvino_model')

    subimg = rospy.Subscriber("/usb_cam/image_raw", Image, imageCallback)

    rospy.spin()
