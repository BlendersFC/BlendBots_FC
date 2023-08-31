import cv2
from ObjectDetection import ObjectDetection
import numpy as np

prevCircle = None
dist = lambda x1,y1,x2,y2: (x1-x2)**2*(y1-y2)**2
old_values = 0
kernel = np.ones((5,5),np.uint8)

shot = cv2.VideoCapture(0)

green_lower = (10, 0, 0)
green_upper = (50, 255, 180)

while True:
    ret, frame = shot.read()

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

    """field = ObjectDetection(frame)

    field.hsv_detection(green_lower, green_upper, 1, 1, True)

    print(field.mask)"""

    cv2.imshow("Video", frame)
    cv2.imshow("Mask", final_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

shot.release()
cv2.destroyAllWindows()