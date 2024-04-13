import cv2
import numpy as np
"""Module for object detection operations"""
class ObjectDetection:

    def __init__(self, frame) -> None:
        self.frame = frame
        self.center = None

    def hsv_detection(self, lower_range, upper_range, erode=None, dilate=None, obstacle=None):

        hsv_frame = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)

        hsv = cv2.inRange(hsv_frame, lower_range, upper_range)
        self.mask = cv2.erode(hsv,np.ones((erode,erode), np.uint8))
        self.mask = cv2.dilate(self.mask, np.ones((dilate,dilate), np.uint8))

        self.contours, _ = cv2.findContours(self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if obstacle:
            self.center = []

            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(self.mask, 4, cv2.CV_32S)

            self.center = [(int(i[0]), int(i[1])) for i in centroids[1:].tolist()]

            self.mask = []

            for label in range(0,num_labels-1):
                fake_mask = np.zeros((labels.shape), dtype=np.uint8)

                fake_mask[labels==label+1] = 255
                #print(fake_mask.shape)
                self.mask.append(fake_mask)
               
    def binary_detection(self, mask, binary_frame, threshold=125, erode=None, dilate=None):

        #blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        #gray_frame = cv2.cvtColor(blurred,cv2.COLOR_BGR2GRAY)

        _, umbral = cv2.threshold(binary_frame,threshold,255,cv2.THRESH_BINARY)

        #mask = cv2.erode(umbral,np.ones((erode,erode), np.uint8))
        #mask = cv2.dilate(mask, np.ones((dilate,dilate), np.uint8))

        self.contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def inv_binary_detection(self, mask, binary_frame, threshold=125, erode=None, dilate=None):

        #blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        #gray_frame = cv2.cvtColor(blurred,cv2.COLOR_BGR2GRAY)

        _, umbral = cv2.threshold(binary_frame,threshold,255,cv2.THRESH_BINARY_INV)

        #mask = cv2.erode(umbral,np.ones((erode,erode), np.uint8))
        #mask = cv2.dilate(mask, np.ones((dilate,dilate), np.uint8))

        self.contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    def rectangle_drawing(self, min_area=None, max_area=None, orientation=False, robot=False):
        self.box = []
        
        for idx, cnt in enumerate(self.contours):
            area = cv2.contourArea(cnt)

            if area > min_area and area < max_area:
                contour = cv2.approxPolyDP(cnt, 0.001*cv2.arcLength(cnt, True), True)

                if orientation:
                    if robot:
                        x,y,w,h = cv2.boundingRect(contour)
                        cX = int(x+(w/2))
                        cY = int(y+(h/2))

                        self.center = [(cX, cY)]
                        idx = 0

                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    
                    cv2.drawContours(self.frame,[box],0,(255,0,255), 2)
                    cv2.circle(self.frame, self.center[idx], 3, color=(255,255,255), thickness=2)
                    self.box.append(box)

                else:
                    M = cv2.moments(cnt)
                    cX = int(M["m10"]/M["m00"])
                    cY = int(M["m01"]/M["m00"])
                    center = (cX, cY)
                    
                    x,y,w,h = cv2.boundingRect(contour)
                    
                    cv2.rectangle(self.frame, (x,y), (x+w,y+h), color=(255,255,0), thickness=3)
                    cv2.circle(self.frame, center, 3, color=(255,255,255), thickness=2)
                    self.center = center

        self.box.reverse()

    def potential_fields(self, frame, int_field, mid_field, ext_field, offset=None, offset2=None):

        self.mid_field_mask = np.zeros_like(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
        self.mid_boxes = []

        for obs in range(0,len(self.center)): #obstacle.center
            self.binary_detection(int_field[obs], 250, 1, 1)
            self.frame = frame
            self.rectangle_drawing(1, 1000, True)

            self.binary_detection(mid_field[obs], 250, 1, 1)
            self.frame = frame
            self.rectangle_drawing(1, 10000, True)

            self.binary_detection(mid_field[obs], 250, 1, 1)
            self.frame = self.mid_field_mask
            self.rectangle_drawing(1, 10000, True)
            self.mid_boxes.append(self.box)

            self.binary_detection(ext_field[obs], 250, 1, 1)
            self.frame = frame
            self.rectangle_drawing(1, 100000, True)

        """if self.box.any():
            box = self.box
        else:
            print("Need to run square_drawing() with orientation first!")
            return

        self.field_mask = np.zeros_like(frame)

        try:
            #Middle potential field
            cv2.line(self.frame, (box[0][0]-offset, box[0][1]+offset), (box[1][0]-offset, box[1][1]-offset), (0,255,0), 2)
            cv2.line(self.frame, (box[1][0]-offset, box[1][1]-offset), (box[2][0]+offset, box[2][1]-offset), (0,255,0), 2)
            cv2.line(self.frame, (box[2][0]+offset, box[2][1]-offset), (box[3][0]+offset, box[3][1]+offset), (0,255,0), 2)
            cv2.line(self.frame, (box[3][0]+offset, box[3][1]+offset), (box[0][0]-offset, box[0][1]+offset), (0,255,0), 2)

            #Drawing it in an empty mask
            cv2.line(self.field_mask, (box[0][0]-offset, box[0][1]+offset), (box[1][0]-offset, box[1][1]-offset), (0,255,0), 2)
            cv2.line(self.field_mask, (box[1][0]-offset, box[1][1]-offset), (box[2][0]+offset, box[2][1]-offset), (0,255,0), 2)
            cv2.line(self.field_mask, (box[2][0]+offset, box[2][1]-offset), (box[3][0]+offset, box[3][1]+offset), (0,255,0), 2)
            cv2.line(self.field_mask, (box[3][0]+offset, box[3][1]+offset), (box[0][0]-offset, box[0][1]+offset), (0,255,0), 2)

            #External potential field
            cv2.line(self.frame, (box[0][0]-offset2, box[0][1]+offset2), (box[1][0]-offset2, box[1][1]-offset2), (255, 0,255), 2)
            cv2.line(self.frame, (box[1][0]-offset2, box[1][1]-offset2), (box[2][0]+offset2, box[2][1]-offset2), (255, 0,255), 2)
            cv2.line(self.frame, (box[2][0]+offset2, box[2][1]-offset2), (box[3][0]+offset2, box[3][1]+offset2), (255, 0,255), 2)
            cv2.line(self.frame, (box[3][0]+offset2, box[3][1]+offset2), (box[0][0]-offset2, box[0][1]+offset2), (255, 0,255), 2)
        except:
            pass"""