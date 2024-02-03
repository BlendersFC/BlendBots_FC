
#!/usr/bin/env python2.7
import numpy as np
import cv2
import math
import os
def line_elimination(frame):
    green_lower = (32, 112, 16)
    green_upper = (101, 255, 255)
    white_lower = (36, 0, 44)
    white_upper = (133, 31, 255)

    #frame = cv2.resize(frame, dsize=(470, 640), interpolation=cv2.INTER_CUBIC)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    dummy_mask = np.zeros_like(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))

    hsv = cv2.inRange(hsv_frame, white_lower, white_upper)
    mask=hsv
    _, umbral = cv2.threshold(hsv, 250, 255, cv2.THRESH_BINARY)

    ## LINEA ENTERA####
    img = cv2.GaussianBlur(mask, (7, 7), 0)
    edges = cv2.Canny(img, 50, 150, apertureSize=5)  # limites y matriz de gradiente (impar)
    lines2 = cv2.HoughLines(edges, 1, 3.141592 / 180, 150) 

    if lines2 is not None:
        for line in lines2:
            rho, theta = line[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(frame, (x1, y1), (x2, y2), (32, 112, 16), 17)

    ####3GREEN DETECTION #####
    hsv = cv2.inRange(hsv_frame, green_lower, green_upper)
    mask2 = cv2.erode(hsv, np.ones((7, 7), np.uint8))
    mask2 = cv2.dilate(mask2, np.ones((4, 4), np.uint8))
    _, umbral = cv2.threshold(hsv, 250, 255, cv2.THRESH_BINARY_INV)
    mask2 = cv2.erode(umbral, np.ones((7, 7), np.uint8))
    mask2 = cv2.dilate(mask2, np.ones((3, 3), np.uint8))
    
    final = mask2-dummy_mask # White reduction
    return frame

def white_f(frame):
    
    #white_lower = (0, 0, 88)
    #white_upper = (88, 90, 234)
    white_lower = (0, 13, 146)
    white_upper = (150, 250, 255)

    frame = cv2.resize(frame, dsize=(640, 480), interpolation=cv2.INTER_CUBIC)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    dummy_mask = np.zeros_like(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

    mask = cv2.inRange(hsv_frame, white_lower, white_upper) #mascara donde solo quedará en blanco lo blanco
    #mask = cv2.bitwise_and(frame, frame, mask=mask)
    mask = cv2.erode(mask,np.ones((3,3), np.uint8))
    mask = cv2.dilate(mask, np.ones((5,5), np.uint8))
    mask = cv2.erode(mask,np.ones((4,4), np.uint8))
    mask = cv2.dilate(mask, np.ones((3,3), np.uint8))
    kernel=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    _, umbral = cv2.threshold(mask, 250, 255, cv2.THRESH_BINARY)
    
    # find contours in the binary image
    contours, hirarchy= cv2.findContours(umbral,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    centroid_x_list = []  # Lista para almacenar las coordenadas x de los centroides
    centroid_y_list = []
    for c in contours:
    # calculate moments for each contour
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroid_x_list.append(cX)
            centroid_y_list.append(cY)
            
            
        else:
            cX, cY = 0, 0

    if centroid_x_list and centroid_y_list is not None:
        avg_cX =int(np.mean(centroid_x_list))
        avg_cY = int(np.mean(centroid_y_list))
        cv2.circle(mask, (avg_cX, avg_cY), 60, (255, 255, 255), -1)
    
    # Display images
    cv2.imshow("White mask", mask)
    return mask



































import cv2

cap = cv2.VideoCapture(1,cv2.CAP_DSHOW) 
#cap = cv2.VideoCapture("D:/videos crudos/videos_prueba/op3_pov.WEBM")
#cap = cv2.VideoCapture("D:/videos crudos/videos_prueba/op3_pov.WEBM")
#cap = cv2.VideoCapture("D:/videos crudos/roja_1.mp4")

pelota_clas=cv2.CascadeClassifier('C:/GitHub_Ws/BlendBots_FC/ComputerVision/cascade_manual.xml')
#pelota_clas = cv2.CascadeClassifier('C:/GitHub_Ws/BlendBots_FC/ComputerVision/cascade2ky7.7k.xml')
#pelota_clas = cv2.CascadeClassifier('C:/GitHub_Ws/BlendBots_FC/ComputerVision/soccer_ball_cascade.xml')


while True:
	
	ret,frame = cap.read()
	frame_r=frame#cv2.resize(frame,(960, 540)) #(540,960)
	gray = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
	objects = pelota_clas.detectMultiScale(gray,
	scaleFactor = 1.25,
	minNeighbors = 10)

	# Si se detectó al menos un objeto, tomar el primer rectángulo
	if len(objects) > 0:
		x, y, w, h = objects[0]
		size=w*h
		print(size)
		if size<100000:
			# Dibujar el rectángulo alrededor del primer objeto detectado
			cx=int((x+(x+w))/2)
			cy=int((y+(y+h))/2)
			cv2.rectangle(frame_r, (x, y), (x+w, y+h), (255, 0, 0), 2)
			cv2.circle(frame_r, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)
		else:
			print("blanco")
			frame=line_elimination(frame)
			white=white_f(frame)
	cv2.imshow('frame',frame_r)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()