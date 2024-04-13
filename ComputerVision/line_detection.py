#!/usr/bin/env python2.7

import numpy as np
import cv2

def line_elimination(frame,border):
    green_lower = (0, 44, 0)
    green_upper = (49, 180, 180)

    white_lower = (0, 0, 0)
    white_upper = (180, 82, 255)

    #frame = cv2.resize(frame, dsize=(470, 640), interpolation=cv2.INTER_CUBIC)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hsv = cv2.inRange(hsv_frame, white_lower, white_upper)
    mask=hsv
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.dilate(mask,kernel,iterations = 1)
    mask = cv2.erode(mask,kernel,iterations = 1)

    ## LINEA ENTERA####
    #img = cv2.GaussianBlur(mask, (7, 7), 0)
    edges = cv2.Canny(mask, 30, 80, apertureSize=7)  # limites y matriz de gradiente (impar)
    lines2 = cv2.HoughLines(edges, 1, 3.141592 / 180, 100) #image, pixels, theta, threshold

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
            if ( y1) > border and (y2) > border:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 10)
        edges = cv2.Canny(mask, 30, 80, apertureSize=7)  # limites y matriz de gradiente (impar)

    '''edges = cv2.Canny(mask, 30, 80, apertureSize=5)  # limites y matriz de gradiente (impar)
    lines2 = cv2.HoughLines(edges, 1,np.pi / 4, 80) #image, pixels, theta, threshold

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
            if ( y1) > border or (y2) > border:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 20)'''
    return frame
