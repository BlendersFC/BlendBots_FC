 
# import the required library 
import numpy as np 
import cv2 
from matplotlib import pyplot as plt 

def find_top_border(image):
    # Convert the image to HSV
    #soy montse
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the lower and upper bounds for the green color
    lower_green = np.array([0, 39, 0])  # Adjust as needed
    upper_green = np.array([37, 121, 144])

    # Threshold the image to extract the green color
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

     # Display the green mask for visualization
    cv2.imshow("Green Mask", green_mask)

    # Find contours in the green mask
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the contour with the minimum y-coordinate (top border)
        min_y = min(contours, key=lambda c: cv2.boundingRect(c)[1])[0][0][1]
        return min_y
    else:
        return 0  # Default to 0 if no contours are foun



def line_elimination(frame,border):
    green_lower = (0, 0, 127)
    green_upper = (179, 40, 255)

    #frame = cv2.resize(frame, dsize=(470, 640), interpolation=cv2.INTER_CUBIC)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    hsv = cv2.inRange(hsv_frame, green_lower, green_upper)
    mask=hsv
    _, umbral = cv2.threshold(hsv, 250, 255, cv2.THRESH_BINARY)

    
    kernel = np.ones((5,5),np.uint8)
    mask = cv2.dilate(mask,kernel,iterations = 1)
    mask = cv2.erode(mask,kernel,iterations = 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    ## LINEA ENTERA####
    #img = cv2.GaussianBlur(mask, (7, 7), 0)
    edges = cv2.Canny(mask, 50, 150, apertureSize=5)  # limites y matriz de gradiente (impar)
    lines2 = cv2.HoughLines(edges, 1, 3.141592 / 180, 180) 

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
                cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 10)
    return frame


def main():
    # read the image 
    #frame= cv2.imread("C:/Users/lizet/Downloads/CAP.png") 
    cap= cv2.VideoCapture("C:/Users/lizet/Downloads/VIDPRUEBA.mp4")
    white_lower = (0, 0, 180)
    white_upper = (179, 255, 255)
    green_lower = (0, 0, 127)
    green_upper = (179, 40, 255)


    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            break

        border= find_top_border(frame)
        frame=line_elimination(frame,border)

        cv2.line(frame, (0, border), (frame.shape[1], border), (0, 255, 255), 2)

        frame = cv2.resize(frame, dsize=(640, 470), interpolation=cv2.INTER_CUBIC)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv = cv2.inRange(hsv_frame, white_lower, white_upper)
        mask=hsv
        _, umbral = cv2.threshold(hsv, 250, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.erode(mask,kernel,iterations = 1)


        cv2.imshow("gray", mask)
    
        # detect corners with the goodFeaturesToTrack function. 
        corners = cv2.goodFeaturesToTrack(mask, 20, 0.05, 50)  #corners, quality level, maximum distance
        corners = np.int0(corners) 
    
        # we iterate through each corner,  
        # making a circle at each point that we think is a corner. 
        average_x=0
        average_y=0
        totali=0
        for i in corners: 
                x, y = i.ravel() 
                if y> border:
                    average_x +=x
                    average_y +=y
                    totali +=1
                    cv2.circle(frame, (x, y), 3, 255, 5) 

        if totali != 0: 
            average_x = int(average_x/totali)
            average_y= int( average_y /totali)
            cv2.circle (frame,( average_x, average_y), 3, 0, 20)
    
        cv2.imshow("frame", frame)
        # Exit when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()