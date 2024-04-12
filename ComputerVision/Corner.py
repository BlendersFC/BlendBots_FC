 
# import the required library 
import numpy as np 
import cv2 
#from cv2 import xphoto
from matplotlib import pyplot as plt 

def find_top_border(image):
    # Convert the image to HSV
    #soy montse
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
   
    # Define the lower and upper bounds for the green color
    #lower_green = np.array([0, 39, 0])  # Adjust as needed
    #upper_green = np.array([37, 121, 144])

    lower_green = (0, 111, 0)
    upper_green = (58, 255, 255)

    # Threshold the image to extract the green color
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    kernel = np.ones((5,5),np.uint8)
    green_mask = cv2.dilate(green_mask,kernel,iterations = 1)
    green_mask = cv2.erode(green_mask,kernel,iterations = 3)

     # Display the green mask for visualization
    cv2.imshow("Green Mask", green_mask)

    # Find contours in the green mask
    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Filtrar contornos por área mínimaq
    min_contour_area= 1000
    contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

    if contours:
        # Find the contour with the minimum y-coordinate (top border)
        min_y = min(contours, key=lambda c: cv2.boundingRect(c)[1])[0][0][1]
        return min_y
    else:
        return 0  # Default to 0 if no contours are foun



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

    y2_prev2=0
    y1_prev2=0

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
            
            if ( y1) > border and (y2) > border and (abs(y2_prev2-y2))>2 and (abs(y1_prev2-y1))>2:
                cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 3)
            y2_prev2=y2
            y1_prev2=y1

    
    edges2 = cv2.Canny(mask, 20, 80, apertureSize=7)  # limites y matriz de gradiente (impar)   
    lines = cv2.HoughLinesP(edges2,1, 3.141592 /180,10,1,30)
    y2_prev=0
    y1_prev=0
    if lines is not None:

        N = lines.shape[0]
        for i in range(N):
            x1 = lines[i][0][0]
            y1 = lines[i][0][1]    
            x2 = lines[i][0][2]
            y2 = lines[i][0][3]
            if y1>border and y2 >border and (abs(y2_prev-y2))>3 and (abs(y1_prev-y1))>3 :    
                cv2.line(frame,(x1,y1),(x2,y2),(255,255,255),3)
            y2_prev=y2
            y1_prev=y1
    
    lines_combined = []
    lines_combined.extend([line[0] for line in lines])
    lines_combined.extend(lines2)
    
    return frame,lines_combined

def intersection(line1, line2):
    """
    Encuentra la intersección entre dos líneas especificadas en la forma de Hesse normal.

    Devuelve las coordenadas del punto de intersección.

    Referencia:
    https://stackoverflow.com/a/383527/5087436
    """
    #rho1, theta1 = line1[0]
    #rho2, theta2 = line2[0]

    try:
        rho1, theta1 = line1
    except TypeError:
        rho1, theta1 = 0,0
    
    try:
        rho2, theta2 = line2
    except TypeError:
        rho2, theta2 = 0,0

    angle_rad = abs(theta1 - theta2)
    angle_deg = np.degrees(angle_rad)
    if angle_deg>45:        
        A = np.array([[np.cos(theta1), np.sin(theta1)],
                    [np.cos(theta2), np.sin(theta2)]])
        b = np.array([[rho1], [rho2]])
        
        try:
            x0, y0 = np.linalg.solve(A, b)
        except np.linalg.LinAlgError:
            # Las líneas son paralelas, no hay intersección
            return None
        
        x0, y0 = int(np.round(x0)), int(np.round(y0))
        return (x0, y0)
    return(1,1)

def segmented_intersections(frame,lines):
    """
    Encuentra la intersección entre todas las combinaciones de líneas en grupos de líneas.

    Devuelve una lista de puntos de intersección.
    """
    intersections = []
    for i, group in enumerate(lines[:-1]):
        for next_group in lines[i+1:]:
            for line1 in group:
                for line2 in next_group:
                    intersec_point = intersection(line1, line2)
                    if intersec_point:
                        intersections.append(intersec_point)
    
    if intersections is not None:
        for point in intersections:
            print(type(point))
            try:
                cv2.circle(frame, point, 10, (255, 0, 0), -1  )
            except cv2.error:
                continue

    return frame, intersections

def main():
    # read the image 
    #frame= cv2.imread("C:/Users/lizet/Downloads/CAP.png") 
    cap= cv2.VideoCapture("C:/Users/lizet/Downloads/Crop.mp4")



    white_lower = (0, 0, 0)
    white_upper = (180, 82, 255)
    #green_lower = (0, 0, 127)
    #green_upper = (179, 40, 255)
    green_lower = (0, 44, 0)
    green_upper = (49, 200, 180)


    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("original", frame)

        border= find_top_border(frame)
        frame,lines=line_elimination(frame,border)
        if lines is not None:
            frame,intersection= segmented_intersections(frame,lines)
        
        

        cv2.line(frame, (0, border), (frame.shape[1], border), (0, 255, 255), 2)

        frame = cv2.resize(frame, dsize=(640, 470), interpolation=cv2.INTER_CUBIC)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv = cv2.inRange(hsv_frame, white_lower, white_upper)
        mask=hsv
        _, umbral = cv2.threshold(hsv, 250, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask = cv2.erode(mask,kernel,iterations = 1)

        cv2.imshow("white", mask)
    
        '''# detect corners with the goodFeaturesToTrack function. 
        corners = cv2.goodFeaturesToTrack(mask, 3, 0.9, 200)  #corners, quality level, maximum distance
        
        if corners is not None:
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
                cv2.circle (frame,( average_x, average_y), 3, 0, 20)'''
        

    
        cv2.imshow("frame", frame)
        # Exit when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()
if __name__ == "__main__":
    main()