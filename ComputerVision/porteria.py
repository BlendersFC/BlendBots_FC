import cv2
import numpy as np
import math

gradient = lambda x1,y1,x2,y2: (y2-y1)/(x2-x1)

shot = cv2.VideoCapture(0)

old_x1 = 0
old_y1 = 0
old_x2 = 0
old_y2 = 0
centro_sum = 0
cnt = 0
old_centerx = 0
old_centery = 0

while True:
    ret, frame = shot.read()

    y_frame, x_frame, _ = frame.shape

    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, binarie = cv2.threshold(gray_frame, 85, 255, cv2.THRESH_BINARY)
    canny = cv2.Canny(gray_frame, 0, 100)
    dst = cv2.dilate(canny, (15,15), iterations=5) 

    out1=np.zeros_like(dst)
    out2=np.zeros_like(dst)

    lines = cv2.HoughLinesP(dst, 1, np.pi/180, 150, 10, 100)
        
    for line in lines:
        x1, y1, x2, y2 = line[0]

        m2 = gradient(x1, y1, x2, y2)
        m1 = gradient(x2, y2, old_x1, old_y1)
        angleR = math.atan((m2-m1)/(1+(m2*m1)))
        angleD = round(math.degrees(angleR))

        if (angleD > -50 and angleD < 50) and binarie[y1][x1] and binarie[y2][x2]:

            cv2.line(out1,(x1,y1),(x2,y2),(255,255,255),8)
            cv2.line(frame,(x1,y1),(x2,y2),(255,0,255),8)

            centro = x2 + (x1-x2)/2
            centro_sum = centro_sum + centro
                
            if cnt == 1:
                centro_promedio = int(centro_sum/cnt)
                cv2.line(out2,(centro_promedio,y_frame),(centro_promedio,0),(255,255,255),3)
                #cv2.putText(frame,str(angleD),(x1-40,y1-20),cv2.FONT_HERSHEY_COMPLEX,1.5,(0,0,255),2)
                cnt = 0
                centro_sum = 0
                goal = True
                break
                
            cnt = cnt + 1
        else:
            goal = False
        
        old_x1 = x1
        old_y1 = y1
        old_x2 = x2
        old_y2 = y2
    
    mask = cv2.bitwise_and(out1, out2)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (5, 5))

    x_center,y_center,w_center,h_center = cv2.boundingRect(opening)

    centerx = int(x_center+(w_center/2))
    centery = int(y_center+(h_center/2))

    if centerx != 0 and centery != 0:
        cv2.circle(frame, (centerx, centery), 5,  (255,255,255), 2)
        old_centerx = centerx
        old_centery = centery
    elif centerx == 0 and centery == 0 and goal:
        #print(centerx, centery)
        cv2.circle(frame, (old_centerx, old_centery), 5,  (255,255,255), 2)

    try:
        cv2.imshow('Video', out2)
    except:
        pass

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
shot.release()
cv2.destroyAllWindows()