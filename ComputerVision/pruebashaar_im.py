import cv2

majinBooClassif = cv2.CascadeClassifier('C:/Users/vicen/Documents/Git/BlendBots_FC/ComputerVision/cascade1500y300.xml')


frame=cv2.imread('C:/WhatsApp Unknown 2023-12-09 at 7.58.58 PM/WhatsApp Image 2023-11-09 at 9.33.37 AM.jpeg')
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

toy = majinBooClassif.detectMultiScale(gray,
scaleFactor = 6,
minNeighbors = 100,minSize=(70,78))

for (x,y,w,h) in toy:
    cv2.rectangle(frame, (x,y),(x+w,y+h),(0,255,0),2)
    cx=int((x+(x+w))/2)
    cy=int((y+(y+h))/2)
    cv2.circle(frame, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)
    cv2.putText(frame,'BALÓN',(x,y-10),2,0.7,(0,255,0),2,cv2.LINE_AA)

cv2.imshow('frame',frame)
cv2.waitKey(0)
