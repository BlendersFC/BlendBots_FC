
import cv2

#cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) 
#cap = cv2.VideoCapture('C:/Users/vicen/Downloads/WIN_20231208_15_17_00_Pro.mp4')
#cap = cv2.VideoCapture('C:/Users/vicen/Downloads/WIN_20231208_15_16_32_Pro.mp4')
cap = cv2.VideoCapture('C:/Users/vicen/Downloads/WIN_20231117_11_36_57_Pro (1).mp4')

pelota_clas = cv2.CascadeClassifier('C:/Users/vicen/Documents/Git/BlendBots_FC/ComputerVision/cascade1500y300.xml')

while True:
	
	ret,frame = cap.read()
	frame_r=cv2.resize(frame,(960, 540))
	gray = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

	#parece que con estos parámetros va bien pero habrá que ver con el OP3
	ball = pelota_clas.detectMultiScale(gray,
	scaleFactor = 6,
	minNeighbors = 100, minSize=(100,100))

	for (x,y,w,h) in ball:
		cv2.rectangle(frame_r, (x,y),(x+w,y+h),(0,255,0),2)
		cx=int((x+(x+w))/2)
		cy=int((y+(y+h))/2)
		cv2.circle(frame_r, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)

	cv2.imshow('frame',frame_r)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()