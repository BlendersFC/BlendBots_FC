


import cv2

cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) 
#cap = cv2.VideoCapture("D:/videos crudos/pelota blanca/media1.mp4")
#cap = cv2.VideoCapture("D:/videos crudos/pelota blanca/media2.mp4")
cap = cv2.VideoCapture("D:/videos crudos/pelota_azul/azul15.mp4")


#classifier for complete and incomplete balls
ball_clas=cv2.CascadeClassifier('ComputerVision/cascade_manual.xml')
half_clas=cv2.CascadeClassifier('ComputerVision/half_cascade.xml')

#which classifier
half_boolean=True

while True:
	
	ret,frame = cap.read()

	frame_r=cv2.resize(frame,(960, 540)) #(540,960)
	gray = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
	if half_boolean:
		objects = ball_clas.detectMultiScale(gray,
		scaleFactor = 1.25,
		minNeighbors = 10)
	else:
		objects = half_clas.detectMultiScale(gray,
		scaleFactor = 1.25,
		minNeighbors = 2)	

	# Si se detectó al menos un objeto, tomar el primer rectángulo
	if len(objects) > 0:
		x, y, w, h = objects[0]
		size=w*h

       # Dibujar el rectángulo alrededor del primer objeto detectado
		cx=int((x+(x+w))/2)
		cy=int((y+(y+h))/2)
		cv2.rectangle(frame_r, (x, y), (x+w, y+h), (255, 0, 0), 2)
		cv2.circle(frame_r, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)

	cv2.imshow('frame',frame_r)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()