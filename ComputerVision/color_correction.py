import cv2
import numpy as np


def filter(img):
    retinex = cv2.xphoto.createSimpleWB()
    balanced_image = retinex.balanceWhite(img)
	
    balanced_image_gray = cv2.cvtColor(balanced_image, cv2.COLOR_BGR2GRAY)
    equa = cv2.equalizeHist(balanced_image_gray)

    kernel = np.ones((5, 5), np.uint8)  #kernel for morfologic
    imagen_erosionada = cv2.erode(balanced_image_gray, kernel, iterations=1) #erosion
    apertura = cv2.dilate(imagen_erosionada, kernel, iterations=1)#dilatation
    apertura_c = cv2.cvtColor(apertura,cv2.COLOR_GRAY2BGR) #grayscale to BGR


    return [balanced_image,balanced_image_gray,equa,apertura_c]




cap = cv2.VideoCapture("D:/videos crudos/videos_prueba/op3_pov.webm")
while True:
	
	ret,frame_final = cap.read()
	frame_final=filter(frame_final)[3]
	cv2.imshow('frame',frame_final)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()
