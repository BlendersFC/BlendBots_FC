import cv2
import math
import numpy as np

def apply_mask(matrix, mask, fill_value):
    masked = np.ma.array(matrix, mask=mask, fill_value=fill_value)
    return masked.filled()

def apply_threshold(matrix, low_value, high_value):
    low_mask = matrix < low_value
    matrix = apply_mask(matrix, low_mask, low_value)

    high_mask = matrix > high_value
    matrix = apply_mask(matrix, high_mask, high_value)

    return matrix

def simplest_cb(img, percent):
    #check parameters, image RGB and percentage betweeen 0 and 100
    assert img.shape[2] == 3
    assert percent > 0 and percent < 100

    #divide to get half_percent for bottom values and half for high values
    half_percent = percent / 200.0

    #get 3 channels
    channels = cv2.split(img)

    #corrected color channels list
    out_channels = []

    for channel in channels:
        #height and width only
        assert len(channel.shape) == 2
        # find the low and high precentile values (based on the input percentile)
        height, width = channel.shape
        vec_size = width * height #total number of pixels
        flat = channel.reshape(vec_size) #get values in flat distribution

        #chechl that flat is flat
        assert len(flat.shape) == 1

        #sort values
        flat = np.sort(flat)

        #total pixels
        n_cols = flat.shape[0]

        #getting high and low values for threshold based on parameters percent
        low_val  = flat[math.floor(n_cols * half_percent)]
        high_val = flat[math.ceil( n_cols * (1.0 - half_percent))]


        # saturate below the low percentile and above the high percentile
        thresholded = apply_threshold(channel, low_val, high_val)
        # scale the channel
        normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
        out_channels.append(normalized)

    return cv2.merge(out_channels)

def grenpp(image):
    b, g, r=cv2.split(image)
    green=250-g
    corrected_img=cv2.merge([b,green,r])
    return corrected_img
def histogram(img):
    histogram_array=np.zeros(256)
    height, width = img.shape

    for id_rows in range(height):
      for id_cols in range (width):
        intensity = img[id_rows][id_cols]
        histogram_array[intensity]+=1
    histogram_array/= (height*width) #se divide entre el número total de pixeles para que el resultado esté normalizaod
    return histogram_array



#histograma acumulativo normalizado
def histogram_a(img):
    histogram_array_a=np.zeros(256)
    height, width = img.shape

    for id_rows in range(height):
      for id_cols in range (width):
        intensity = img[id_rows][id_cols]
        histogram_array_a[intensity]+=1
    histogram_array_a/=(height*width) #se divide entre el número total de pixeles para que el resultado esté normalizaod

    #Se recorre el histograma en cada intensidad y se va sumando el valor anterior al conteo de esa intensidad
    for i in range(1, 256):
        histogram_array_a[i] += histogram_array_a[i - 1]
    return histogram_array_a



def ecualizador(img):
  #histograma acumulativo normalizado
  histograma_acumulado = histogram_a(img)

  # Mapea los valores de intensidad
  height, width = img.shape
  img_ecualizada = np.zeros_like(img) #imagen base para la ecualizada
  for id_rows in range(height):
    ###############
    #fragmento de código tomado de Copilot
      for id_cols in range(width):
          intensidad = img[id_rows][id_cols] #recorrer cada pixel y tomar su intensidad (0-255)
          nueva_intensidad = int(255 * histograma_acumulado[intensidad]) #multiplicar 255 por el acumulado de cada intensidad, reduce las intensidades muy altas y amplifica las que son muy pequeñas
          img_ecualizada[id_rows][id_cols] = nueva_intensidad #asignar a la imiagen ecualizada
    ################
  return img_ecualizada

cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) 
#cap = cv2.VideoCapture("D:/videos crudos/pelota_azul/azul15.mp4")
ball_clas=cv2.CascadeClassifier('cascade_manual.xml')
half_clas=cv2.CascadeClassifier('half_cascade.xml')
while True:
	
	ret,frame = cap.read()

	frame_r=cv2.resize(frame,(960, 540)) #(540,960)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	objects = ball_clas.detectMultiScale(gray,
	scaleFactor = 1.25,
	minNeighbors = 10)

	# Take first rectangle if object detected
	if len(objects) > 0:
		x, y, w, h = objects[0]
		size=w*h

       #draw rectangle around ball
		cx=int((x+(x+w))/2)
		cy=int((y+(y+h))/2)
		cv2.rectangle(frame_r, (x, y), (x+w, y+h), (255, 0, 0), 2)
		cv2.circle(frame_r, (cx, cy), radius=5, color=(0, 255, 0), thickness=1)

	cv2.imshow('frame',frame_r)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()
    

     
    