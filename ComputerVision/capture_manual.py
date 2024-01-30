import cv2
import os

# Ruta de la carpeta con las imágenes originales
ruta_origen = "D:/imagenes crudas/"

# Ruta de la carpeta donde se guardarán las imágenes recortadas
ruta_destino_pos = "D:/Img/pos_manual/"
ruta_destino_neg= "D:/Img/neg_manual/"
# Obtener la lista de archivos en la carpeta de origen
archivos = os.listdir(ruta_origen)

# Comenzar desde la imagen número 57
for indice, nombre_archivo in enumerate(archivos[3493:], start=3494 ):
    if nombre_archivo.endswith((".jpg", ".png")):
        # Cargar la imagen original
        imagen = cv2.imread(os.path.join(ruta_origen, nombre_archivo))
        #image_r = cv2.rotate(imagen, cv2.ROTATE_90_CLOCKWISE)
        #image_r = cv2.rotate(imagen, cv2.ROTATE_180)
        imagen= cv2.resize(imagen,(1080,720))

        # Seleccionar un rectángulo de interés con el cursor del ratón
        rectangulo = cv2.selectROI("Selecciona el área", imagen, fromCenter=False, showCrosshair=True)

        # Recortar la imagen según las coordenadas del rectángulo
        imagen_recortada = imagen[int(rectangulo[1]):int(rectangulo[1] + rectangulo[3]),
                                  int(rectangulo[0]):int(rectangulo[0] + rectangulo[2])]

        #Guardar la imagen recortada en la carpeta de destino
        imagen_guardada = os.path.join(ruta_destino_pos, nombre_archivo)
        cv2.imwrite(imagen_guardada, imagen_recortada)

        print(f"Imagen pos recortada guardada en {imagen_guardada}")



print("Proceso completado. Imágenes recortadas guardadas en la carpeta de destino.")
