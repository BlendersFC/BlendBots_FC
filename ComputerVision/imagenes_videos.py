from PIL import Image
import os

def resize_images(image_directory, width, height):
    for filename in os.listdir(image_directory):
        if filename.endswith(".jpg") or filename.endswith(".png"): 
            img = Image.open(os.path.join(image_directory, filename))
            new_img = img.resize((width, height))
            new_img.save(os.path.join(image_directory, "resized_" + filename))

# Llama a la función con la ruta de tu directorio de imágenes
resize_images('C:/Users/vicen/Documents/Git/data_video/positivas_sd', 38, 46)