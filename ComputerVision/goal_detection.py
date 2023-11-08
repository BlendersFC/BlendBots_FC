import cv2 
import numpy as np
from matplotlib import pyplot as plt



def main():

      

     #Webcam video
     vc = cv2.VideoCapture(0)

     ret, frame = vc.read()

     small_frame = cv2.resize(frame, (160,120)) # resize to lower resolution

     saliency_hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)

     # 0 - 255 grayscale channel of HSV
     value_channel = saliency_hsv[:, :, 2]

     # generate hist
     hist = cv2.calcHist([value_channel], [0], None, [256], [0, 256])


     #generate histogram
     #hist = cv2.calcHist([saliency_hsv],[0, 1, 2], None, [180, 256, 256], [0, 180, 0, 256, 0, 256])

   
     # Reshape to 2D
     hist_2d = hist.reshape(-1) 

    # Plot in color
     plt.xlabel('Value') 
     plt.ylabel('# of pixels')
     plt.xlim(0,255)
     plt.plot(hist_2d)


     plt.show()


if __name__ == "__main__":
     main()