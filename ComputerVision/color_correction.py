import cv2

# load and resize image
input_image = cv2.imread("D:/videos crudos/videos_prueba/i5.jpg")
input_image = cv2.resize(input_image,(960, 540))
# Create retinex object
retinex = cv2.xphoto.createSimpleWB()

# Apply white balance
balanced_image = retinex.balanceWhite(input_image)

# show results
cv2.imshow('Original Image', input_image)
cv2.imshow('Balanced Image (Retinex)', balanced_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
