import cv2 as cv
import numpy as np

img = cv.imread("/home/dheeraj/catkin_ws/src/drone_controller/bagfiles/image_1.png")

print(len(img[0][0]))
# cv.imshow("Image", img)