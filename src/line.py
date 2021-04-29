import cv2
import numpy as np 

image = cv2.imread('sample.png')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

lower_white = np.array([0, 0, 70])
upper_white = np.array([131, 255, 255])

mask = cv2.inRange(hsv, lower_white, upper_white)

cv2.imshow('line', mask)
cv2.waitKey(10000)