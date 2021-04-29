import cv2
import numpy as np

image = cv2.imread('sample.png')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

lower_white = np.array([0, 0, 70])
upper_white = np.array([131, 255, 255])
img = cv2.inRange(hsv, lower_white, upper_white)

xx = 20
while True:
    area = img[430:450, xx:xx+15]
    if cv2.countNonZero(area) > 200:
        image = cv2.rectangle(image, (xx,430), (xx+15, 450), (0,255,0), 3)
    else:
        image = cv2.rectangle(image, (xx,430), (xx+15, 450), (255,0,0), 3)
    xx = xx+20
    if xx > 640:
        break

cv2.imshow('countNonZero', image)
cv2.waitKey(10000)