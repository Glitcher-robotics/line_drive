#!/usr/bin/env python

import cv2, time
import numpy as np

cap = cv2.VideoCapture('track2.avi')

threshold_60 = 150
threshold_100 = 100

width_640 = 640
scan_width_200, scan_height_20 = 200, 20
lmid_200, rmid_440 = scan_width_200, width_640 - scan_width_200
area_width_20, area_height_10 = 20, 10
vertical_430 = 430
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10
pixel_threshold_160 = 0.8 * area_width_20 * area_height_10

while True:
    ret, frame = cap.read()
    if not ret:
        break
    if cv2.waitKey(1) & 0xFF == 27:
        break

    roi = frame[vertical_430:vertical_430 + scan_height_20, :]
    frame = cv2.rectangle(frame, (0, vertical_430),
        (width_640 - 1, vertical_430 + scan_height_20),
        (255, 0, 0), 3)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, threshold_60], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    bin = cv2.inRange(hsv, lbound, ubound)
    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

    left, right = -1, -1

    for l in range(area_width_20, lmid_200):
        area = bin[row_begin_5:row_end_15, l - area_width_20:l] 
        if cv2.countNonZero(area) > pixel_threshold_160:
            left = l
            break

    for r in range(width_640 - area_width_20, rmid_440, -1):
        area = bin[row_begin_5:row_end_15, r:r + area_width_20]
        if cv2.countNonZero(area) > pixel_threshold_160:
            right = r
            break

    if left != -1:
        lsquare = cv2.rectangle(view,
                                (left - area_width_20, row_begin_5),
                                (left, row_end_15),
                                (0, 255, 0), 3)
    else:
        print("Lost left line")

    if right != -1:
        rsquare = cv2.rectangle(view,
                                (right, row_begin_5),
                                (right + area_width_20, row_end_15),
                                (0, 255, 0), 3)
    else:
        print("Lost right line")

    cv2.imshow("origin", frame)
    cv2.imshow("view", view)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lbound = np.array([0, 0, threshold_60], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)   
    hsv = cv2.inRange(hsv, lbound, ubound)
    cv2.imshow("hsv", hsv) 

    time.sleep(0.1)

cap.release()
cv2.destroyAllWindows()

