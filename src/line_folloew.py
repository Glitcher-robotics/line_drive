# !/usr/bin/env python
import rospy, rospkg
import numpy as np

import cv2, time
import math, os

from xycar_motor.msg import xycar_motor #motor topic
from sensor_msgs.msg import Image # import camera topic
from cv_bridge import CvBridge

# path = rospkg.Rospack().get_path('line_drive')+"/src/track.avi"
# cap = cv2.VideoCapture(path)

bridge = CvBridge()
cv_image = np.empty(shape=[0]) # ROS Image topic for transforming to image data

threshold_60 = 60
threshold_100 = 100 # for image binary, the number from experiment

width_640 = 640
scan_width_200, scan_height = 200, 20 # the size to scanning

lmid_200, rmid_440 = scan_width_200, width_640 - scan_width_200 # the end of scanning

area_width_20, area_height_10 = 20, 10 # the size of scanning the white pixel

vertical_430 = 430 # the upper limit to setting ROI
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10 # coordinate's begin and end of pixel scanning size of ROI

pixel_threshold_160 = 0.8 * area_width_20 * area_height_10 # the lower limit of white pixel to judge line


def img_callback(img_data): # callback function
    global cv_image
    global bridge
    cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8") # ROS topic image to cv2 image


def drive(Angle, Spee): # topic publisher
    global pub

    msg = xycar_motor
    msg.angle = Angle
    msg.speed = Speed #xycar motor topic publish with Angle % Speed

    pub.publish(msg)


def start():
    global pub
    global cv_image

    rospy.init_node('auto_driver') # init ROS node
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    while not rospy.is_shutdown():

        while not cv_image.size == (640*480*3): # wait first image
            continue

        frame = cv_image # transformed cv2 image data by callback function move to frame
        
        if cv2.waitKey(1) & 0xFF == 27: #waiting for ESC Key
            break

        roi = frame[vertical_430:vertical_430+scan_height_20, :]
        frame = cv2.rectangle(frame, (0, vertical_430), (width_640 - 1, vertical_430 + scan_height_20), (255, 0, 0), 3)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV) # setting with long line shape, from color image to HSV

        lbound = np.array([0, 0, threshold_100], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.unit8) #set the upper, lower limit,, white & black binary

        bin = cv2.inRange(hsv, lbound, ubound) # white/black binary image

        view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR) # color image (for green rectangle)

        left, right = -1, -1

        for l in range(area_width_20, lmid_200): # one pixel move
            area = bin[row_begin_5:row_end_15, l - area_width_20:l]
            if cv2.countNonZero(area) > pixel_threshold_160:
                left = l
                break

        for r in range(width_640 - area_width_20, rmid_440, -1):
            area = bin[row_begin_5:row_end_15, r:r+area_width_20]
            if cv2.countNonZero(area) > pixel_threshold_160:
                right = r
                break # cv2.countNonZero() : return the pixel number that binary number is not zero in given image


        left, right = -1, -1

        for l in range(area_width_20, lmid_200):
            area = bin[row_begin_5:row_end_15, l - area_width_20:l]
            if cv2.countNonZero(area) > pixel_threshold_160:
                left = l
                break # comparison with lower limit(80% of area)

        for r in range(width_640 -area_width_20, rmid_440, -1):
            area = bin[row_begin_5:row_end_15, r:r+area_width_20]
            if cv2.countNonZero(area) > pixel_threshold_160:
                right = r
                break

        if left != -1:
            lsquare = cv2.rectangle(view, (left - area_width_20, row_begin_5), (0, 255, 0), 3)

        else:
            print("Lost left line")
        
        if right != -1:
            rsquare = cv2.rectangle(view, (right, row_begin_5), (right+area_width_20, row_end_15), (0,255,0), 3)
        
        else:
            print("Lost right line") # if right line detected, draw the green rectangle on the ROI veiw image

        cv2.imshow("origin", frame)
        cv2.imshow("view", view) #Display

        center = (right+left)/2 # detect the middle line
        shift = center - 320 # put the difference in the shift

        Angle = shift/3 # angle = shift / 3
        if Angle < -50:
            Angle = -50
        if Angle > 50:
            Angle = 50

        Speed = 20 # speed is fixed by 20
        drive(Angle, Speed) # call pub_motor(), publish xycar_motor

    cap.release()
    cv2.destroyAllWindows()
    
if __name__ == '__main__': #main funcition

    start()









