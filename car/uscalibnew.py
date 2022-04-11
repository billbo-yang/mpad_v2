import cv2
import numpy as np
from filters import ColorThreshholdFilter
import math

import serial



import RPi.GPIO as GPIO

ser=serial.Serial("/dev/ttyACM0",9600)  #change ACM number as found from ls /dev/tty/ACM*



ser.baudrate=9600
us1cords=[None]*80
us2cords=[None]*80
us3cords=[None]*80
us4cords=[None]*80
us5cords=[None]*80
u1=0
u2=0
u3=0
u4=0
u5=0

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#cap.set(cv2.CAP_PROP_FPS, 60)
colorThreshholdFilter = ColorThreshholdFilter()

lower_channels = [255, 255, 255]
higher_channels = [0, 0, 0]
selector_window_name = "Selector"

def checkNewHSVMinMax(h, s, v):
    if (h < lower_channels[0]):
        lower_channels[0] = h
    if (s < lower_channels[1]):
        lower_channels[1] = s
    if (v < lower_channels[2]):
        lower_channels[2] = v
    if (h > higher_channels[0]):
        higher_channels[0] = h
    if (s > higher_channels[1]):
        higher_channels[1] = s
    if (v > higher_channels[2]):
        higher_channels[2] = v

def mouseHSV(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
        colorsH = int(frame_HSV[y, x, 0])
        colorsS = int(frame_HSV[y, x, 1])
        colorsV = int(frame_HSV[y, x, 2])
        checkNewHSVMinMax(colorsH, colorsS, colorsV)
        print(x," ",y)
        input()
        

cv2.namedWindow(selector_window_name)
cv2.setMouseCallback(selector_window_name, mouseHSV)

while(True):
    try:
        read_ser=str(ser.readline())
    except:
        print("error")
    print(read_ser)
    _, frame = cap.read()
    #frame = cv2.imread("../samples/higgins_pink/4_box.jpg")
    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masked = colorThreshholdFilter.apply(frame, np.array(lower_channels), np.array(higher_channels))

    cv2.imshow(selector_window_name, frame)
    #cv2.imshow("Result", masked)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()