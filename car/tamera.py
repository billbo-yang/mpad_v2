import cv2
import numpy as np
from filters import ColorThreshholdFilter, FrameMaskFilter, LineDetectionFilter
import time
from adafruit_servokit import ServoKit
import serial
import RPi.GPIO as GPIO


cap = cv2.VideoCapture(0)

colorThreshholdFilter = ColorThreshholdFilter()

kit = ServoKit(channels=16)
kit.servo[0].angle = 90
kit.continuous_servo[1].throttle = 0


def empty(value):
    pass

result_window_name = "Final Output"
cv2.namedWindow(result_window_name)



while(True):
    t0=time.time()

    # Take each frame
    _, frame = cap.read()
    height, width, channels = frame.shape
    middle=width/2
    frame1init = frame[225:300, 0:int(int(width)/3)]
    frame2init = frame[225:300, int(2*int(width)/3):int(width)]



    frame1 = colorThreshholdFilter.apply(frame1init, [86, 95, 153] , [100, 253, 216])
    frame2 = colorThreshholdFilter.apply(frame2init, [86, 95, 153] , [100, 253, 216])

    
    leftlane=np.mean([coordinate[1] for coordinate in np.argwhere(frame1 == 255)])
    rightlane=(2*int(width)/3)+np.mean([coordinate[1] for coordinate in np.argwhere(frame2 == 255)])

    offsetl=(middle-leftlane)
    offsetr=(rightlane-middle)

    if np.isnan(offsetr):
        offsetr=300
    if np.isnan(offsetl):
        offsetl=300

    offset=offsetr-offsetl
    peroffset=offset/(width)

    if peroffset>0.33:
        peroffset=0.33
    if peroffset<-0.33:
        peroffset=-0.33

    angleset=90+(180*peroffset)

    if angleset < 40:
        angleset=40
    if angleset > 150:
        angleset=150

    if angleset > 60 and angleset < 120:
        angleset=90+(180*peroffset)

    kit.servo[0].angle=angleset
    print(angleset)

    kit.continuous_servo[1].throttle = 0.35




