import cv2
import numpy as np
from filters import ColorThreshholdFilter, FrameMaskFilter, LineDetectionFilter
import time
from adafruit_servokit import ServoKit
import serial
import RPi.GPIO as GPIO
import argparse
import time
import base64
from datetime import datetime
import json

cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#cap.set(cv2.CAP_PROP_FPS, 60)
colorThreshholdFilter = ColorThreshholdFilter()

kit = ServoKit(channels=16) #Initializes the servo shield
kit.servo[0].angle = 90 # Sets wheels forward
kit.continuous_servo[1].throttle = 0 # Sets speed to zero


def empty(value):
    pass

# These are calibration numbers, it is the output of another program. The values in the array represent the minimum and maximum x coordinates of the 
# pixels in the camera input of every reading from the ultrasonic. These numbers should be updated whenever the module is installed on another car.
# The following numbers are just examples. We will recalibrate once we get a newly 3d printed car.
# us1calib=[None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, (176, 179), (36, 204), (21, 197), (0, 169), (0, 167), (0, 169), None, (156, 303), (164, 299), (170, 297), (172, 299), (140, 306), (93, 308), (94, 309), (0, 180), (40, 235), (40, 234), (64, 262), (52, 247), (68, 263), (16, 282), (10, 281), (11, 277), (12, 277), (108, 273), (12, 271), (12, 306), (10, 305), (248, 351), (15, 322), (29, 324), (180, 325), (106, 325), (32, 327), (248, 351), (10, 280), (11, 195), (55, 187), (11, 191), (55, 191), (10, 253), (15, 253), (15, 252), None, (10, 257), (11, 262), (24, 264), (10, 265), (55, 195), (65, 212), (64, 214), (55, 204), (64, 230), (11, 243), (55, 227), (48, 147), (56, 179), (56, 181), (52, 151), None, None, None, None, None]
# us2calib=[None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, (176, 179), (36, 204), (21, 197), (0, 169), (0, 167), (0, 169), None, (156, 303), (164, 299), (170, 297), (172, 299), (140, 306), (93, 308), (94, 309), (0, 180), (40, 235), (40, 234), (64, 262), (52, 247), (68, 263), (16, 282), (10, 281), (11, 277), (12, 277), (108, 273), (12, 271), (12, 306), (10, 305), (248, 351), (15, 322), (29, 324), (180, 325), (106, 325), (32, 327), (248, 351), (10, 280), (11, 195), (55, 187), (11, 191), (55, 191), (10, 253), (15, 253), (15, 252), None, (10, 257), (11, 262), (24, 264), (10, 265), (55, 195), (65, 212), (64, 214), (55, 204), (64, 230), (11, 243), (55, 227), (48, 147), (56, 179), (56, 181), (52, 151), None, None, None, None, None]
# us3calib=[None, None, None, None, None, (232, 343), (200, 233), (436, 437), (346, 411), (316, 439), (293, 391), (293, 363), (224, 287), (331, 379), (269, 344), (246, 328), (242, 403), (264, 337), (316, 359), (298, 389), (409, 439), (406, 439), (248, 407), (397, 423), (312, 407), (296, 411), (387, 415), (405, 406), (386, 417), (406, 416), (240, 407), (292, 387), (236, 405), (420, 425), (391, 409), (316, 351), (403, 439), (418, 428), (228, 267), (387, 435), (388, 421), (251, 289), (197, 267), (268, 312), (240, 293), (232, 263), (348, 369), (365, 386), (352, 407), (190, 243), (360, 403), (360, 394), (358, 406), (360, 394), (268, 292), (300, 388), (268, 271), (257, 259), (272, 323), (439, 439), (288, 313), (358, 363), (422, 423), (405, 405), (264, 287), None, (405, 405), (271, 293), (248, 407), (282, 300), (416, 427), (364, 371), (377, 386), (426, 438), (231, 248), (328, 347), (396, 408), (231, 250), (348, 361), (388, 402), None, None, None, None, None]



while(True):
    t0 = time.time() # this is just for debugging and making sure the loop time is low
    #read_ser=str(ser.readline())
    #read_ser=""

    speed=0.15 #sets the throttle

    etc_file = open('/etc/selfdriving-rc/serial-out', 'r')
    read_ser=etc_file.read()

    
    # these turn the ultrasonic inputs from the arduino from strings into integers







    u1=int(read_ser[slice(read_ser.find("Distance 1:")+len("Distance 1:"),read_ser.find('.'))])







    read_ser=read_ser[slice(read_ser.find('.')+1,len(read_ser))]







    u2=int(read_ser[slice(read_ser.find("Distance 2:")+len("Distance 2:"),read_ser.find('.'))])







    read_ser=read_ser[slice(read_ser.find('.')+1,len(read_ser))]







    u3=int(read_ser[slice(read_ser.find("Distance 3:")+len("Distance 3:"),read_ser.find('.'))])







    read_ser=read_ser[slice(read_ser.find('.')+1,len(read_ser))]







    u4=int(read_ser[slice(read_ser.find("Distance 4:")+len("Distance 4:"),read_ser.find('.'))])







    read_ser=read_ser[slice(read_ser.find('.')+1,len(read_ser))]







    u5=int(read_ser[slice(read_ser.find("Distance 5:")+len("Distance 5:"),read_ser.find('.'))])





    if u1 ==0:







        u1=100







    if u2 ==0:







        u2=100







    if u3 ==0:







        u3=100







    if u4 ==0:







        u4=100







    if u5 ==0:







        u5=100




    us1min=[0,0]







    us1max=[2.25,69.4]















    us2min=[0.61142,155.6]







    us2max=[1,99999,181.4]















    us3min=[-0.4524,244.344]







    us3max=[0.12459,320.5573]















    us4min=[-2.7716,403.2237]







    us4max=[-1.10155,457.81]















    us5min=[-0.3999999,463.99]




    _, frame = cap.read()
    height, width, channels = frame.shape


    us5max=[0,width]


    read_ser=read_ser[slice(read_ser.find('.')+1,len(read_ser))]

    # Take each frame




    middle=width/2
    # The following code segments the camera input to do different calculations
    frame1init = frame[200:300, 0:int(int(width)/3)]
    frame3 = frame[(height-60):(height-20), int(int(width)/3):int(2*int(width)/3)] #Frame 3 gets the bottom of the screen
    frame2init = frame[200:300, int(2*int(width)/3):int(width)]
    frame4init = frame[(height-250):(height-100),int(int(width)/3):int(middle)]
    frame5init = frame[(height-250):(height-100),int(middle):int(2*int(width)/3)]

    # these are the color filters. The values are the RGB min and max values. the color filter makes every pixel in that range white and everything else black
    frame1 = colorThreshholdFilter.apply(frame1init, [86, 95, 153] , [100, 253, 216])
    frame2 = colorThreshholdFilter.apply(frame2init, [86, 95, 153] , [100, 253, 216])
    frame4 = colorThreshholdFilter.apply(frame4init, [86, 95, 153] , [100, 253, 216])
    frame5 = colorThreshholdFilter.apply(frame5init, [86, 95, 153] , [100, 253, 216])
    cv2.imshow("Frame", frame)
    cv2.imshow("Frame 1", frame1)
    cv2.imshow("Frame 2", frame2)
    cv2.imshow("Frame 4", frame4)
    cv2.imshow("Frame 5", frame5)


    '''
    # These are for colors that make the car go faster or slower.
    frame3slow = colorThreshholdFilter.apply(frame3, [92, 115, 132], [102, 255, 199])
    frame3fast = colorThreshholdFilter.apply(frame3, [97, 136, 144], [100, 247, 199])

    if len(np.argwhere(frame3slow == 255)) >20:
        speed=speed
    if len(np.argwhere(frame3fast == 255)) >20:
        speed=speed
    '''
    # The timeout in the arduino code sets the ultrasonic to 0 meaning that the true value of a 0 reading is undetected. 200 is a good value for the calculations.



    # These two look at the color filtered images and gets the median of the lanes.
    leftlane=np.median([coordinate[1] for coordinate in np.argwhere(frame1 == 255)])
    rightlane=(2*int(width)/3)+np.median([coordinate[1] for coordinate in np.argwhere(frame2 == 255)])














    if np.isnan(leftlane):







        leftlane=0







    if np.isnan(rightlane):







        rightlane=width





    if u1<40 and u2<60 and u4<60 and u5>40:

        u4=100



    if u1>40 and u2<60 and u4<60 and u5<40:

        u2=100



    if u1<40 and u4 <60:

        u4=100



    if u5<40 and u2<60:

        u2=100



    if u2<60 and u4<60:

        if u2<u4:

            u4=100

        else:

            u2=100





    if u1<40:





        leftlane =max(leftlane,int(us1max[0]*u1+us1max[1]))



    if u1<25:





        leftlane =middle







    if u2<60:







        leftlane =max(leftlane,int(us2max[0]*u2+us2max[1]))











    if u5<25:



        rightlane = middle





    if u5<40:



        rightlane = min(rightlane,int(us5min[0]*u5+us5min[1]))





    if u4<60:







        rightlane = min(rightlane,int(us4min[0]*u4+us4min[1]))






    offsetl=(middle-leftlane)
    offsetr=(rightlane-middle)


    # This code just sees the difference from the middle

    # If no pixels are detected it means that the lane is far away (out of camera FOV). This code just sets the lane as far away.
    if np.isnan(offsetr):
        offsetr=middle+0.2*middle
    if np.isnan(offsetl):
        offsetl=middle+0.2*middle

    # This code just turns the offsets into a percentage value
    offset=offsetr-offsetl
    peroffset=offset/(width)

    if len(np.argwhere(frame4 == 255)) + len(np.argwhere(frame5 == 255))> 70:
        offsetl=np.median([coordinate[0] for coordinate in np.argwhere(frame4 == 255)])
        offsetr=np.median([coordinate[0] for coordinate in np.argwhere(frame5 == 255)])

        if np.isnan(offsetr):
            offsetr=0
        if np.isnan(offsetl):
            offsetl=0
        print("left: ",offsetl," right:",offsetr)
        peroffset=(offsetl-offsetr)/150
        speed=speed/2


    # This code sets a hard limit for the turn angle. This is to reduce the stress on the wheel joints
    if peroffset>0.34:
        peroffset=0.34
    if peroffset<-0.34:
        peroffset=-0.34

    # This transforms the percentage into proper angle format
    angleset=90+(180*peroffset)

    if angleset <0:
        angleset=0

    if angleset > 180:
        angleset=180

    #angleset=180-angleset #THIS IS BANDAID

    # Sets the angle
    try:
        kit.servo[0].angle=angleset
    except:
        print("ERROR")


    if True:
        speed = (speed*(1-(1*abs(peroffset)))) #This is an option equation for slowing down in turns. We have to play around with the numbers.   
    if True:
        if speed < 0.15 and speed > 0:
            speed=0.15
        if speed > -0.15 and speed < 0:
            speed=-0.15

    # This checks if the car should break or drive normally
    try:
        kit.continuous_servo[1].throttle = speed # This sets the speed for the car. the range is 0 to 1. 0.15 is the slowest it can go in our tests.
        print(int(angleset), u1, u2, u3 ,u4 ,u5)
    except:
        print("ERROR")
    print(offsetl, "  ", offsetr)
    # Stops the program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        try:
            kit.servo[0].angle = 90
            kit.continuous_servo[1].throttle = 0
            break
        except:
            print("ERROR")

kit.continuous_servo[1].throttle = 0
kit.servo[0].angle = 90
cv2.destroyAllWindows()


