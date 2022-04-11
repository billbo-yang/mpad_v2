import cv2
import numpy as np
from filters import ColorThreshholdFilter, FrameMaskFilter, LineDetectionFilter
import time
from adafruit_servokit import ServoKit
import serial
import RPi.GPIO as GPIO

ser=serial.Serial("/dev/ttyACM0",9600)  #change ACM number as found from ls /dev/tty/ACM*
ser.baudrate=9600


cap = cv2.VideoCapture(0)

colorThreshholdFilter = ColorThreshholdFilter()

kit = ServoKit(channels=16)
kit.servo[0].angle = 90
kit.continuous_servo[1].throttle = 0


def empty(value):
    pass

result_window_name = "Final Output"
cv2.namedWindow(result_window_name)

# u1 u2 are bumper left
# u3 is bumper middle
# u4 u5 bumper right
# u6 is car left
# u7 is car right

# These are calibration numbers, it is the output of another program. The values in the array represent the minimum and maximum x coordinates of the 
# pixels in the camera input of every reading from the ultrasonic. These numbers should be updated whenever the module is installed on another car.
# The following numbers are just examples. We will recalibrate once we get a newly 3d printed car.
'''
us1:
min 0
max 2.250000000000001 69.33333333333329

us2:
min 0.6114285714285695 155.60000000000008
max 1.9999999999999962 181.33333333333354

us3:
min -0.45245901639344077 244.3442622950819
max 0.1245901639344264 320.55737704918033

us4:
min -2.7716894977168978 403.2237442922375
max -1.0155251141552546 457.81278538812813

us5:
min -0.3999999999999936 463.9999999999999
max 1.5500000000000158 510.99999999999943 #or max
'''



read_ser=str(ser.readline())
time.sleep(1)
while(True):
    t0=time.time()


    read_ser=str(ser.readline())
    print(read_ser)

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
    read_ser=read_ser[slice(read_ser.find('.')+1,len(read_ser))]


    # The timeout in the arduino code sets the ultrasonic to 0 meaning that the true value of a 0 reading is undetected. 200 is a good value for the calculations.
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


    # Take each frame
    _, frame = cap.read()
    height, width, channels = frame.shape
    middle = width / 2

    frame1 = frame[0:height, 0:int(width)]

    us1min=[0,0]
    us1max=[2.25,69.4]

    us2min=[0.61142,155.6]
    us2max=[1,99999,181.4]

    us3min=[-0.4524,244.344]
    us3max=[0.12459,320.5573]

    us4min=[-2.7716,403.2237]
    us4max=[-1.10155,457.81]

    us5min=[-0.3999999,463.99]
    us5max=[0,width]




    if u1<30:
        cv2.rectangle(frame,(int(us1min[0]*u1+us1min[1]),height),(int(us1max[0]*u1+us1max[1]),0),(255, 0, 0),3)
    if u2<60:
        cv2.rectangle(frame,(int(us2min[0]*u2+us2min[1]),height),(int(us2max[0]*u2+us2max[1]),0),(255, 0, 0),3)
    if u3<60:
        cv2.rectangle(frame,(int(us3min[0]*u3+us3min[1]),height),(int(us3max[0]*u3+us3max[1]),0),(255, 0, 0),3)
    if u4<60:
        cv2.rectangle(frame,(int(us4min[0]*u4+us4min[1]),height),(int(us4max[0]*u4+us4max[1]),0),(255, 0, 0),3)
    if u5<30:
        cv2.rectangle(frame,(int(us5min[0]*u5+us5min[1]),height),(int(us5max[0]*u5+us5max[1]),0),(255, 0, 0),3)

    cv2.imshow("yo", frame1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        try:
            kit.servo[0].angle = 90
            kit.continuous_servo[1].throttle = 0
            break
        except:
            print("ERROR")



