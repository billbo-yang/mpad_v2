# imports for driving
import cv2
import numpy as np
import time
from adafruit_servokit import ServoKit
from filters import ColorThreshholdFilter
import RPi.GPIO as GPIO
import subprocess

# master setting variables
SERVO_CHANNEL = 0
MOTOR_CHANNEL = 1

# enable the servo and motors
print("enabling servo and setting to 90")
kit = ServoKit(channels=16)  # Initializes the servo shield
kit.servo[SERVO_CHANNEL].angle = 90  # Sets wheels forward
kit.continuous_servo[MOTOR_CHANNEL].throttle = 0  # Sets speed to zero

print("sleeping for 3 seconds...")
time.sleep(3)

print("setting servo to 140")
kit.servo[SERVO_CHANNEL].angle = 140

print("sleeping for 3 seconds...")
time.sleep(3)

print("setting servo to 40")
kit.servo[SERVO_CHANNEL].angle = 40

print("sleeping for 3 seconds...")
time.sleep(3)

print("resetting servo back to 90")
kit.servo[SERVO_CHANNEL].angle = 90
