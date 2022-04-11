
import cv2
import numpy as np
from filters import ColorThreshholdFilter, FrameMaskFilter, LineDetectionFilter
import time
from adafruit_servokit import ServoKit
import serial
import RPi.GPIO as GPIO

us1calib=[206.0, None, None, None, None, None, None, None, 22.5, None, None, None, None, 6.0, 24.0, None, 40.0, None, None, None, None, None, 77.0, 64.0, None, 90.0, 27.0, None, 101.0, None, 26.0, 120.0, 38.5, 45.0, None, 43.0, None, 46.0, None, 56.0, None, None, None, None, 57.0, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, 200.0, 173.0, 194.0, None, 165.0, 160.0, 167.5, 147.0, None, None, 148.0, 135.0]
us2calib=[196.0, None, 180.5, 185.0, 172.0, None, None, None, None, None, 116.0, 122.0, 120.0, 103.0, None, None, None, None, None, None, None, None, None, None, 66.0, 110.0, None, 76.0, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, 9.0, 31.0, 60.0, 52.0, 69.0, 64.0, 173.0, None, 82.0, None, None, None, None, 200.0, None, 176.0, 188.0, 98.0, 110.0, 1.0, 151.0, 120.0, 125.0, 157.0, 176.0, 140.0, 110.0, 165.0, 141.0, 160.0, 148.0, 133.0, 135.0, 143.0, 176.0]
us3calib=[0.0, None, 116.0, 126.0, None, None, None, None, 29.0, 35.0, 54.0, None, 161.0, None, None, 126.0, 124.0, 133.0, 133.0, 14.0, 137.5, None, 176.0, 174.0, 142.0, 126.0, 168.0, None, 165.0, 126.0, None, 125.5, 125.0, 125.0, 129.0, None, None, None, None, None, None, None, None, None, None, 87.0, 69.0, 91.0, 110.0, 48.0, 122.0, 27.0, 143.0, None, None, 165.0, 187.0, 45.0, 125.0, 157.0, None, 151.0, 98.0, 14.0, 135.0, 188.0, 154.0, 200.0, 176.0, None, 195.5, None, 157.0, 197.0, 194.0, 182.0, 33.0, 203.5, 1.0, 9.0]

us4calib=[0.0, None, 62.0, 90.0, None, None, None, 134.0, 146.0, 89.0, 9.5, None, None, None, None, None, None, None, None, None, None, None, None, None, None, 8.0, None, 14.5, 187.0, None, None, None, None, None, None, 14.0, None, None, None, None, None, None, None, None, 124.0, 129.0, 138.0, None, 33.0, None, None, None, None, None, None, None, None, None, None, None, None, 148.0, 167.0, 143.0, 102.0, 121.0, None, None, None, None, None, 188.0, 2.0, 187.0, 111.0, 48.0, 124.5, 162.0, 124.0, 141.0]
us5calib=[0.0, None, None, None, None, None, None, 83.0, None, 88.0, None, 110.0, None, 58.0, 70.0, 90.0, None, None, 95.0, 66.5, 142.0, 162.0, None, 64.5, 134.0, None, None, None, None, None, None, None, None, 34.0, 42.0, 32.0, None, 27.0, 27.0, 33.0, None, None, None, 52.0, None, None, 18.5, None, None, None, None, None, None, None, None, None, 0.0, None, 48.5, None, 50.0, None, 46.0, None, None, None, None, 39.0, 122.0, None, None, None, None, None, None, None, 130.0, None, None, 122.0]
