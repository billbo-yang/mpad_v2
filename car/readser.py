import serial
import time
import requests
import subprocess

ser = serial.Serial('/dev/ttyACM0', 9600)
etc_file = open('/etc/selfdriving-rc/serial-out', 'w')

while True:
    reading = ser.readline()
    try:
        reading = str(reading, 'utf-8')
    except UnicodeDecodeError as e:
        print("UE error")
        continue
    except Exception as e:
        print("General exception")
        continue
    reading = reading.rstrip('\r\n')
    etc_file.seek(0)
    etc_file.write(reading)
    etc_file.truncate()

etc_file.close()
ser.close()