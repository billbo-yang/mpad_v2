#!/bin/bash

sudo apt-get install -y build-essential python3 python3-dev \
python3-pip python3-virtualenv python3-numpy python3-picamera \
python3-pandas python3-rpi.gpio i2c-tools avahi-utils joystick \
libopenjp2-7-dev libtiff5-dev gfortran libatlas-base-dev libopenblas-dev \
libhdf5-serial-dev git ntp libilmbase-dev libopenexr-dev libgstreamer1.0-dev \
libjasper-dev libwebp-dev libatlas-base-dev python3-opencv

sudo apt-get install -y python3-opencv
sudo apt-get install -y libhdf5-dev
sudo apt-get install -y libhdf5-serial-dev
sudo apt-get install -y libatlas-base-dev
sudo apt-get install -y libjasper-dev 
sudo apt-get install -y libqtgui4 
sudo apt-get install -y libqt4-test
sudo apt-get install -y fswebcam

pip3 install -r requirements.txt

pip3 install opencv-python
pip3 install imutils
pip3 install python-socketio
pip3 install -U numpy

