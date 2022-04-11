#!/bin/bash

sleep 30

SERVER_URL="https://ai-car.herokuapp.com"
FLASK_DEBUG_IP="127.0.0.1:5000"
CAR_ID_LOCATION="/etc/selfdriving-rc/car_id.txt"
CAR_SPEED_LOCATION="/etc/selfdriving-rc/car_speed"
CAR_STEERING_LOCATION="/etc/selfdriving-rc/car_steering"
CAR_SERVO_ADJUSTMENT_LOCATION="/etc/selfdriving-rc/car_servo_adjustment"
CAR_SENSOR_DATA_LOCATION="/etc/selfdriving-rc/sensor_data"
CAR_SERVO_CHANNEL_LOCATION="/etc/selfdriving-rc/servo_channel"
CAR_ESC_CHANNEL_LOCATION="/etc/selfdriving-rc/esc_channel"
CAR_DRIVING_PROGRAM="/home/pi/Documents/car/drive.py"
SERVER_PROGRAM="/home/pi/Documents/dashboard/app.py"
SENSOR_PROGRAM="/home/pi/Documents/car/sensors.py"
PI_HOSTNAME=$(hostname -I | xargs)
SERVER_URL="$PI_HOSTNAME":5000

RUNNING_PROCESSES=`lsof -i -P -n | grep $FLASK_DEBUG_IP`

if [ -z $RUNNING_PROCESSES ]; then
echo "No flask debug server found. Connecting to remote server at '$SERVER_URL'."
else
echo "Flask debug server detected. Connecting to '$FLASK_DEBUG_IP'."
# SERVER_URL="$FLASK_DEBUG_IP"
fi

echo "Resetting config files and removing any duplicate instances of '$CAR_DRIVING_PROGRAM'"
PROGRAM_FILENAME=`basename $CAR_DRIVING_PROGRAM`
pkill -f $PROGRAM_FILENAME
echo "" > $CAR_ID_LOCATION
echo 0 > $CAR_SPEED_LOCATION
#echo "" > $CAR_SENSOR_DATA_LOCATION

sleep 5

#stdbuf -o0 cat /dev/ttyACM0 >$CAR_SENSOR_DATA_LOCATION &

echo "Starting local server"
nohup python3 $SERVER_PROGRAM > /home/pi/Documents/dashboard/logs/server &
SERVER_SCRIPT_PID=$!

sleep 5

echo "Starting selfdriving program in child process at $SERVER_URL"
nohup python3 $CAR_DRIVING_PROGRAM --server-addr 0.0.0.0:5000 > /home/pi/Documents/car/logs/drive &
DRIVING_SCRIPT_PID=$!

sleep 5

echo "Starting sensor communication program in child process"
nohup python3 $SENSOR_PROGRAM > /home/pi/Documents/car/logs/sensors &
SENSOR_SCRIPT_PID=$!

# To be called in order to properly terminate child process
terminate_startup () {
    kill $SERVER_SCRIPT_PID
    kill $DRIVING_SCRIPT_PID
    kill $SENSOR_SCRIPT_PID
    exit $1
}

trap terminate_startup 0 EXIT

# Delay to ensure driving code has recieved an ID and updated the /car_id file
sleep 10

echo "Reading car ID from '$CAR_ID_LOCATION'."
CAR_ID=`cat $CAR_ID_LOCATION`
if [ ${#CAR_ID} -eq 36 ]; then
echo "Car is enrolled with the id '$CAR_ID'."
else
echo "ID was malformed. Please ensure '$CAR_DRIVING_PROGRAM' is properly \
setting the file at '$CAR_ID_LOCATION'."
#terminate_startup 0
fi

while :
do
    echo "Components running..."
    sleep 3
done
