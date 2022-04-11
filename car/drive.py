# imports for driving
import cv2
import numpy as np
import time
from adafruit_servokit import ServoKit
from filters import ColorThreshholdFilter
import RPi.GPIO as GPIO
import subprocess
import os
import time

# imports for server
import argparse
from imutils.video import VideoStream
import socketio
import base64
from datetime import datetime
import json
import requests

# master setting variables
SERVO_CHANNEL = 0
MOTOR_CHANNEL = 1

# global lock variables for manual driving
LEFT_ARROW = False
RIGHT_ARROW = False
UP_ARROW = False
DOWN_ARROW = False

# server event code
# for debugging on, use
#sio = socketio.Client(logger=True, engineio_logger=True)
sio = socketio.Client()
output_frame = None
filtered_frame = None

carnumber_file = open("/etc/selfdriving-rc/carnumber", "r")
carnumber = carnumber_file.readline()
carnumber_file.close()

@sio.event(namespace='/cv')
def connect():
    print('[INFO] Connected to server.')

@sio.event(namespace='/cv')
def connect_error():
    print('[INFO] Failed to connect to server.')

@sio.event(namespace='/cv')
def disconnect():
    print('[INFO] Disconnected from server.')


class CVClient(object):
    def __init__(self, server_addr, lower_channels, higher_channels):
        self.stream = True
        self.direction = 1
        self.servo_direction = 1
        self.drive = False
        self.exit = False
        self.car_id = 'none'
        self.server_addr = server_addr
        self.lower_channels = lower_channels
        self.higher_channels = higher_channels
        # streamer var for testing servo angle
        self.test_angle = False
        self.test_throttle = False

    def setup(self):
        print('[INFO] Connecting to server http://{}...'.format(
            self.server_addr))
        sio.connect(
            'http://{}'.format(self.server_addr),
            transports=['websocket'],
            namespaces=['/cv', '/manual_drive', '/test_diagnostic'],
            headers={'carnumber': f"{carnumber}"})
        time.sleep(2.0)
        return self

    def _convert_image_to_jpeg(self, image):
        # masked = cv2.inRange(image, np.array(lower_channels), np.array(higher_channels))
        # encode the frame in JPEG format
        (flag, encodedImage) = cv2.imencode(".jpg", image)
        frame = encodedImage.tobytes()
        # Encode frame in base64 representation and remove
        # utf-8 encoding
        frame = base64.b64encode(frame).decode('utf-8')
        return "data:image/jpeg;base64,{}".format(frame)
        # yield the output frame in the byte format
        # return (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

    def send_video_feed(self, frame, route):
        sio.emit(
            route,
            {
                'carid': self.car_id,
                'image': self._convert_image_to_jpeg(frame)
            },
            namespace='/cv'
        )

    # Set the car's color channels
    def set_color_channels(self, x, y):
        global filtered_frame

        if filtered_frame is None:
            return "no output frame found"
        # x = 650 - x

        h = int(filtered_frame[y, x, 0])
        s = int(filtered_frame[y, x, 1])
        v = int(filtered_frame[y, x, 2])
        self.check_new_hsv(h, s, v)

    def check_new_hsv(self, h, s, v):
        if h < self.lower_channels[0]:
            self.lower_channels[0] = h
        if s < self.lower_channels[1]:
            self.lower_channels[1] = s
        if v < self.lower_channels[2]:
            self.lower_channels[2] = v
        if h > self.higher_channels[0]:
            self.higher_channels[0] = h
        if s > self.higher_channels[1]:
            self.higher_channels[1] = s
        if v > self.higher_channels[2]:
            self.higher_channels[2] = v


streamer = CVClient('ai-car.herokuapp.com', [255, 255, 255], [0, 0, 0])
@sio.on('carid2cv', namespace=f'/cv')
def set_car_id(carid):
    # make sure the car does not already have an id

    if streamer.car_id == 'none':
        # set the car id to the streamer
        print('setting car id to', carid)
        streamer.car_id = carid

        # write it to a file on the car
        f = open("/etc/selfdriving-rc/car_id.txt", "w")
        f.write(carid)
        f.close()

        # socket connection for coordinates to be sent to the car
        coordinates2cv_string = 'coordinates2cv/' + streamer.car_id
        @sio.on(coordinates2cv_string, namespace='/cv')
        def coordinates_to_hsv(message):
            json_data = json.loads(message)
            streamer.set_color_channels(json_data['x'], json_data['y'])
            color_channels = json.dumps({
                "carid": carid,
                "lower_channels": streamer.lower_channels,
                "higher_channels": streamer.higher_channels
            })
            sio.emit('channels2server', color_channels, namespace='/cv')

        # socket connection to reset the colors on the car
        resetcolors2cv_string = 'resetcolors2cv/' + streamer.car_id
        @sio.on(resetcolors2cv_string, namespace='/cv')
        def reset_color_channels():
            streamer.higher_channels = [0, 0, 0]
            streamer.lower_channels = [255, 255, 255]

        # socket on terminate driving
        terminate2cv_string = 'terminate2cv/' + streamer.car_id
        @sio.on(terminate2cv_string, namespace='/cv')
        def terminate():
            streamer.exit = True

        # socket to reset drive trigger
        stop2cv_string = 'stop2cv/' + streamer.car_id
        @sio.on(stop2cv_string, namespace='/cv')
        def stop_driving():
            streamer.drive = False

        # socket on start driving
        drive2cv_string = 'drive2cv/' + streamer.car_id
        @sio.on(drive2cv_string, namespace='/cv')
        def drive():
            streamer.drive = True

        # socket on toggle direction
        direction2cv_string = 'direction2cv/' + streamer.car_id
        @sio.on(direction2cv_string, namespace='/cv')
        def toggle_direction():
            streamer.direction = -1 * streamer.direction
            
        # socket on toggle servo direction
        servodirection2cv_string = 'servodirection2cv/' + streamer.car_id
        @sio.on(servodirection2cv_string, namespace='/cv')
        def toggle_servo_direction():
            streamer.servo_direction = -1 * streamer.servo_direction
            
        # sockets for enabling/disabling video
        disable2cv_string = 'disable2cv/' + streamer.car_id
        @sio.on(disable2cv_string, namespace='/cv')
        def disable_video():
            streamer.stream = False

        enable2cv_string = 'enable2cv/' + streamer.car_id
        @sio.on(enable2cv_string, namespace='/cv')
        def enable_video():
            streamer.stream = True

        # sockets for testing/diagnostics
        servo_angle_string = 'test/servo_angle/' + streamer.car_id
        @sio.on(servo_angle_string, namespace='/test_diagnostic')
        def test_servo_angle():
            print('test servo angle')
            # check if car is autonomously driving (check streamer.drive)
            # if not autonomously driving, set test variable to true
            if not streamer.drive:
                streamer.test_angle = True

        throttle_string = 'test/throttle/' + streamer.car_id
        @sio.on(throttle_string, namespace='/test_diagnostic')
        def test_throttle():
            print('test throttle')
            if not streamer.drive:
                streamer.test_throttle = True

        # sockets for manual driving
        turn_left_string = 'turn_left/' + streamer.car_id
        @sio.on(turn_left_string, namespace='/manual_drive')
        def manual_turn_left():
            global LEFT_ARROW, RIGHT_ARROW
            # if the right arrrow is held, we don't want to do anything
            if RIGHT_ARROW:
                return
            # otherwise if the left arrow isn't pressed
            if not LEFT_ARROW:
                LEFT_ARROW = True
                print("manual turn left")

        turn_left_stop_string = 'turn_left_stop/' + streamer.car_id
        @sio.on(turn_left_stop_string, namespace='/manual_drive')
        def manual_turn_left_stop():
            global LEFT_ARROW, RIGHT_ARROW
            # if the right arrow is held, we don't want to do anything
            if RIGHT_ARROW:
                return
            # otherwise release left arrow lock
            LEFT_ARROW = False
            print("turn left stopped")

        turn_right_string = 'turn_right/' + streamer.car_id
        @sio.on(turn_right_string, namespace='/manual_drive')
        def manual_turn_right():
            global LEFT_ARROW, RIGHT_ARROW
            # if the left arrow is held, we don't want to do anything
            if LEFT_ARROW:
                return
            # otherwise if the right arrow isn't pressed
            if not RIGHT_ARROW:
                RIGHT_ARROW = True
                print("manual turn right")

        turn_right_stop_string = 'turn_right_stop/' + streamer.car_id
        @sio.on(turn_right_stop_string, namespace='/manual_drive')
        def manual_turn_right_stop():
            global LEFT_ARROW, RIGHT_ARROW
            # if the left arrow is held, we don't want to do anything
            if LEFT_ARROW:
                return
            # otherwise release right arrow lock
            RIGHT_ARROW = False
            print("turn right stopped")

        throttle_up_string = 'throttle_up/' + streamer.car_id
        @sio.on(throttle_up_string, namespace='/manual_drive')
        def manual_throttle_up():
            global UP_ARROW, DOWN_ARROW
            # if the down arrow is held, we don't want to do anything
            if DOWN_ARROW:
                return
            # otherwise if the up arrow isn't pressed
            if not UP_ARROW:
                UP_ARROW = True
                print("manual throttle up")

        throttle_up_stop_string = 'throttle_up_stop/' + streamer.car_id
        @sio.on(throttle_up_stop_string, namespace='/manual_drive')
        def manual_throttle_up_stop():
            global UP_ARROW, DOWN_ARROW
            # if the down arrow is held, we don't want to do anything
            if DOWN_ARROW:
                return
            # otherwise release up arrow lock
            UP_ARROW = False
            print("throttle up stopped")

        throttle_back_string = 'throttle_back/' + streamer.car_id
        @sio.on(throttle_back_string, namespace='/manual_drive')
        def manual_throttle_back():
            global UP_ARROW, DOWN_ARROW
            # if the up arrow is held, we don't want to do anything
            if UP_ARROW:
                return
            # otherwise if the down arrow isn't held
            if not DOWN_ARROW:
                DOWN_ARROW = True
                print("manual throttle back")

        throttle_back_stop_string = 'throttle_back_stop/' + streamer.car_id
        @sio.on(throttle_back_stop_string, namespace='/manual_drive')
        def manual_throttle_back_stop():
            global UP_ARROW, DOWN_ARROW
            # if the up arow is held, we don't want to do anything
            if UP_ARROW:
                return
            # otherwise release down arrow lock
            DOWN_ARROW = False
            print("throttle back stopped")

    else:
        print('car\'s id is already', streamer.car_id)


def main(server_addr, speed, steering, lower_channels, higher_channels):
    global streamer, output_frame, filtered_frame, SERVO_CHANNEL, MOTOR_CHANNEL, servo_adjustment
    # vs = VideoStream(src=0).start()
    servo_adjustment = 0
    cap = cv2.VideoCapture(-1)
    last_time = datetime.now()
    time.sleep(2.0)

    streamer = CVClient(server_addr, lower_channels, higher_channels)
    streamer.setup()
    sio.sleep(2.0)

    colorThreshholdFilter = ColorThreshholdFilter()
    kit = ServoKit(channels=16)  # Initializes the servo shield
    kit.servo[SERVO_CHANNEL].angle = 90 + servo_adjustment # Sets wheels forward
    kit.continuous_servo[MOTOR_CHANNEL].throttle = 0  # Sets speed to zero
    scale = 75
    max_speed = 1

    last_update_time = time.time()

    inc = 1
    while True:
        if streamer.exit:
            break

        _, frame = cap.read()

        this_time = datetime.now()
        if streamer.stream:
            frame_copy = frame.copy()
            web_width = int(frame.shape[1] * scale / 100)
            web_height = int(frame.shape[0] * scale / 100)
            dim = (web_width, web_height)
            output_frame = cv2.resize(frame_copy, dim, interpolation=cv2.INTER_AREA)

            filtered_frame = cv2.cvtColor(output_frame, cv2.COLOR_BGR2HSV)
            masked = cv2.inRange(filtered_frame, np.array(streamer.lower_channels), np.array(streamer.higher_channels))

            time_difference = this_time - last_time
            if time_difference.total_seconds() >= 0.3:
                streamer.send_video_feed(output_frame, 'cvimage2server')
                streamer.send_video_feed(masked, 'cvfiltered2server')
                last_time = this_time

        # only update every 3 seconds
        current_time = time.time()
        if current_time - last_update_time > 3:
            # speed request
            speed_r = requests.get("http://{}/api/car/{}/get/speed".format(
                streamer.server_addr, streamer.car_id
            ))
            speed = int(speed_r.text.replace('"', ''))
            print("speed: {}".format(speed))

            # steering request
            steering_r = requests.get("http://{}/api/car/{}/get/steering".format(
                streamer.server_addr, streamer.car_id
            ))
            steering = int(steering_r.text.replace('"', ''))
            print("steering: {}".format(steering))

            # servo adjustment request
            servo_adjustment_r = requests.get("http://{}/api/car/{}/get/servo_adjustment".format(
                streamer.server_addr, streamer.car_id
            ))
            servo_adjustment = int(servo_adjustment_r.text.replace('"', ''))
            print("servo adjustment: {}".format(servo_adjustment))

            # servo channel request
            servochannel_r = requests.get("http://{}/api/car/{}/get/servo_channel".format(
                streamer.server_addr, streamer.car_id
            ))
            SERVO_CHANNEL = int(servochannel_r.text.replace('"', ''))
            print("servo channel: {}".format(SERVO_CHANNEL))

            # esc channel request
            motorchannel_r = requests.get("http://{}/api/car/{}/get/esc_channel".format(
                streamer.server_addr, streamer.car_id
            ))
            MOTOR_CHANNEL = int(motorchannel_r.text.replace('"', ''))
            print("motor channel: {}".format(MOTOR_CHANNEL))

            # set last update time to now
            last_update_time = time.time()

        # if test program is running
        # put test logic here
        # end if statement with 'continue'
        if streamer.test_angle:
            # test servo angle logic here:
            # enable the servo and motors
            print("enabling servo and setting to 90")
            kit.servo[SERVO_CHANNEL].angle = 90 + servo_adjustment  # Sets wheels forward
            kit.continuous_servo[MOTOR_CHANNEL].throttle = 0  # Sets speed to zero
            print("sleeping for 3 seconds...")
            time.sleep(3)
            print("setting servo to 140")
            kit.servo[SERVO_CHANNEL].angle = 140 + servo_adjustment
            print("sleeping for 3 seconds...")
            time.sleep(3)
            print("setting servo to 40")
            kit.servo[SERVO_CHANNEL].angle = 40 + servo_adjustment
            print("sleeping for 3 seconds...")
            time.sleep(3)
            print("resetting servo back to 90")
            kit.servo[SERVO_CHANNEL].angle = 90 + servo_adjustment
            time.sleep(2)
            print("End of front servo angle test")
            streamer.test_angle = False
            continue

        if streamer.test_throttle:
            # test throttle logic here:
            # enable the servo and motors
            print("enabling servo and setting throttle to 0")
            kit.servo[SERVO_CHANNEL].angle = 90 + servo_adjustment  # Sets wheels forward
            kit.continuous_servo[MOTOR_CHANNEL].throttle = 0  # Sets speed to zero
            print("sleeping for 2 seconds...")
            time.sleep(2)
            print("setting servo throttle to 15%")
            kit.continuous_servo[MOTOR_CHANNEL].throttle = 0.15
            print("running for 2 seconds...")
            time.sleep(2)
            print("setting servo throttle to 0")
            kit.continuous_servo[MOTOR_CHANNEL].throttle = 0
            time.sleep(1)
            print("End of throttle test")
            streamer.test_throttle = False
            continue

        if not streamer.drive:
            # code to manual drive
            manual_driving = False
            # if up arrow is depressed
            if UP_ARROW:
                # print("forward!")
                kit.continuous_servo[MOTOR_CHANNEL].throttle = streamer.direction * (max_speed*(speed/100))
                manual_driving = True

            # if down arrow is depressed
            if DOWN_ARROW:
                # print("backwards!")
                kit.continuous_servo[MOTOR_CHANNEL].throttle = -(streamer.direction * (max_speed*(speed/100)))
                manual_driving = True

            # if left arrow is depressed
            if LEFT_ARROW:
                if streamer.servo_direction == 1:
                    # print("turn left!")
                    kit.servo[SERVO_CHANNEL].angle = 150 + servo_adjustment
                    manual_driving = True
                if streamer.servo_direction == -1:
                    # print("turn left!")
                    kit.servo[SERVO_CHANNEL].angle = 30 + servo_adjustment
                    manual_driving = True

            # if right arrow is depressed
            if RIGHT_ARROW:
                if streamer.servo_direction == 1:
                    # print("turn right!")
                    kit.servo[SERVO_CHANNEL].angle = 30 + servo_adjustment
                    manual_driving = True
                if streamer.servo_direction == -1:
                    # print("turn left!")
                    kit.servo[SERVO_CHANNEL].angle = 150 + servo_adjustment
                    manual_driving = True

            # if no forward or backward
            if not UP_ARROW and not DOWN_ARROW:
                # print("stop!")
                kit.continuous_servo[MOTOR_CHANNEL].throttle = 0

            # if no left or right
            if not LEFT_ARROW and not RIGHT_ARROW:
                # print("no turning!")
                kit.servo[SERVO_CHANNEL].angle = 90 + servo_adjustment

            if not manual_driving:
                kit.continuous_servo[MOTOR_CHANNEL].throttle = 0
                kit.servo[SERVO_CHANNEL].angle = 90 + servo_adjustment
            continue

        height, width, channels = frame.shape
#        print("height: {}, width: {}".format(height, width))
        middle = width / 2
        uph=int(height/2.2) #2.2 original, 1.65 test
        downh=int(height/1.7) #1.7 original, 1.15 test
        frame1init = frame[uph:downh, 0:int(int(width) / 3)]
        frame2init = frame[uph:downh, int(2 * int(width) / 3):int(width)]

        frame1 = colorThreshholdFilter.apply(frame1init, streamer.lower_channels, streamer.higher_channels)
        frame2 = colorThreshholdFilter.apply(frame2init, streamer.lower_channels, streamer.higher_channels)

        leftlane = np.mean([coordinate[1] for coordinate in np.argwhere(frame1 == 255)])
        rightlane = (2 * int(width) / 3) + np.mean([coordinate[1] for coordinate in np.argwhere(frame2 == 255)])

        offsetl = (middle - leftlane)
        offsetr = (rightlane - middle)

        if np.isnan(offsetr):
            offsetr = middle
        if np.isnan(offsetl):
            offsetl = middle

        offset = offsetr - offsetl
        peroffset = offset / width

        if peroffset > 0.33:
            peroffset = 0.33
        if peroffset < -0.33:
            peroffset = -0.33

        angleset = 90 + (180 * -peroffset * (steering/100))

        if angleset < 40:
            angleset = 40
        if angleset > 150:
            angleset = 150
            
        
        if streamer.servo_direction == -1:
            kit.servo[SERVO_CHANNEL].angle = (180 - angleset) + servo_adjustment   
        else:
            kit.servo[SERVO_CHANNEL].angle = angleset + servo_adjustment
        #print("servo angle: ", kit.servo[SERVO_CHANNEL].angle)

        kit.continuous_servo[MOTOR_CHANNEL].throttle = streamer.direction * (max_speed*(speed/100))  # This sets the speed for the car. the range is 0 to 1. 0.15 is the slowest it can go in our tests.
        #print("throttle: ", kit.continuous_servo[MOTOR_CHANNEL].throttle)

    kit.continuous_servo[MOTOR_CHANNEL].throttle = 0
    kit.servo[SERVO_CHANNEL].angle = 90 + servo_adjustment
    sio.disconnect()
    subprocess.run("sleep 10; sudo shutdown -h now", shell=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='MQP Dashboard Video Streamer')
    parser.add_argument(
            '--server-addr',  type=str, default='ai-car.herokuapp.com',
            help='The IP address or hostname of the SocketIO server.')
    parser.add_argument("--speed", help="Car Speed", default=0)
    parser.add_argument("--steering", help="Car Steering", default=100)
    parser.add_argument("--lowerArr", help="Lower Color Channel", default=[255, 255, 255])
    parser.add_argument("--higherArr", help="Higher Color Channel", default=[0, 0, 0])
    args = parser.parse_args()
    main(args.server_addr, args.speed, args.steering, args.lowerArr, args.higherArr)
    
