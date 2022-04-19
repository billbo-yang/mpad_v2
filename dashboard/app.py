from flask_socketio import SocketIO
from flask import Flask, render_template, Response, jsonify, request, abort
from random import randint
import json
import uuid
from redisConn import RedisConn
import time

app = Flask(__name__)
socketio = SocketIO(app, ping_timeout=30)
redis = RedisConn()

# Initialize the cars in the set
initial_cars = {}
redis.set_car_json('cars', json.dumps(initial_cars))

# Video Socket logging, one namespace for computer vision client and one for the web client
@socketio.on('connect', namespace='/web')
def connect_web():
    print('[INFO] Web client connected: {}'.format(request.sid))


@socketio.on('disconnect', namespace='/web')
def disconnect_web():
    print('[INFO] Web client disconnected: {}'.format(request.sid))


@socketio.on('connect', namespace='/cv')
def connect_cv():
    print(f'Trying to get the arguments: {request.headers}')
    enrollCar(request.sid, request.headers.get('carnumber'))

@socketio.on('disconnect', namespace='/cv')
def disconnect_cv():
    redis.remove_car(request.sid)
    print('[INFO] CV client disconnected: {}'.format(request.sid))


@socketio.on('channels2server', namespace='/cv')
def color_channels_to_redis(message):
    json_data = json.loads(message)
    car_json = redis.get_car_json(json_data['carid'])
    car_json['lower_channels'] = json_data['lower_channels']
    car_json['higher_channels'] = json_data['higher_channels']
    setCar(json_data['carid'], car_json)


# Video Socket message handlers
@socketio.on('cvimage2server', namespace='/cv')
def handle_cv_message(message):
    image2web_string = 'image2web/' + message['carid']
    socketio.emit(image2web_string, message, namespace='/web')


@socketio.on('cvfiltered2server', namespace='/cv')
def handle_filtered_cv_message(message):
    filtered2web_string = 'filtered2web/' + message['carid']
    socketio.emit(filtered2web_string, message, namespace='/web')

def parse_reading(sensor_string):
    sensor_dict = {}
    sensor_string = sensor_string.strip()
    sensor_string = sensor_string.replace("||", "|")
    sensor_string = sensor_string.lstrip("|")
    sensor_string = sensor_string.rstrip("|")
    readings = sensor_string.split("|")
    for reading in readings:
        reading_kv = reading.split(":")
        sensor_dict[reading_kv[0]] = reading_kv[1]
    
    return sensor_dict

def getCar(car_id):
    car_json = redis.get_car_json(car_id)
    if car_json is None:
        abort(404, description=(f"Your request for car '{car_id}' could not be processed as the car doesn't exist."))
    else:
        return car_json

def setCar(car_id, car_data):
    car_json = json.dumps(car_data)
    redis.set_car_json(car_id, car_json)

def enrollCar(sid, car_number=-1):
    # generate a unique id for the car
    car_id = str(uuid.uuid4())

    # link the socket id with the car id to maintain it on disconnect
    redis.link_ids(sid, car_id)

    # create the initial car configs on startup
    initial_configs = {
        "speed": 0,
        "steering": 100,
        "servo_adjustment": 0,
        "direction": 1,
        "servo_direction": 1,
        "servo_channel": 0,
        "esc_channel": 1,
        "video_stream": True,
        "lower_channels": [255, 255, 255],
        "higher_channels": [0, 0, 0],
        "timestamp": [],
        "temperature_data": [],
        "humidity_data": [],
        "imu_data": [],
        "ultrasonic_data": [],
        "hall_effect_data": [],
        "battery_data": [],
        "lipo_data": []
    }

    # write the car to redis
    setCar(car_id, initial_configs)

    # add the car to the cars list (with a friendly display name)
    cars = redis.get_car_json('cars')
    cars[car_id] = getFriendlyCarName() + f" (Team {car_number})"
    print(f"[TARGET] team {car_number} was enrolled")
    redis.set_car_json('cars', json.dumps(cars))

    # send the carid to the car
    socketio.emit('carid2cv', car_id, namespace='/cv')
    return jsonify({"id": car_id}), 200


@app.errorhandler(404)
def resource_not_found(e):
    return jsonify(error=str(e)), 404

@app.route('/')
def selectCar():
    cars = redis.get_car_json('cars')
    return render_template("landing.html", cars=cars), 200

@app.route('/dashboard/<car_id>')
def carDashboard(car_id):
    cars = redis.get_car_json('cars')
    friendly_name = cars[car_id]
    if (car_id not in cars):
        return "That car can't be found. Go back to the dashboard to see currently online cars.", 404
    return render_template("dashboard.html", carid=car_id, friendly_name=friendly_name), 200

@app.route('/dashboard/<car_id>/colorselector')
def colorSelector(car_id):
    cars = redis.get_car_json('cars')
    friendly_name = cars[car_id]
    if (car_id not in cars):
        return "That car can't be found. Go back to the dashboard to see currently online cars.", 404
    return render_template("colorselector.html", carid=car_id, friendly_name=friendly_name), 200

@app.route('/api/client/<car_id>/control')
def controlCar(car_id):
    cars = redis.get_car_json('cars')
    car = cars[car_id]
    car.isDriving = request.args.get('driving')
    return '200 OK', 200

@app.route('/api/car/<car_id>/control', methods=['POST', 'GET'])
def carControl(car_id):
    car = getCar(car_id)
    if request.method == 'POST':
        accepted_args = ['throttle_speed', 'algorithm', 'algorithm_mode', 'higher_channels', 'lower_channels']
        for argument in request.args:
            print(f"Attempting to match the argument named '{argument}'.")
            if argument in accepted_args:
                print(f"Setting the value '{argument}' to '{request.args.get(argument)}'.")
                car[argument] = request.args.get(argument)
        setCar(car_id, car)
        return '200 OK', 200
    if request.method == 'GET':
        return jsonify(car), 200

@app.route('/api/car/<car_id>/data', methods=['POST', 'GET'])
def car_data(car_id):
    car_json = getCar(car_id)
    if request.method == 'POST':
        sensor_string = request.get_json()['sensor_string']
        if sensor_string == "":
            return '400 BAD REQUEST', 400
        partial_readings = parse_reading(sensor_string)
        print(f"Sensor string: {sensor_string}")
        full_readings = redis.sanitize_sensor_reading(car_id, partial_readings)
        new_readings = json.loads(redis.store_sensor_readingtimestamps(car_id, car_json, full_readings))
        data2web_string = 'data2web/' + car_id
        socketio.emit(data2web_string, json.dumps(new_readings), namespace='/web')
        return '200 OK', 200
    if request.method == 'GET':
        data = redis.read_data(car_json)
        return jsonify(data), 200

def getFriendlyCarName():
    descriptors = ['Rusty', 'Shiny', 'Squeaky', 'Leaky', 'New', 'Speedy', 'Sluggish', 'Electric', 'Hybrid', 'Classic', 'Sporty', 'Used']
    colors = ['Red', 'Green', 'Turquoise', 'Yellow', 'Blue', 'Bergundy', 'Gray', 'White', 'Black', 'Hot-pink', 'Tan']
    cars = ['Jeep', '4x4', 'Dually', 'Horse-Drawn Carriage', 'Tricycle', 'Hatchback', 'Van', 'Sedan', 'Civic', 'Bus', 'Tractor']

    descriptor = descriptors[randint(0, len(descriptors) - 1)]
    color = colors[randint(0, len(colors) - 1)]
    car = cars[randint(0, len(cars) - 1)]

    if randint(1, 100) == 66:
        ee_names = ['Binky\'s Bus', 'Gordon\'s Granola Gondola', 'Pradeep\'s Jeep', 'Raj\'s Boof Bus', 'Eric\'s Elf E-Bike', 'Cricky\'s Chili Dog Cart', 'Stultz\'s Stinky Surfboard', 'Dexter\'s Devious Dumptruck', 'Taylan\'s Tipsy Trailer', 'Enzo\'s Electric Elephant']
        return ee_names[randint(0, len(ee_names) - 1)]

    return f"{descriptor} {color} {car}"

@app.route('/api/client/<car_id>/export/data')
def export_sensor_data(car_id):
    car_json = getCar(car_id)
    return jsonify({
        "timestamp": car_json['timestamp'],
        "hall_effect": car_json['hall_effect_data'],
        "battery": car_json['battery_data'],
        "temperature": car_json['temperature_data'],
        "humidity": car_json['humidity_data'],
        "imu": car_json['imu_data'],
        "ultrasonic": car_json['ultrasonic_data'],
        "lipo": car_json['lipo_data']
    })

@app.route('/api/car/<car_id>/set/speed/<speed>', methods=['POST'])
def set_speed(car_id, speed):
    car_json = redis.get_car_json(car_id)
    car_json['speed'] = speed
    redis.set_car_json(car_id, json.dumps(car_json))
    return '200 OK', 200

@app.route('/api/car/<car_id>/get/speed')
def get_speed(car_id):
    car_json = redis.get_car_json(car_id)
    return json.dumps(car_json['speed'])

@app.route('/api/car/<car_id>/set/steering/<steering>', methods=['POST'])
def set_steering(car_id, steering):
    car_json = redis.get_car_json(car_id)
    car_json['steering'] = steering
    redis.set_car_json(car_id, json.dumps(car_json))
    return '200 OK', 200

@app.route('/api/car/<car_id>/get/servo_adjustment')
def get_servo_adjustment(car_id):
    car_json = redis.get_car_json(car_id)
    return json.dumps(car_json['servo_adjustment'])

@app.route('/api/car/<car_id>/set/servo_adjustment/<servo_adjustment>', methods=['POST'])
def set_servo_adjustment(car_id, servo_adjustment):
    car_json = redis.get_car_json(car_id)
    car_json['servo_adjustment'] = servo_adjustment
    redis.set_car_json(car_id, json.dumps(car_json))
    return '200 OK', 200

@app.route('/api/car/<car_id>/get/steering')
def get_steering(car_id):
    car_json = redis.get_car_json(car_id)
    return json.dumps(car_json['steering'])

@app.route('/api/car/<car_id>/send/coordinates', methods=['POST'])
def send_coordinates(car_id):
    if request.method == 'POST':
        coordinates = request.get_json()
        coordinates2cv_string = 'coordinates2cv/' + car_id
        socketio.emit(coordinates2cv_string, json.dumps(coordinates), namespace='/cv')
        return '200 OK', 200

@app.route('/api/car/<car_id>/reset/color')
def reset_color_channels(car_id):
    car_json = redis.get_car_json(car_id)
    car_json['higher_channels'] = [0, 0, 0]
    car_json['lower_channels'] = [255, 255, 255]
    redis.set_car_json(car_id, json.dumps(car_json))
    resetcolors2cv_string = 'resetcolors2cv/' + car_id
    socketio.emit(resetcolors2cv_string, namespace='/cv')
    return '200 OK', 200

@app.route('/api/car/<car_id>/terminate')
def terminate_program(car_id):
    terminate2cv_string = 'terminate2cv/' + car_id
    socketio.emit(terminate2cv_string, namespace='/cv')
    return '200 OK', 200

@app.route('/api/car/<car_id>/stop/drive')
def stop_driving(car_id):
    stop2cv_string = 'stop2cv/' + car_id
    socketio.emit(stop2cv_string, namespace='/cv')
    return '200 OK', 200

@app.route('/api/car/<car_id>/drive')
def begin_driving(car_id):
    drive2cv_string = 'drive2cv/' + car_id
    socketio.emit(drive2cv_string, namespace='/cv')
    car_json = redis.get_car_json(car_id)
    return jsonify({
        "speed": car_json['speed'],
        "steering": car_json['steering'],
        "lower_channels": car_json['lower_channels'],
        "higher_channels": car_json['higher_channels']
    })

# GET request handlers for manual driving
@app.route('/api/car/<car_id>/drive/turn/left')
def turn_left(car_id):
    print("manual turn left")
    turn_left_string = 'turn_left/' + car_id
    socketio.emit(turn_left_string, namespace='/manual_drive')
    return '200 OK', 200

@app.route('/api/car/<car_id>/drive/turn/left_stop')
def turn_left_stop(car_id):
    print("manual turn left stop")
    turn_left_stop_string = 'turn_left_stop/' + car_id
    socketio.emit(turn_left_stop_string, namespace='/manual_drive')
    return '200 OK', 200

@app.route('/api/car/<car_id>/drive/turn/right')
def turn_right(car_id):
    print("manual turn right")
    turn_right_string = 'turn_right/' + car_id
    socketio.emit(turn_right_string, namespace='/manual_drive')
    return '200 OK', 200

@app.route('/api/car/<car_id>/drive/turn/right_stop')
def turn_right_stop(car_id):
    print("manual turn right stop")
    turn_right_stop_string = 'turn_right_stop/' + car_id
    socketio.emit(turn_right_stop_string, namespace='/manual_drive')
    return '200 OK', 200

@app.route('/api/car/<car_id>/drive/throttle/up')
def throttle_up(car_id):
    print("manual throttle up")
    throttle_up_string = 'throttle_up/' + car_id
    socketio.emit(throttle_up_string, namespace='/manual_drive')
    return '200 OK', 200

@app.route('/api/car/<car_id>/drive/throttle/up_stop')
def throttle_up_stop(car_id):
    print("manual throttle up stop")
    throttle_up_stop_string = 'throttle_up_stop/' + car_id
    socketio.emit(throttle_up_stop_string, namespace='/manual_drive')
    return '200 OK', 200

@app.route('/api/car/<car_id>/drive/throttle/back')
def throttle_back(car_id):
    print("manual throttle back")
    throttle_back_string = 'throttle_back/' + car_id
    socketio.emit(throttle_back_string, namespace='/manual_drive')
    return '200 OK', 200

@app.route('/api/car/<car_id>/drive/throttle/back_stop')
def throttle_back_stop(car_id):
    print("manual throttle back stop")
    throttle_back_stop_string = 'throttle_back_stop/' + car_id
    socketio.emit(throttle_back_stop_string, namespace='/manual_drive')
    return '200 OK', 200

@app.route('/api/car/<car_id>/get/direction')
def get_direction(car_id):
    car_json = redis.get_car_json(car_id)
    return json.dumps(car_json['direction'])

@app.route('/api/car/<car_id>/get/servo_direction')
def get_servo_direction(car_id):
    car_json = redis.get_car_json(car_id)
    return json.dumps(car_json['servo_direction'])

@app.route('/api/car/<car_id>/toggle/direction')
def toggle_driving_direction(car_id):
    car_json = redis.get_car_json(car_id)
    car_json['direction'] = -1*car_json['direction']
    redis.set_car_json(car_id, json.dumps(car_json))
    direction2cv_string = 'direction2cv/' + car_id
    socketio.emit(direction2cv_string, namespace='/cv')
    return '200 OK', 200

@app.route('/api/car/<car_id>/toggle/servo_direction')
def toggle_driving_servo_direction(car_id):
    car_json = redis.get_car_json(car_id)
    car_json['servo_direction'] = -1*car_json['servo_direction']
    redis.set_car_json(car_id, json.dumps(car_json))
    servodirection2cv_string = 'servodirection2cv/' + car_id
    socketio.emit(servodirection2cv_string, namespace='/cv')
    return '200 OK', 200

@app.route('/api/car/<car_id>/get/stream')
def is_streaming(car_id):
    car_json = redis.get_car_json(car_id)
    return json.dumps(car_json["video_stream"])

@app.route('/api/car/<car_id>/disable/video')
def disable_video(car_id):
    car_json = redis.get_car_json(car_id)
    car_json['video_stream'] = False
    redis.set_car_json(car_id, json.dumps(car_json))
    disable2cv_string = 'disable2cv/' + car_id
    socketio.emit(disable2cv_string, namespace='/cv')
    return '200 OK', 200

@app.route('/api/car/<car_id>/enable/video')
def enable_video(car_id):
    car_json = redis.get_car_json(car_id)
    car_json['video_stream'] = True
    redis.set_car_json(car_id, json.dumps(car_json))
    enable2cv_string = 'enable2cv/' + car_id
    socketio.emit(enable2cv_string, namespace='/cv')
    return '200 OK', 200

# GET request handlers for servo/esc channel selection  
@app.route('/api/car/<car_id>/set/servo_channel/<channel>', methods=['POST'])
def set_servoChannel(car_id, channel):
    car_json = redis.get_car_json(car_id)
    car_json['servo_channel'] = channel
    redis.set_car_json(car_id, json.dumps(car_json))
    return '200 OK', 200

@app.route('/api/car/<car_id>/get/servo_channel')
def get_servoChannel(car_id):
    car_json = redis.get_car_json(car_id)
    return json.dumps(car_json['servo_channel'])

@app.route('/api/car/<car_id>/set/esc_channel/<channel>', methods=['POST'])
def set_escChannel(car_id, channel):
    car_json = redis.get_car_json(car_id)
    car_json['esc_channel'] = channel
    redis.set_car_json(car_id, json.dumps(car_json))
    return '200 OK', 200

@app.route('/api/car/<car_id>/get/esc_channel')
def get_escChannel(car_id):
    car_json = redis.get_car_json(car_id)
    return json.dumps(car_json['esc_channel'])

# GET request handler for tests/diagnostics
@app.route('/api/car/<car_id>/test/servo_angle')
def test_servo_angle(car_id):
    print("test servo angle")
    servo_angle_string = 'test/servo_angle/' + car_id
    socketio.emit(servo_angle_string, namespace='/test_diagnostic')
    return '200 OK', 200

@app.route('/api/car/<car_id>/test/throttle')
def test_throttle(car_id):
    print("test throttle")
    throttle_string = 'test/throttle/' + car_id
    socketio.emit(throttle_string, namespace='/test_diagnostic')
    return '200 OK', 200

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000,  debug=True)
