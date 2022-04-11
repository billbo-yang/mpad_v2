import os
import time
import json
from dotenv import load_dotenv
from datetime import datetime
import sys
import redis

load_dotenv('.env')
r = redis.from_url(os.environ.get("REDIS_URL", 'redis://localhost:6379'))

# Hacky method for updating the order of ultrasonic readings
def bungle_ultrasonics(good_array):
    print(good_array)
    if len(good_array) < 5:
        return good_array
    bad_array = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    bad_array[0] = good_array[2]
    bad_array[1] = good_array[3]
    bad_array[2] = good_array[4]
    bad_array[3] = 0.0
    bad_array[4] = 0.0
    bad_array[5] = good_array[0]
    bad_array[6] = good_array[1]

    return bad_array

class RedisConn:
    def __init__(self):
        pass

    def get_car_json(self, car_id):
        return json.loads(r.get(car_id))

    def set_car_json(self, car_id, car_json):
        r.set(car_id, car_json)

    def link_ids(self, socket_id, car_id):
        r.set(socket_id, car_id)

    def remove_car(self, socket_id):
        car_id_encoded = r.get(socket_id)
        car_id = car_id_encoded.decode('utf-8')
        r.delete(socket_id)
        r.delete(car_id)
        cars = self.get_car_json('cars')
        cars.pop(car_id)
        self.set_car_json('cars', json.dumps(cars))

    # Get the most recent sensor string
    def sanitize_sensor_reading(self, car_id, sensor_data):
        output_dict = {
            "hall_effect": None,
            "battery": None,
            "temperature": None,
            "humidity": None,
            "imu": [None, None, None],
            "ultrasonic": [],
            "lipo": None
        }

        sensor_key = car_id + "-last_sensor_reading"
        if r.exists(sensor_key):
            last_reading_dict = json.loads(r.get(sensor_key))
            for last_reading in last_reading_dict:
                if "Distance" in last_reading:
                    output_dict['ultrasonic'].append(last_reading_dict[last_reading])
                if last_reading == "humidity":
                    output_dict['humidity'] = last_reading_dict[last_reading]
                if last_reading == "temperature":
                    output_dict['temperature'] = last_reading_dict[last_reading]
                if last_reading == "imu":
                    output_dict['imu'][0] = last_reading_dict[last_reading][0]
                    output_dict['imu'][1] = last_reading_dict[last_reading][1]
                    output_dict['imu'][2] = last_reading_dict[last_reading][2]
                if last_reading == "battery":
                    output_dict['battery'] = last_reading_dict['battery']
                if last_reading == "lipo":
                    output_dict['lipo'] = last_reading_dict[last_reading]
        else:
            r.set(sensor_key, json.dumps(sensor_data))

        for reading in sensor_data:
                if "Distance" in reading:
                    output_dict['ultrasonic'].append(sensor_data[reading])
                if reading == "humidity":
                    output_dict['humidity'] = sensor_data[reading]
                if reading == "temperature":
                    output_dict['temperature'] = sensor_data[reading]
                if reading == "x-axis":
                    output_dict['imu'][0] = sensor_data[reading]
                if reading == "y-axis":
                    output_dict['imu'][1] = sensor_data[reading]
                if reading == "z-axis":
                    output_dict['imu'][2] = sensor_data[reading]
                if reading == "cpu_temp":
                    output_dict['battery'] = sensor_data[reading]
                if reading == "LiPo":
                    output_dict['lipo'] = sensor_data[reading]

        # Hacky fix for replacing ultrasonics
        output_dict['ultrasonic'] = bungle_ultrasonics(output_dict['ultrasonic'])

        r.set(sensor_key, json.dumps(output_dict))

        return output_dict

    def store_sensor_readingtimestamps(self, car_id, car_json, sensor_readings):
        # Append the new readings to the historic data
        sensor_time = datetime.now().strftime('%H:%M:%S')
        car_json["timestamp"].append(sensor_time)
        car_json["hall_effect_data"].append(sensor_readings['hall_effect'])
        car_json["battery_data"].append(sensor_readings['battery'])
        car_json["temperature_data"].append(sensor_readings['temperature'])
        car_json["humidity_data"].append(sensor_readings['humidity'])
        car_json["imu_data"].append(sensor_readings['imu'])
        car_json["lipo_data"].append(sensor_readings['lipo'])
        for idx in range(len(sensor_readings['ultrasonic'])):
            if sensor_readings['ultrasonic'][idx] == "0":
                sensor_readings['ultrasonic'][idx] = 100
        car_json["ultrasonic_data"].append(sensor_readings['ultrasonic'])
        r.set(car_id, json.dumps(car_json))
        return json.dumps({
            "timestamp": sensor_time,
            "hall_effect_data": sensor_readings["hall_effect"],
            "battery_data": sensor_readings['battery'],
            "temperature_data": sensor_readings['temperature'],
            "humidity_data": sensor_readings['humidity'],
            "imu_data": sensor_readings['imu'],
            "ultrasonic_data": sensor_readings['ultrasonic'],
            "lipo_data": sensor_readings['lipo']
        })

    def lastNEntries(self, arr, entries):
        if (len(arr) < entries):
            entries = len(arr)
        return arr[-entries:]

    # Get the last stores entries as a dictionary
    def read_data(self, car_json):
        return {
            "timestamp": self.lastNEntries(car_json["timestamp"], 30),
            "hall_effect": self.lastNEntries(car_json["hall_effect_data"], 30),
            "battery": self.lastNEntries(car_json["battery_data"], 30),
            "temperature": self.lastNEntries(car_json["temperature_data"], 30),
            "humidity": self.lastNEntries(car_json["humidity_data"], 30),
            "imu": self.lastNEntries(car_json["imu_data"], 30),
            "ultrasonic": self.lastNEntries(car_json["ultrasonic_data"], 30),
            "lipo": self.lastNEntries(car_json["lipo_data"], 30)
        }
