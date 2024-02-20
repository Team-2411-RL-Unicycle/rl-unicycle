import paho.mqtt.client as mqtt
import json
import time
import asyncio

class MQTTClient:
    def __init__(self):
        # Define the MQTT settings
        broker_address = "172.22.0.1" #Pi's IP address
        port = 1883

        # Create a client instance
        self.client = mqtt.Client("UnicycleRobot")
        self.client.connect(broker_address, port)

    def publish_data(self, topic, data):
        self.client.publish(topic, json.dumps(data))
        pass
    
    def publish_sensor_data(self, ax, ay, az, gx, gy, gz):
        topic = "robot/sensors/imu"
        data = {
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "gyro_x" : gx,
            "gyro_y" : gy,
            "gyro_z" : gz,
        }
        self.publish_data(topic, data)
        
    def publish_loop_time(self, loop_time):
        topic = "robot/control/loop_time"
        data = {
            "loop_time": loop_time
        }
        self.publish_data(topic, data)
        
    def shutdown(self):
        self.client.disconnect()
        pass