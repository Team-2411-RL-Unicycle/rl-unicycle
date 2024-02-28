import paho.mqtt.client as mqtt
import json
import threading
import time
import asyncio

class MQTTClient:
    COMMAND_TOPIC = "robot/commands"
    
    def __init__(self, send_queue, receive_queue):
        # Define the MQTT settings
        broker_address = "172.22.1.1" #Lenovo Mosquitto Broker Adress
        port = 1883
        
        self.loop_time = .001 #1ms

        # Create a client instance
        self.client = mqtt.Client("UnicycleRobot")
        self.client.connect(broker_address, port)
        
        self.send_queue = send_queue
        self.receive_queue = receive_queue
        
        self.setup_callbacks()  # Setup MQTT callbacks
        self.client.loop_start()
                    
    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        self.client.subscribe(self.COMMAND_TOPIC)  # Subscribe to a topic for commands

    def on_message(self, client, userdata, msg):
        # Assuming the message payload is JSON-formatted
        command = json.loads(msg.payload)
        self.receive_queue.put(command)  # Add the received command to the receive queue

    def setup_callbacks(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
    def start(self):
        self.telemetry_loop()
        
    def telemetry_loop(self):        
        while True:
            # Handle outgoing messages
            while not self.send_queue.empty():
                topic, message = self.send_queue.get()
                self.publish_data(topic, message)
                self.send_queue.task_done()

            time.sleep(self.loop_time)  # Sleep for a period to prevent busy-waiting

    def publish_data(self, topic, data):
        self.client.publish(topic, json.dumps(data))
        pass
            
    def shutdown(self):
        self.client.disconnect()