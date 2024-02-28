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

        # Create a client instance
        self.client = mqtt.Client("UnicycleRobot")
        self.client.connect(broker_address, port)
        
        self.send_queue = send_queue
        self.receive_queue = receive_queue
        
        self.setup_callbacks()  # Setup MQTT callbacks
        
        # # Start the background thread
        # self.active = True  # Control flag
        # self.thread = threading.Thread(target=self.process_queues)
        # self.thread.daemon = True
        # self.thread.start()
        
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
        
    def communication_loop(self):
        # Start the loop to process MQTT messages
        self.client.loop_start()
        
        while True:
            # Handle outgoing messages
            if not self.send_queue.empty():
                topic, message = self.send_queue.get()
                self.publish_data(topic, message)
                self.send_queue.task_done()

            # Check for new MQTT messages (commands) and add them to the receive queue
            # This assumes you've set up appropriate callbacks for MQTT subscribe operations
            # You would typically use `self.client.on_message` callback to receive messages

            time.sleep(0.001)  # Sleep for 1ms to prevent busy waiting

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
        # self.active = False  # Signal the thread to exit
        # self.thread.join()   # Wait for the thread to finish
        # self.client.loop_stop()  # Stop the MQTT loop
        self.client.disconnect()