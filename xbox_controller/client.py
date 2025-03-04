import os
import json
import time
import multiprocessing as mp
import paho.mqtt.client as mqtt
import pygame
import numpy as np


# MQTT Client Class
class MQTTClient:
    COMMAND_TOPIC = "robot/commands"

    def __init__(
        self, send_queue: mp.Queue, receive_queue: mp.Queue, shutdown_event: mp.Event
    ):
        self.broker_address = "172.22.1.1"  # Lenovo Mosquitto Broker Address
        self.port = 1883
        self.loop_time = 0.001  # 1ms

        self.client = mqtt.Client(clean_session=True)
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address, self.port)

        self.send_queue = send_queue
        self.receive_queue = receive_queue
        self.shutdown_event = shutdown_event

        self.client.loop_start()  # Start MQTT listener loop

    def on_message(self, client, userdata, msg):
        """Handles incoming messages and adds them to the receive queue."""
        try:
            command = json.loads(msg.payload)
            self.receive_queue.put(command)
        except json.JSONDecodeError:
            print(f"Invalid JSON received: {msg.payload}")

    def publish_command(self, command: dict):
        """Publishes a command message to the robot."""
        payload = json.dumps(command)
        self.client.publish(self.COMMAND_TOPIC, payload)

    def stop(self):
        """Stops the MQTT client."""
        self.client.loop_stop()
        self.client.disconnect()


# Xbox Controller Handling
class XboxController:
    def __init__(self, mqtt_client):
        pygame.init()
        pygame.joystick.init()

        self.mqtt_client = mqtt_client
        self.joystick = None
        self.connected = False

        self.detect_controller()

    def detect_controller(self):
        """Detects and initializes an Xbox controller."""
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            name = joystick.get_name()

            if "Xbox" in name:
                self.joystick = joystick
                self.connected = True
                print(f"Connected to: {name}")
                return

        print("No Xbox controller detected.")

    def read_stick_and_publish(self):
        """Reads the right analog stick (yaw) and sends a command."""
        if not self.connected:
            self.detect_controller()
            return

        pygame.event.pump()  # Process events

        yaw_value = self.joystick.get_axis(2)  # Right stick X-axis
        yaw_value = max(-1, min(yaw_value, 1))  # Normalize to -1 to 1
        if abs(yaw_value) < 0.03:
            yaw_value = 0 + np.random.normal(0, 0.0001)

        roll_value = self.joystick.get_axis(0)  # Right stick X-axis
        roll_value = max(-1, min(roll_value, 1))  # Normalize to -1 to 1
        if abs(roll_value) < 0.03:
            roll_value = 0 + np.random.normal(0, 0.0001)

        pitch_value = self.joystick.get_axis(1)  # Left stick Y-axis
        pitch_value = max(-1, min(pitch_value, 1))
        if abs(pitch_value) < 0.03:
            pitch_value = 0 + np.random.normal(0, 0.0001)

        pitch_value = -pitch_value

        # Send all values in a single command
        command = {"yaw": yaw_value, "pitch": pitch_value, "roll": roll_value}
        self.mqtt_client.publish_command(command)
        print(yaw_value)


# Main function
def main():
    send_queue = mp.Queue()
    receive_queue = mp.Queue()
    shutdown_event = mp.Event()

    mqtt_client = MQTTClient(send_queue, receive_queue, shutdown_event)
    controller = XboxController(mqtt_client)

    try:
        while True:
            controller.read_stick_and_publish()
            time.sleep(0.05)  # 10 Hz update rate
    except KeyboardInterrupt:
        print("\nShutting down...")
        mqtt_client.stop()
        pygame.quit()


if __name__ == "__main__":
    main()
