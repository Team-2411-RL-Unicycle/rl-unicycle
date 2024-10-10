import json
import logging
import multiprocessing as mp
import time

import paho.mqtt.client as mqtt

from rluni.robot.teledata import TelemetryData

logger = logging.getLogger(__name__)


class MQTTClient:
    COMMAND_TOPIC = "robot/commands"

    def __init__(
        self, send_queue: mp.Queue, receive_queue: mp.Queue, shutdown_event: mp.Event
    ):
        # Define the MQTT settings
        broker_address = "172.22.1.1"  # Lenovo Mosquitto Broker Adress
        port = 1883

        self.loop_time = 0.001  # 1ms

        # Create a client instance
        self.client = mqtt.Client(clean_session=True)
        self.client.connect(broker_address, port)

        self.send_queue = send_queue
        self.receive_queue = receive_queue
        self.shutdown_event = shutdown_event

        # Initialize batch counter
        self.batch_count = 0
        self.downsample_rate = 10

        self.setup_callbacks()  # Setup MQTT callbacks
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        logger.info("Connected with result code " + str(rc))
        self.client.subscribe(self.COMMAND_TOPIC)  # Subscribe to a topic for commands

    def on_message(self, client, userdata, msg):
        # Assuming the message payload is JSON-formatted
        command = json.loads(msg.payload)
        logger.debug(f"Command recieved: {command}")
        self.receive_queue.put(command)  # Add the received command to the receive queue

    def setup_callbacks(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        self.telemetry_loop()

    def telemetry_loop(self):
        while not self.shutdown_event.is_set():
            try:
                while not self.send_queue.empty():
                    batch = self.send_queue.get()

                    self.batch_count += 1  # Increment batch counter
                    if self.batch_count >= self.downsample_rate:
                        self.batch_count = 0
                        is_downsampled = True
                    else:
                        is_downsampled = False

                    if isinstance(batch, list):
                        for telemetry_data in batch:
                            self.process_telemetry_data(telemetry_data, is_downsampled)
                    else:
                        self.process_telemetry_data(batch, is_downsampled)
            except Exception as e:
                logger.exception(f"Error in telemetry_loop: {e}")

            time.sleep(self.loop_time)

    def process_telemetry_data(
        self, telemetry_data: TelemetryData, is_downsampled: bool
    ):
        # Check that the telemetry data is an instance of the TelemetryData class
        if not isinstance(telemetry_data, TelemetryData):
            logger.error(f"Invalid telemetry data: {telemetry_data}")
            return

        try:
            topic, data = telemetry_data.to_mqtt()

            # Publish to the original topic
            self.publish_data(topic, data)
            # logger.debug(f"Published to {topic}: {data}")

            # If this is a downsampled batch, also publish to the downsampled topic
            if is_downsampled:
                downsampled_topic = f"downsampled/{topic}"
                self.publish_data(downsampled_topic, data)
                # logger.debug(f"Published to {downsampled_topic}: {data}")

        except Exception as e:
            logger.exception(f"Error processing telemetry data for topic {topic}: {e}")

    def publish_data(self, topic, data):
        self.client.publish(topic, json.dumps(data))
        pass

    def shutdown(self):
        self.client.disconnect()
