import sys, os, ctypes
import queue
import threading
import robot_system as rs
from communication.mqtt import MQTTClient

# Define constants for the scheduling policy
SCHED_FIFO = 1  # FIFO real-time policy

class SchedParam(ctypes.Structure):
    _fields_ = [('sched_priority', ctypes.c_int)]
    
def set_realtime_priority(priority=99):
    libc = ctypes.CDLL('libc.so.6')
    param = SchedParam(priority)
    # Set the scheduling policy to FIFO and priority for the entire process (0 refers to the current process)
    if libc.sched_setscheduler(0, SCHED_FIFO, ctypes.byref(param)) != 0:
        raise ValueError("Failed to set real-time priority. Check permissions.")

def start_robot(robot):
    robot.control_loop()
    
def start_mqtt(mqtt_client):
    mqtt_client.communication_loop()

def main():
    set_realtime_priority()
    # Create thread-safe queues for pub sub to mqtt handler
    mqtt_send_queue = queue.Queue()
    mqtt_receive_queue = queue.Queue()
    
    # Initialize MQTT client with queues
    mqtt_client = MQTTClient(mqtt_send_queue, mqtt_receive_queue)

    # Initialize the robot system with the telemetry queue
    robot = rs.RobotSystem(mqtt_send_queue, mqtt_receive_queue)
    
    # Start the control loop in its own thread
    control_thread = threading.Thread(target=start_robot, args=(robot,), daemon=True)
    control_thread.start()

    # Pass the MQTT client and the receive queue to the telemetry thread
    mqtt_thread = threading.Thread(target=start_mqtt, args=(mqtt_client,), daemon=True)
    mqtt_thread.start()
 
    try:
        control_thread.join()
        mqtt_thread.join()
    except KeyboardInterrupt:
        print("Shutdown signal received")
    finally:
        robot.shutdown()
        mqtt_client.shutdown()
        print("Cleanup complete. Exiting program.")

if __name__ == '__main__':
    main()