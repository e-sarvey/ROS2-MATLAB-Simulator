import paho.mqtt.client as mqtt
import time

# MQTT broker details
broker_address = "broker.hivemq.com"
client_id = "pick_place_python"
topic_angles = "robot_joint_angles"
topic_status = "robot_status"

# Define safe joint angles for pick-and-place operation
pick_position = "[0.0, -1.57, 1.57, 0.0, 0.0, 0.0]"  # Pick position
place_position = "[-3.14, -1.0, 1.0, 0.0, 0.0, 0.0]"  # Place position
intermediate_position_1 = "[0.0, -1.2, 1.2, 0.0, 0.0, 0.0]"  # Lift position 1
intermediate_position_2 = "[-3.14, -0.8, 0.8, 0.0, 0.0, 0.0]"  # Lift position 2 (above place position)

# Initialize state to track robot status
status_received = False

# Callback when a message is received from the robot/status topic
def on_message(client, userdata, message):
    global status_received
    if message.topic == topic_status:
        msg = message.payload.decode("utf-8")
        print(f"Status message received: {msg}")
        if msg == "done":
            status_received = True  # Set flag to send next command

# Callback when the client connects to the MQTT broker
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker")
    client.subscribe(topic_status)  # Subscribe to robot/status for feedback

# Initialize MQTT client
client = mqtt.Client(client_id)
client.on_message = on_message
client.on_connect = on_connect

# Connect to the broker
client.connect(broker_address)

# Start the MQTT client loop to listen for incoming messages
client.loop_start()

# Simulate pick and place operations with intermediate steps
try:
    while True:
        # Move to intermediate position 1 (lifting)
        print("Moving to intermediate position 1...")
        client.publish(topic_angles, intermediate_position_1)
        status_received = False

        # Wait for the "done" message before sending the next command
        while not status_received:
            time.sleep(0.1)

        # Move to pick position
        print("Moving to pick position...")
        client.publish(topic_angles, pick_position)
        status_received = False

        # Wait for the "done" message before sending the next command
        while not status_received:
            time.sleep(0.1)

        # Move to intermediate position 1 again (lifting from pick position)
        print("Moving to intermediate position 1...")
        client.publish(topic_angles, intermediate_position_1)
        status_received = False

        # Wait for the "done" message before sending the next command
        while not status_received:
            time.sleep(0.1)

        # Move to intermediate position 2 (above place position)
        print("Moving to intermediate position 2...")
        client.publish(topic_angles, intermediate_position_2)
        status_received = False

        # Wait for the "done" message before sending the next command
        while not status_received:
            time.sleep(0.1)

        # Move to place position
        print("Moving to place position...")
        client.publish(topic_angles, place_position)
        status_received = False

        # Wait for the "done" message before sending the next command
        while not status_received:
            time.sleep(0.1)

        # Move back to intermediate position 2 (lifting from place position)
        print("Moving to intermediate position 2...")
        client.publish(topic_angles, intermediate_position_2)
        status_received = False

        # Wait for the "done" message before sending the next command
        while not status_received:
            time.sleep(0.1)

except KeyboardInterrupt:
    print("Pick-and-place simulation stopped.")

# Stop the loop and disconnect from the broker
client.loop_stop()
client.disconnect()
