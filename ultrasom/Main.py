import paho.mqtt.client as mqtt

# MQTT Settings
broker = "broker.hivemq.com"
port = 1883
topic = "esp32/ultrasonic/distance"

# Callback when the client connects to the broker
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    # Subscribe to the topic
    client.subscribe(topic)

# Callback when a message is received
def on_message(client, userdata, msg):
    if msg.topic == topic:
        try:
            distance = float(msg.payload.decode())
            print(f"Received distance: {distance} cm")
        except ValueError:
            print(f"Invalid distance value received: {msg.payload.decode()}")

# Create MQTT client instance
client = mqtt.Client()

# Assign callback functions
client.on_connect = on_connect
client.on_message = on_message

# Connect to the broker
client.connect(broker, port, 60)

# Start the network loop
client.loop_forever()