#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import paho.mqtt.client as mqtt
import sys

# MQTT Settings
MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_TOPIC = "esp32/ultrasonic/distance"

class MqttUltrasonicNode(Node):
    def __init__(self, car_name):
        super().__init__('mqtt_ultrasonic_node')

        topic_name = f'{car_name}/ultrasonic_distance'
        self.get_logger().info(f'Publishing to ROS 2 topic: {topic_name}')

        # Create a ROS 2 publisher
        self.publisher_ = self.create_publisher(Float32, topic_name, 10)

        # Setup MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message

        # Connect to MQTT broker
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.get_logger().info(f"Connected to MQTT broker at {MQTT_BROKER}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {str(e)}")

        # Start MQTT loop in a separate thread
        self.mqtt_client.loop_start()

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT connected successfully")
            client.subscribe(MQTT_TOPIC)
        else:
            self.get_logger().error("MQTT connection failed")

    def on_mqtt_message(self, client, userdata, msg):
        if msg.topic == MQTT_TOPIC:
            try:
                distance = float(msg.payload.decode())
                self.get_logger().info(f"Received distance: {distance} cm")

                # Publish via ROS 2 topic
                ros_msg = Float32()
                ros_msg.data = distance
                self.publisher_.publish(ros_msg)
            except ValueError:
                self.get_logger().warn(f"Invalid value received: {msg.payload.decode()}")

def main(args=None):
    rclpy.init()

    # Get car_name from command line argument
    if len(sys.argv) < 2:
        print("Usage: python3 Main.py <car_name>")
        return

    car_name = sys.argv[1]
    node = MqttUltrasonicNode(car_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()