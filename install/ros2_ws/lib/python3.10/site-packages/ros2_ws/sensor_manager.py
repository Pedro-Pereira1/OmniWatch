import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)  # 1 second interval
        self.get_logger().info("Temperature sensor node started")

    def publish_temperature(self):
        temperature = round(random.uniform(20.0, 30.0), 2)
        msg = Float32()
        msg.data = temperature
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published temperature: {temperature} °C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
