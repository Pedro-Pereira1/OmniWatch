#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', QoSProfile(depth=10))
        self.timer = self.create_timer(0.1, self.send_command)

    def send_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
