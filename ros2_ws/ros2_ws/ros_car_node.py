import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import random
import sys
from sklearn.preprocessing import LabelEncoder
import pandas as pd
import joblib
import numpy as np

class CarNode(Node):
    def __init__(self, car_id):
        super().__init__(f'{car_id}_node')
        self.car_id = car_id

        df = pd.read_parquet("models/cic-collection_reduzido.parquet")
        self.X = df.drop(columns=["Label", "ClassLabel"])

        # Publishers
        self.log_pub = self.create_publisher(String, f'{car_id}/logs', 10)
        self.weight_pub = self.create_publisher(Float32, f'{car_id}/weight', 10)
        self.pos_pub = self.create_publisher(Twist, f'{car_id}/odom', 10)
        self.timer = self.create_timer(20.0, self.publish_data)

        self.get_logger().info(f"{car_id} node started")

    def publish_data(self):
        # Simulated log message
        #is_malicious = random.random() < 0.01  # 5% chance
        #label = "malicious" if is_malicious else "benign"
        #log_msg = String()
        #log_msg.data = f"{label}"

        index = random.randint(0, len(self.X) - 1)
        sample = self.X.iloc[[index]]
        log_msg = String()
        log_msg.data = sample.to_json()
        self.log_pub.publish(log_msg)

        # Simulated weight (0 = empty, ~75kg = someone inside)
        weight_msg = Float32()
        weight_msg.data = random.choice([0.0, 75.0])
        self.weight_pub.publish(weight_msg)

        # Simulated position as Twist
        pos_msg = Twist()
        pos_msg.linear.x = random.uniform(0, 100)   # x position
        pos_msg.linear.y = random.uniform(0, 100)   # y position (optional)
        pos_msg.angular.z = random.uniform(-1, 1)   # heading angle or turn rate
        self.pos_pub.publish(pos_msg)

        self.get_logger().info(
            f"{self.car_id} | log: {log_msg.data} | weight: {weight_msg.data} | pos: x={pos_msg.linear.x:.1f}, y={pos_msg.linear.y:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    car_id = sys.argv[1] if len(sys.argv) > 1 else 'car1'
    node = CarNode(car_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
