#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math
import numpy as np
import json  # Para ler o ponto publicado
import pandas as pd
import random
from ament_index_python.packages import get_package_share_directory
import os
from stable_baselines3 import PPO
import torch

ALIGN_THRESHOLD = 0.08   # radianos (~5 graus)
DIST_THRESHOLD = 0.1     # metros
ANGULAR_SPEED = 0.8      # rad/s
LINEAR_SPEED = 1.0       # m/s
pkg_share = get_package_share_directory('simple_car')
model_path = os.path.join(pkg_share, 'model', 'ppo_simple_random')

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.declare_parameter('car_name', 'car_1')
        car_name = self.get_parameter('car_name').get_parameter_value().string_value
        pkg = get_package_share_directory('simple_car')
        dir = os.path.join(pkg, 'models', 'cic-collection_reduzido.parquet')
        df = pd.read_parquet(dir)
        self.X = df.drop(columns=["Label", "ClassLabel"])
        self.publisher = self.create_publisher(Twist, f'{car_name}/cmd_vel', QoSProfile(depth=10))
        self.publisher_logs = self.create_publisher(String, f'{car_name}/logs', QoSProfile(depth=10))
        #self.publisher_pose = self.create_publisher(String, f'{car_name}/pose', QoSProfile(depth=10))
        self.publisher_weight = self.create_publisher(Float32, f'{car_name}/weight', QoSProfile(depth=10))
        self.subscription = self.create_subscription(Odometry, f'{car_name}/odom', self.odom_callback, QoSProfile(depth=10))

        # Subscrever ao novo tópico para receber um waypoint dinâmico
        self.subscription_new_wp = self.create_subscription(
            String,
            f'{car_name}/new_waypoint',
            self.new_waypoint_callback,
            QoSProfile(depth=10)
        )

        self.subscription_color = self.create_subscription(
            String,
            f'{car_name}/set_color',
            self.set_color_callback,
            QoSProfile(depth=10)
        )

        self.pose = None
        self.yaw = None
        self.current_goal = None
        
        self.logs_timer = self.create_timer(1, self.publish_logs_callback)
        self.timer = self.create_timer(0.1, self.move_to_goal)

        self.model = PPO.load(model_path)
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def publish_logs_callback(self):
        index = random.randint(0, len(self.X) - 1)
        sample = self.X.iloc[[index]]
        log_msg = String()
        log_msg.data = sample.to_json()
        self.get_logger().info(f"[ROS2] Novo objetivo recebido: {self.current_goal}")
        self.publisher_logs.publish(log_msg)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def new_waypoint_callback(self, msg):
        try:
            # Espera JSON simples: [x, y]
            point = json.loads(msg.data)
            if isinstance(point, list) and len(point) == 2:
                self.current_goal = (float(point[0]), float(point[1]))
                self.get_logger().info(f"[ROS2] Novo objetivo recebido: {self.current_goal}")
            else:
                self.get_logger().error("[ROS2] Formato inválido. Espera: [x, y]")
        except Exception as e:
            self.get_logger().error(f"[ROS2] Erro ao processar novo waypoint: {e}")

    def move_to_goal(self):
        if self.pose is None or self.yaw is None or self.current_goal is None:
            self.publisher.publish(Twist())
            return

        goal_x, goal_y = self.current_goal
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = self.normalize_angle(angle_to_goal - self.yaw)

        obs = np.array([
            distance,
            math.sin(heading_error),
            math.cos(heading_error),
            self.linear_vel,
            self.angular_vel,
            0.0  # progress info opcional
        ], dtype=np.float32)

        action, _ = self.model.predict(obs, deterministic=True)
        lin = float(np.clip(action[0], 0.0, 0.6))  # ou [0, 0.6] conforme treino
        ang = float(np.clip(action[1], -0.4, 0.4))

        # Atualizar estados simulados (não obrigatório, só se quiseres log)
        self.linear_vel = lin
        self.angular_vel = ang

        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.publisher.publish(msg)

        if distance < DIST_THRESHOLD:
            print("[ROS2] Objetivo alcançado!")
            self.current_goal = None


        # Simular peso
        #random_value = np.random.uniform(15, 300)
        #weight_msg = Float32()
        #weight_msg.data = random_value
        #self.publisher_weight.publish(weight_msg)

    def set_color_callback(self, msg):
        color = msg.data.lower()
        car_name = self.get_parameter('car_name').get_parameter_value().string_value

        self.get_logger().info(f"[ROS2] Mudando cor de {car_name} para {color}")

        # Mapeamento simples para materiais do Gazebo
        material_map = {
            'red': 'Gazebo/Red',
            'green': 'Gazebo/Green',
            'blue': 'Gazebo/Blue',
            'yellow': 'Gazebo/Yellow',
            'white': 'Gazebo/White',
            'black': 'Gazebo/Black'
        }

        material = material_map.get(color)
        if not material:
            self.get_logger().error(f"[ROS2] Cor '{color}' não suportada.")
            return

        # Comando de sistema para mudar o material via gz command
        cmd = f'gz model -m {car_name} -v 0 -M {material}'
        os.system(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
