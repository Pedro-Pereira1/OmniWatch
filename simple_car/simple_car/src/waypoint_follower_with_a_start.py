#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
import math
import numpy as np
import heapq

# Parâmetros
ALIGN_THRESHOLD = 0.08   # radianos (~5 graus)
DIST_THRESHOLD = 0.1     # metros
ANGULAR_SPEED = 0.8      # rad/s
LINEAR_SPEED = 1.0       # m/s

# Grid para A* (exemplo simplificado, 0: livre, 1: obstáculo)
GRID = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]
GRID_RESOLUTION = 1.0  # metros por célula
START = (9, 0)
GOAL = (0, 9)

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.declare_parameter('car_name', 'car_1')
        car_name = self.get_parameter('car_name').get_parameter_value().string_value

        self.publisher = self.create_publisher(Twist, f'{car_name}/cmd_vel', QoSProfile(depth=10))
        self.publisher_weight = self.create_publisher(Float32, f'{car_name}/weight', QoSProfile(depth=10))
        self.subscription = self.create_subscription(Odometry, f'{car_name}/odom', self.odom_callback, QoSProfile(depth=10))

        self.pose = None
        self.yaw = None
        self.current_waypoint = 0

        # Calcular caminho A* e converter em waypoints
        self.waypoints = self.compute_a_star_path(START, GOAL)
        print(f"Waypoints calculados: {self.waypoints}")

        self.timer = self.create_timer(0.1, self.follow_waypoints)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1.0 - 2.0 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def follow_waypoints(self):
        if self.pose is None or self.yaw is None:
            print("Aguardando odometria...")
            return

        if self.current_waypoint >= len(self.waypoints):
            print("Todos os waypoints foram atingidos. Parando o robô.")
            self.publisher.publish(Twist())
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint]
        dx = goal_x - self.pose.x
        dy = goal_y - self.pose.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.yaw)

        msg = Twist()
        if abs(angle_diff) > ALIGN_THRESHOLD:
            msg.angular.z = max(min(ANGULAR_SPEED * angle_diff, ANGULAR_SPEED), -ANGULAR_SPEED)
            msg.linear.x = 0.0
            print("A alinhar (rodar parado)")
        else:
            msg.linear.x = min(LINEAR_SPEED, distance)
            msg.angular.z = 0.0
            print("A avançar (em frente)")

        if distance < DIST_THRESHOLD:
            print(f"Waypoint {self.current_waypoint} atingido. Indo para o próximo.")
            self.current_waypoint += 1

        self.publisher.publish(msg)

        # Simula peso
        #random_value = np.random.uniform(15, 300)
        #weight_msg = Float32()
        #weight_msg.data = random_value
        #self.publisher_weight.publish(weight_msg)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def compute_a_star_path(self, start, goal):
        def heuristic(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < len(GRID) and 0 <= neighbor[1] < len(GRID[0]):
                    if GRID[neighbor[0]][neighbor[1]] == 1:
                        continue  # obstáculo
                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, neighbor))

        print("Caminho não encontrado.")
        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        # Converter para coordenadas de mundo (exemplo: 1 célula = 1 metro)
        world_coords = [(x * GRID_RESOLUTION, y * GRID_RESOLUTION) for x, y in path]
        return world_coords

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
