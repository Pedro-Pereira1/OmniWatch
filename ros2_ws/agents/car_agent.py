import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from threading import Thread
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import asyncio
import json
import sys
import heapq
import math
import json
import rclpy
from std_msgs.msg import String as StringMsg

class AStarPlanner:
    def __init__(self, grid, resolution=1.0):
        self.grid = grid
        self.resolution = resolution

    def heuristic(self, a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def plan(self, start, goal):
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
                if 0 <= neighbor[0] < len(self.grid) and 0 <= neighbor[1] < len(self.grid[0]):
                    if self.grid[neighbor[0]][neighbor[1]] == 1:
                        continue
                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score, neighbor))

        print("A*: Caminho não encontrado.")
        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        world_coords = [(x * self.resolution + 0.5 * self.resolution,
                  y * self.resolution + 0.5 * self.resolution) for x, y in path]
        return world_coords

    def publish_path(self, node, topic, path):
        path_msg = StringMsg()
        path_msg.data = json.dumps(path)
        publisher = node.create_publisher(StringMsg, topic, 10)
        
        publisher.publish(path_msg)
        print(f"A*: Caminho publicado em '{topic}': {path_msg.data}")

class ROSCarListener(Node):
    def __init__(self, car_id):
        super().__init__(f'{car_id}_listener')
        prefix = f'{car_id}/'

        self.log_data = "No logs yet"
        self.weight_data = 0.0
        self.position_data = (0.0, 0.0)

        self.create_subscription(StringMsg, prefix + 'logs', self.log_callback, 10)
        self.create_subscription(Float32, prefix + 'weight', self.weight_callback, 10)
        self.create_subscription(Odometry, prefix + 'odom', self.position_callback, 10)

    def log_callback(self, msg):
        self.log_data = msg.data

    def weight_callback(self, msg):
        self.weight_data = msg.data

    def position_callback(self, msg: Twist):
        self.pose = msg.pose.pose.position
        self.position_data = (self.pose.x, self.pose.y)

def start_ros_node(agent, car_id):
    rclpy.init()
    agent.ros_node = ROSCarListener(car_id)
    try:
        rclpy.spin(agent.ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        agent.ros_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"[{car_id}] Shutdown warning: {e}")

def calculate_and_publish_path(agent, goal_x, goal_y):
            # Exemplo de grid 20x20 (igual ao teu anterior)
            grid = [
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
            planner = AStarPlanner(grid, resolution=1.0)

            # Posição atual do carro
            current_x, current_y = agent.ros_node.position_data
            start_cell = (int(round(current_x)), int(round(current_y)))
            goal_cell = (int(round(goal_x)), int(round(goal_y)))

            path = planner.plan(start_cell, goal_cell)
            if path:
                planner.publish_path(agent.ros_node, 'new_waypoint', path[1])
            else:
                print("A*: Nenhum caminho possível.")

class CarAgent(Agent):
    def __init__(self, jid, password, car_id):
        super().__init__(jid, password)
        self.car_id = car_id
        self.quarantine = False
        self._stop_requested = False
    
    class SendDataBehaviour(CyclicBehaviour):
        async def run(self):
            if self.agent._stop_requested:
                print(f"[{self.agent.car_id}] STOP requested, halting data transmission.")
                await asyncio.sleep(50)
            else:
                data = {
                    "car_id": self.agent.car_id,
                    "log": self.agent.ros_node.log_data,
                    "weight": self.agent.ros_node.weight_data,
                    "position": {
                        "lat": self.agent.ros_node.position_data[0],
                        "lon": self.agent.ros_node.position_data[1]
                    },
                    "quarantine": self.agent.quarantine
                }

                msg = Message(
                    to="main@localhost",
                    body=json.dumps(data),
                    metadata={"performative": "inform"}
                )
                print(f"[{self.agent.car_id}] Sending: {msg.body}")
                await self.send(msg)

                await asyncio.sleep(60 if self.agent.quarantine else 30)

    class ControlBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=5)
            if msg:
                try:
                    data = json.loads(msg.body)
                    print(data)
                    cmd = data.get("command")
                    if cmd == "plan_path":
                        goal = data.get("goal")  # espera: {"goal": [x, y]}
                        if goal and len(goal) == 2:
                            print(f"[{self.agent.car_id}] Recebido comando para planear caminho até {goal}")
                            calculate_and_publish_path(self.agent, goal[0], goal[1])
                        else:
                            print(f"[{self.agent.car_id}] Objetivo inválido para planeamento.")
                    elif cmd == "stop":
                        self.agent._stop_requested = True
                    elif cmd == "quarantine":
                        self.agent.quarantine = data.get("state", True)
                except Exception as e:
                    print(f"[{self.agent.car_id}] Erro ao processar controlo: {e}")


    async def setup(self):
        print(f"[{self.car_id}] Agent started")
        thread = Thread(target=start_ros_node, args=(self, self.car_id), daemon=True)
        thread.start()
        await asyncio.sleep(2)
        self.add_behaviour(self.SendDataBehaviour())
        self.add_behaviour(self.ControlBehaviour())


if __name__ == "__main__":
    async def main():
        car_id = sys.argv[1]
        agent = CarAgent(f"{car_id}@localhost", "pass", car_id)
        await agent.start(auto_register=True)
        while agent.is_alive():
            await asyncio.sleep(1)
    asyncio.run(main())






