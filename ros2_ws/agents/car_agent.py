import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from threading import Thread
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, OneShotBehaviour
from spade.message import Message
import asyncio
import json
import sys
import heapq
import math
import json
import rclpy
from std_msgs.msg import String as StringMsg
from geometry_msgs.msg import Twist
import time


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
        world_coords = [(x * self.resolution, y * self.resolution) for x, y in path]
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
        self.color_publisher = self.create_publisher(StringMsg, prefix + 'set_color', 10)
        
        self.create_subscription(StringMsg, prefix + 'logs', self.log_callback, 10)
        self.create_subscription(Float32, prefix + 'weight', self.weight_callback, 10)
        self.create_subscription(Odometry, prefix + 'odom', self.position_callback, 10)

    def log_callback(self, msg):
        self.log_data = msg.data

    def weight_callback(self, msg):
        self.weight_data = msg.data

    def position_callback(self, msg: Odometry):
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
            original_grid = [
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
            grid = [row[:] for row in original_grid]

            for car_id, (x, y) in agent.other_cars_positions.items():
                if car_id != agent.car_id:
                    gx, gy = int(round(x)), int(round(y))
                    if 0 <= gx < len(grid) and 0 <= gy < len(grid[0]):
                        grid[gx][gy] = 1
            planner = AStarPlanner(grid, resolution=1.0)

            current_x, current_y = agent.ros_node.position_data
            start_cell = (int(round(current_x)), int(round(current_y)))
            goal_cell = (int(round(goal_x)), int(round(goal_y)))

            path = planner.plan(start_cell, goal_cell)
            if path:
                if len(path) == 1:
                    print(f"[{agent.car_id}] Objetivo alcançado! Parando o planejamento.")
                planner.publish_path(agent.ros_node, f'{agent.car_id}/new_waypoint', path[1])
            else:
                print(f"[{agent.car_id}] A*: Nenhum caminho possível. Objectivo: {goal_cell}")

class CarAgent(Agent):
    def __init__(self, jid, password, car_id, patrol_points):
        super().__init__(jid, password)
        self.car_id = car_id
        self.quarantine = False
        self.is_infected = False
        self._stop_requested = False
        self.goal = None
        self.is_moving = False
        self.other_cars_positions = {}
        self.known_cars = ["car_1@localhost", "car_2@localhost", "car_3@localhost", "car_4@localhost"]#, "car_5@localhost", "car_6@localhost"]
        self.known_zone_managers = ["zone_1@localhost", "zone_2@localhost", "zone_3@localhost", "zone_4@localhost"]
        self.patrol_points = patrol_points if patrol_points else [(5.0, 1.0), (1.0, 9.0), (9.0, 9.0), (9.0, 1.0)]
        self.patrol_index = 0
        self.mission_type = "patrol"
        self.ready_cars = set()
        self.distances = []
        self.passenger_present = False

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
                #print(f"[{self.agent.car_id}] Sending: {msg.body}")
                await self.send(msg)

                await asyncio.sleep(60 if self.agent.quarantine else 30)

    class ControlBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=5)
            if msg:
                try:
                    data = json.loads(msg.body)
                    cmd = data.get("command")
                    if cmd == "plan_path_request":
                        goal = data.get("goal")
                        cars = data.get("cars", [])
                        if len(goal) == 2:
                            self.agent.temp_goal = goal
                            self.agent.add_behaviour(self.agent.CompetitivePathEvaluator(goal, None, msg.sender,cars))
                    elif cmd == "stop":
                        print(f"[{self.agent.car_id}] 🛑 STOP command received.")
                        current_x, current_y = self.agent.ros_node.position_data
                        print(f"[{self.agent.car_id}] The client wanted to go to the point {self.agent.goal}")
                        ride_end = self.agent.goal if self.agent.goal else [current_x, current_y]
                        zone_manager_jid = "zone_1@localhost"
                        self.agent._stop_requested = True
                        self.agent.is_infected = True

                        # Check passenger status and mission state
                        if self.agent.passenger_present or self.agent.mission_type == "mission":
                            reason = "🧍 Passenger onboard" if self.agent.passenger_present else "📦 Aborting mission (no passenger)"
                            print(f"[{self.agent.car_id}] {reason}. Requesting ride continuation...")

                            ride_request_msg = Message(to=zone_manager_jid)
                            ride_request_msg.body = json.dumps({
                                "command": "ride_request",
                                "start": [current_x - 1, current_y],
                                "end": [ride_end[0], ride_end[1]]
                            })
                            ride_request_msg.set_metadata("performative", "inform")
                            await self.send(ride_request_msg)
                        else:
                            print(f"[{self.agent.car_id}] ✅ Stopped. No further action needed.")                          
                    elif cmd == "quarantine":
                        print(f"Quarantine command received.")
                        self.agent.quarantine = True
                    elif cmd == "ready_for_election":
                        self.agent.ready_cars.add(data["car_id"] + "@localhost")
                    elif cmd == "distance_update":
                        print("I received an update on the distance.")
                        self.agent.distances.append((data["car_id"], data["distance"]))
                    elif cmd == "position_update":
                        car_id = data.get("car_id")
                        position = data.get("position", {})
                        if car_id and "lat" in position and "lon" in position:
                            self.agent.other_cars_positions[car_id] = (position["lat"], position["lon"])
                    elif cmd == "ride_request":
                        start = data.get("start")
                        end = data.get("end")
                        cars = data.get("cars", [])
                        if len(start) == 2:
                            self.agent.temp_goal = start
                            self.agent.add_behaviour(self.agent.CompetitivePathEvaluator(start, end, "mission", msg.sender,cars))
                    elif cmd == "change_patrol_points":
                        if self.agent.mission_type == "patrol":
                            points = data.get("patrol_points")
                            self.agent.mission_type = "changing"
                            print(points[0])
                            self.agent.add_behaviour(self.agent.RepeatedPathPlanner(points[0], mission="changing", isEnd=True)) 
                            self.agent.patrol_points = points
                            print(f"[{self.agent.car_id}] I'm a changed man.")
                        
                except Exception as e:
                    print(f"[{self.agent.car_id}] Erro ao processar controle: {e}")

    class CompetitivePathEvaluator(CyclicBehaviour):
        def __init__(self, goal, end=None, mission_id=None, sender=None, cars=[]):
            super().__init__()
            self.goal = goal
            self.end = end
            self.zone_manager_jid = str(sender)
            self.known_cars = cars

        async def run(self):
            # Step 1: Send "I'm ready"
            for car in self.known_cars:
                if car == self.agent.jid:
                    continue
                msg = Message(to=car)
                msg.body = json.dumps({
                    "command": "ready_for_election",
                    "car_id": self.agent.car_id
                })
                msg.set_metadata("performative", "inform")
                await self.send(msg)

            self.agent.ready_cars.add(self.agent.jid)

            # Step 2: Wait for other ready messages
            timeout = 5  # seconds max to wait
            start_time = self.agent.loop.time()
            print(self.agent.ready_cars)
            while len(self.agent.ready_cars) < len(self.known_cars):
                print(f"[{self.agent.car_id}] {len(self.agent.ready_cars)}/{len(self.known_cars)}")
                print(f"[{self.agent.car_id}] I'm missing some cars.")
                await asyncio.sleep(1)
                if self.agent.loop.time() - start_time > timeout:
                    print(f"[{self.agent.car_id}] Timeout waiting for ready signals.")
                    break

            print(f"[{self.agent.car_id}] All ready: {self.agent.ready_cars}")

            # Phase 2: Perform the actual election
            await self.perform_distance_election()

            self.kill()

        async def perform_distance_election(self):
            my_position = self.agent.ros_node.position_data
            my_distance = math.hypot(self.goal[0] - my_position[0], self.goal[1] - my_position[1])
            if self.agent.mission_type == "mission" or self.agent.is_infected:
                my_distance = float('inf')

            # Inform other cars
            for other_car in self.known_cars:
                if other_car == self.agent.jid:
                    continue
                msg = Message(to=str(other_car))
                msg.body = json.dumps({
                    "command": "distance_update",
                    "car_id": self.agent.car_id,
                    "distance": my_distance
                })
                msg.set_metadata("performative", "inform")
                await self.send(msg)

            # Wait to receive distances
            best_car = self.agent.car_id
            best_distance = my_distance

            while len(self.agent.distances) < len(self.agent.known_cars) - 1:
                print(f"[{self.agent.car_id}] It's missing some distances. {len(self.agent.distances)}/{len(self.agent.known_cars)}")
                await asyncio.sleep(1)
                continue

            for car_distance in self.agent.distances:
                if car_distance[1] < best_distance:
                    best_car = car_distance[0]
                    best_distance = car_distance[1]

            print(f"Im car {self.agent.car_id} and my distance is: {my_distance}. The best car is {best_car}")

            if best_car == self.agent.car_id:
                print(f"[{self.agent.car_id}] I am the closest to the goal. Taking the task.")
                self.agent.goal = self.goal
                self.agent.mission_type = "mission"
                #color = StringMsg()
                #color.data = "blue"
                #self.agent.ros_node.color_publisher.publish(color)

                msg = Message(to=self.zone_manager_jid)
                msg.body = json.dumps({
                    "command": "handling_goal",
                    "response": "handling_goal",
                    "car_id": self.agent.car_id
                })
                msg.set_metadata("performative", "inform")
                await self.send(msg)

                print(f"[{self.agent.car_id}] Starting a mission...")
                print(f"{self.goal}/{self.end}")
                self.agent.add_behaviour(self.agent.MissionCoordinator(self.goal, self.end))
                
            # Clean the distances and the ready cars
            self.agent.ready_cars = set()
            self.agent.distances = []  
  
    class SharePositionBehaviour(CyclicBehaviour):
        async def run(self):
            self.agent.passenger_present = self.agent.ros_node.weight_data > 0.0
            if self.agent._stop_requested:
                print(f"[{self.agent.car_id}] STOP requested, halting position sharing.")
                await asyncio.sleep(50)
            else:
                all_targets = self.agent.known_cars + self.agent.known_zone_managers
                for target in all_targets:
                    if target == self.agent.jid:
                        continue
                    msg = Message(to=str(target))
                    msg.body = json.dumps({
                        "command": "position_update",
                        "car_id": self.agent.car_id,
                        "position": {
                            "lat": self.agent.ros_node.position_data[0],
                            "lon": self.agent.ros_node.position_data[1]
                        },
                        "mission":self.agent.mission_type
                    })
                    msg.set_metadata("performative", "inform")
                    await self.send(msg)

                await asyncio.sleep(1)

    class RepeatedPathPlanner(CyclicBehaviour):
        def __init__(self, goal, tolerance=0.1, mission=None, isEnd = False):
            super().__init__()
            self.goal = goal
            self.tolerance = tolerance
            self.mission = mission
            self.isEnd = isEnd

        async def run(self):
            print(f"[{self.agent.car_id}] I'm executing ", self.agent.mission_type)
            current_x, current_y = self.agent.ros_node.position_data
            distance = math.hypot(self.goal[0] - current_x, self.goal[1] - current_y)
            print(f"[{self.agent.car_id}] Distância ao objetivo: {distance:.2f} m")
            print(f"[{self.agent.car_id}] My mission: {self.mission}/{self.agent.mission_type}")
            if self.mission != self.agent.mission_type:
                self.kill()
                return
            if self.agent.passenger_present and self.agent.is_infected:
                self.kill()
                return
            
            if distance > self.tolerance:
                try:
                    calculate_and_publish_path(self.agent, self.goal[0], self.goal[1])
                except:
                    pass
                await asyncio.sleep(3)
            else:
                print(f"[{self.agent.car_id}] Objetivo alcançado! Parando o planejamento.")
                self.agent.goal = None
                
                if self.agent.mission_type != "patrol":
                    if self.isEnd:
                        print(f"[{self.agent.car_id}] Missão finalizada. Retomando patrulha.")
                        self.agent.mission_type = "patrol"
                        self.agent.add_behaviour(self.agent.PatrolBehaviour())

                self.kill()

    class PatrolBehaviour(CyclicBehaviour):
        async def run(self):
            if self.agent._stop_requested or (self.agent.mission_type != "patrol" and self.agent.mission_type != "changing"):
                self.kill()
                return

            if self.agent.goal is not None:
                await asyncio.sleep(2)
                return

            # Definir o próximo ponto da patrulha
            next_point = self.agent.patrol_points[self.agent.patrol_index]
            print(f"[{self.agent.car_id}] Iniciando patrulha para o ponto {next_point}")
            self.agent.goal = next_point
            self.agent.mission_type = "patrol"
            print(f"[{self.agent.car_id}] Faço patrulha para os pontos {self.agent.patrol_points}")
            #color = StringMsg()
            #color.data = "red"
            #self.agent.ros_node.color_publisher.publish(color)
            self.agent.add_behaviour(self.agent.RepeatedPathPlanner(next_point, mission="patrol"))

            # Avança para o próximo ponto na próxima iteração
            self.agent.patrol_index = (self.agent.patrol_index + 1) % len(self.agent.patrol_points)

            await asyncio.sleep(5)

    class MissionCoordinator(OneShotBehaviour):
        def __init__(self, start, end=None):
            super().__init__()
            self.st = start
            self.end = end

        async def run(self):
            print(f"[{self.agent.car_id}] Starting MissionCoordinator with goal: {self.st}, end: {self.end}")
            self.agent.goal = self.st
            planner1 = self.agent.RepeatedPathPlanner(self.st, mission="mission", isEnd=(self.end is None))
            self.agent.add_behaviour(planner1)
            await planner1.join()  # Wait for first destination
            
            if self.end:
                while not self.agent.ros_node.weight_data > 0.0:
                    print(f"[{self.agent.car_id}] Waiting for client: {self.agent.ros_node.weight_data}")
                    time.sleep(1)
                self.agent.goal = self.end
                planner2 = self.agent.RepeatedPathPlanner(self.end, mission="mission", isEnd=True)
                self.agent.add_behaviour(planner2)
                await planner2.join()

    class ColorStatePublisher(CyclicBehaviour):
        async def run(self):
            a = self.agent
            # Determine composite state
            if a.is_infected:
                state = "Infetado"
            elif a.passenger_present and a.quarantine:
                state = "Passageiro/Quarentena"
            elif a.passenger_present:
                state = "Passageiro"
            elif a.mission_type == "patrol" and a.quarantine:
                state = "Patrulha/Quarentena"
            elif a.mission_type == "patrol":
                state = "Patrulha"
            elif a.quarantine:
                state = "Quarentena"
            elif a.mission_type == "changing":
                state = "Changing"
            else:
                state = "Normal"

            # Publish only if state changed
            if state != getattr(a, "_last_color_state", None):
                msg = StringMsg()
                msg.data = state
                a.ros_node.color_publisher.publish(msg)
                a._last_color_state = state
            await asyncio.sleep(1)  # Adjust interval as needed

    async def setup(self):
        print(f"[{self.car_id}] Agent started")
        thread = Thread(target=start_ros_node, args=(self, self.car_id), daemon=True)
        thread.start()
        await asyncio.sleep(2)
        self.add_behaviour(self.SendDataBehaviour())
        self.add_behaviour(self.ControlBehaviour())
        self.add_behaviour(self.SharePositionBehaviour())
        self.add_behaviour(self.PatrolBehaviour())
        self.add_behaviour(self.ColorStatePublisher())


if __name__ == "__main__":
    async def main():
        car_id = sys.argv[1]

        # Parse optional patrol points from command-line arguments
        patrol_args = sys.argv[2:]  # Expecting a flat list like: 1.0 1.0 1.0 9.0 9.0 9.0 ...
        patrol_points = []
        if patrol_args and len(patrol_args) % 2 == 0:
            for i in range(0, len(patrol_args), 2):
                x = float(patrol_args[i])
                y = float(patrol_args[i + 1])
                patrol_points.append((x, y))

        agent = CarAgent(f"{car_id}@localhost", "pass", car_id, patrol_points)
        await agent.start(auto_register=True)
        while agent.is_alive():
            await asyncio.sleep(1)

    asyncio.run(main())






