import heapq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import gym
from gym import spaces
import numpy as np
import math
from stable_baselines3 import PPO
import time
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_path(grid, start, goal):
    neighbors = [(0,1),(1,0),(0,-1),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data.append(start)
            return data[::-1]  # caminho do start ao goal

        close_set.add(current)
        for i,j in neighbors:
            neighbor = current[0]+i, current[1]+j
            tentative_g_score = gscore[current] + 1
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]):
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue  # obstáculo
            else:
                continue  # fora do grid

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return []  # caminho não encontrado



class ROS2EnvWithPlanner(gym.Env):
    def __init__(self, grid, start, goal):
        super().__init__()
        rclpy.init()
        self.node = rclpy.create_node('rl_agent_node')
        print("🗺️  Planned path:") 

        self.publisher = self.node.create_publisher(Twist, '/car_1/cmd_vel', 10)
        self.subscription = self.node.create_subscription(Odometry, '/car_1/odom', self.odom_callback, 10)

        self.spawn_entity_client = self.node.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Serviço /spawn_entity não disponível, aguardando...')

        self.delete_entity_client = self.node.create_client(DeleteEntity, '/delete_entity')
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Serviço /delete_entity não disponível, aguardando...')
       
        self.grid = grid
        self.goal_pos = goal  # destino final no grid (tupla)
        self.start_pos = start  # posição inicial no grid (tupla)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Obter caminho A* no grid (lista de coords)
        self.path = a_star_path(self.grid, self.start_pos, self.goal_pos)
        self.path = self.path[1:]
        margin = int(len(self.path) * 0.3)  # 30% de folga
        self.max_steps = (len(self.path) + margin)*900
        self.steps = 0 
        for i, p in enumerate(self.path):
            print(f"  {i:02d}: {p}")
        if not self.path:
            raise RuntimeError("Caminho A* não encontrado")

        self.current_wp_index = 0
        self.goal = np.array([self.path[self.current_wp_index][0], self.path[self.current_wp_index][1]], dtype=np.float32)
        self.distance_prev = np.linalg.norm(self.goal - np.array([self.x, self.y]))
        
        # Definir espaços de observação e ação
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([0.0, -0.4]), high=np.array([0.6, 0.4]), dtype=np.float32)
        self.start_time = time.time()

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    # ... (imports and __init__ as before)

    def step(self, action):
        twist = Twist()
        max_lin = 0.6
        max_ang = 1.0
        twist.linear.x = float(action[0]) * max_lin
        twist.angular.z = float(action[1]) * max_ang
        self.publisher.publish(twist)

        rclpy.spin_once(self.node, timeout_sec=0.1)
        #time.sleep(0.1)
        #time.sleep(0.1)
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        distance_to_wp = np.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_to_goal - self.yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error)) 

        reward = 0.0
        done = False

        # --- 🎯 Recompensa por progresso ---
        progress = self.distance_prev - distance_to_wp
        reward += 15.0 * progress  # Menor peso que antes

        # --- 🎯 Alinhamento com o objetivo ---
        alignment_bonus = math.cos(angle_to_goal)  # Varia entre -1 e 1
        reward += 2.0 * alignment_bonus

        # --- ❌ Penalização por giros abruptos ---
        reward -= 0.2 * abs(action[1])  # Penaliza alta rotação

        # --- ❌ Penalização por lentidão ---
        if action[0] < 0.05:
            reward -= 0.8  # mais forte se estiver quase parado

        # --- ⏱️ Penalização de tempo ---
        reward -= 0.05  # pequena penalização por passo

        # --- 💥 Colisão ---
        if self.check_for_collision(self.x, self.y):
            reward -= 100.0
            done = True
            print("💥 Collision!")
            time.sleep(1.0)

        # --- ✅ Chegou no waypoint ---
        if distance_to_wp < 0.5:
            reward += 25.0
            self.current_wp_index += 1
            if self.current_wp_index >= len(self.path):
                reward += 300.0
                done = True
                print("🏁 Reached final goal")
            else:
                wp = self.path[self.current_wp_index]
                self.goal = np.array([wp[0], wp[1]], dtype=np.float32)
                self.distance_prev = np.linalg.norm(self.goal - np.array([self.x, self.y]))
                print(f"➡️ New waypoint: {self.goal}")

        self.distance_prev = distance_to_wp
        self.steps += 1

        if self.steps >= self.max_steps:
            reward -= 50.0 + 10.0 * distance_to_wp
            done = True
            print("⏳ Max steps reached")

        state = np.array([
            distance_to_wp,
            math.sin(heading_error),
            math.cos(heading_error),
            self.linear_vel,
            self.angular_vel,
            float(self.current_wp_index) / len(self.path)  # progresso no caminho
        ], dtype=np.float32)
        #print(self.steps)
        return state, np.clip(reward, -100.0, 100.0), done, {}



    def check_for_collision(self, x, y):
        # Convert continuous robot coordinates to grid coordinates
        # This assumes your grid coordinates correspond directly to meter units
        # You might need to adjust for scaling/offset if your grid represents a smaller area than actual Gazebo
        grid_x = int(round(x))
        grid_y = int(round(y))

        # Basic check: is the robot's current grid cell an obstacle?
        if 0 <= grid_x < len(self.grid) and 0 <= grid_y < len(self.grid[0]):
            return self.grid[grid_x][grid_y] == 1
        return False
    
    def reset(self):
        end = time.time()
        elapsed_seconds = end - self.start_time
        elapsed_minutes = elapsed_seconds / 60
        print(f"Passaram: {elapsed_minutes:.2f} minutos")
        self.current_wp_index = 0
        self.steps = 0
        wp = self.path[self.current_wp_index]
        self.reset_robot_pose()
        rclpy.spin_once(self.node, timeout_sec=0.5)
        time.sleep(0.2)

        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        distance_to_goal = np.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_to_goal - self.yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # Normaliza para [-pi, pi]

        self.distance_prev = np.linalg.norm(self.goal - np.array([self.x, self.y]))

        print(f"🔄 Reset - Start Pos: ({self.x:.2f}, {self.y:.2f}) | First goal: {self.goal}")
        return np.array([
            distance_to_goal,
            math.sin(heading_error),
            math.cos(heading_error),
            self.linear_vel,
            self.angular_vel,
            float(self.current_wp_index) / len(self.path) 
        ], dtype=np.float32)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


    def reset_robot_pose(self):
        state = EntityState()
        state.name = 'car_1'  # nome do modelo ou entidade
        state.pose.position.x = float(self.start_pos[0])
        state.pose.position.y = float(self.start_pos[1])
        state.pose.position.z = 0.01
        state.pose.orientation.w = 1.0  # manter orientação padrão
        state.twist = Twist()  # zero twist
        state.reference_frame = 'world'  # frame absoluto

        set_state_client = self.node.create_client(SetEntityState, '/set_entity_state')
        while not set_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Aguardando /set_entity_state...")

        req = SetEntityState.Request()
        req.state = state  # Note o campo 'state' aqui, não 'model_state'

        future = set_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)


_grid = [
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            ]

grid = _grid  # teu grid de obstáculos
start = (9,5)  # posição inicial (exemplo)
goal = (18, 5)  # destino final (exemplo)

if __name__ == "__main__":
    env = ROS2EnvWithPlanner(grid, start, goal)
    """model = PPO(
        "MlpPolicy",
        env,
        verbose=1
    )"""
    model = PPO.load("ppo_simple", env=env)
    model.learn(total_timesteps=100000)
    model.save("ppo_ros_3")
