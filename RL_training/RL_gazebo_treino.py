import gym
from gym import spaces
import numpy as np
import math
import heapq
import time
from stable_baselines3 import PPO
import random


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
            return data[::-1]
        close_set.add(current)
        for i,j in neighbors:
            neighbor = current[0]+i, current[1]+j
            if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]):
                if grid[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
            tentative_g_score = gscore[current] + 1
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return []

def get_random_free_position(grid):
    while True:
        x = random.randint(0, len(grid) - 1)
        y = random.randint(0, len(grid[0]) - 1)
        if grid[x][y] == 0:
            return (x, y)
        
class Simple2DRLEnv(gym.Env):
    def __init__(self, grid, start, goal):
        super().__init__()
        self.grid = grid
        self.start_pos = np.array(start, dtype=np.float32)
        self.goal_pos = np.array(goal, dtype=np.float32)
        self.path = a_star_path(self.grid, start, goal)[1:]

        if not self.path:
            raise ValueError("No path found")

        self.current_wp_index = 0
        self.position = self.start_pos.copy()
        self.yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.max_steps = 700
        self.steps = 0

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([0.0, -0.4]), high=np.array([0.6, 0.4]), dtype=np.float32)

    def reset(self):
        while True:
            self.start_pos = np.array(get_random_free_position(self.grid), dtype=np.float32)
            self.goal_pos = np.array(get_random_free_position(self.grid), dtype=np.float32)
            if not np.array_equal(self.start_pos, self.goal_pos):
                self.path = a_star_path(self.grid, tuple(self.start_pos.astype(int)), tuple(self.goal_pos.astype(int)))
                if self.path:  # path exists
                    break
        self.current_wp_index = 0
        self.position = self.start_pos.copy()
        self.yaw = 0.0
        self.steps = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.goal = np.array(self.path[self.current_wp_index], dtype=np.float32)
        self.distance_prev = np.linalg.norm(self.goal - self.position)
        return self._get_obs()

    def _get_obs(self):
        dx = self.goal[0] - self.position[0]
        dy = self.goal[1] - self.position[1]
        distance = np.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_to_goal - self.yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        return np.array([
            distance,
            math.sin(heading_error),
            math.cos(heading_error),
            self.linear_vel,
            self.angular_vel,
            float(self.current_wp_index) / len(self.path)
        ], dtype=np.float32)

    def step(self, action):
        lin = float(action[0]) * 0.6
        ang = float(action[1]) * 1.0
        self.linear_vel = lin
        self.angular_vel = ang

        dt = 0.1
        self.yaw += ang * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        dx = lin * math.cos(self.yaw) * dt
        dy = lin * math.sin(self.yaw) * dt
        self.position += np.array([dx, dy])

        reward = 0.0
        done = False

        goal_dx = self.goal[0] - self.position[0]
        goal_dy = self.goal[1] - self.position[1]
        dist_to_wp = np.hypot(goal_dx, goal_dy)

        progress = self.distance_prev - dist_to_wp
        reward += 15.0 * progress

        angle_to_goal = math.atan2(goal_dy, goal_dx)
        heading_error = math.atan2(math.sin(angle_to_goal - self.yaw), math.cos(angle_to_goal - self.yaw))
        reward += 2.0 * math.cos(heading_error)

        reward -= 0.5 * abs(ang)
        if lin < 0.05:
            reward -= 0.8

        reward -= 0.05  # penalização constante por passo

        grid_x = int(round(self.position[0]))
        grid_y = int(round(self.position[1]))
        if 0 <= grid_x < len(self.grid) and 0 <= grid_y < len(self.grid[0]):
            if self.grid[grid_x][grid_y] == 1:
                reward -= 100.0
                done = True

        if dist_to_wp < 0.3:
            reward += 25.0
            self.current_wp_index += 1
            if self.current_wp_index >= len(self.path):
                reward += 300.0
                done = True
            else:
                self.goal = np.array(self.path[self.current_wp_index], dtype=np.float32)

        self.distance_prev = dist_to_wp
        self.steps += 1
        if self.steps >= self.max_steps:
            reward -= 50.0 + 10.0 * dist_to_wp
            done = True

        return self._get_obs(), np.clip(reward, -100.0, 100.0), done, {}

    def render(self, mode='human'):
        print(f"Pos: {self.position}, Goal: {self.goal}")

    def close(self):
        pass


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

start = (9,5)
goal = (18,5)
if __name__ == "__main__":
    grid = _grid  # Usa o mesmo grid do teu código anterior
    env = Simple2DRLEnv(grid, (0, 0), (1, 0))
    model = PPO("MlpPolicy", env, verbose=1)
    #model = PPO.load("ppo_simple", env=env)
    model.learn(total_timesteps=75000)
    model.save("ppo_simple_random")


