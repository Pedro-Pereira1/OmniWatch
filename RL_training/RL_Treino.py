import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pandas as pd
from pettingzoo import AECEnv
from gymnasium import spaces
import supersuit as ss
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from pettingzoo.utils.conversions import aec_to_parallel
import heapq
import math
import time

# -----------------------------
# A * Search Algorithm
# -----------------------------
def heuristic(a, b):
    """Manhattan distance heuristic"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    
    came_from = {}
    g_score = {start: 0}
    
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Reverse path

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            x, y = neighbor
            if 0 <= x < rows and 0 <= y < cols and grid[x][y] == 0:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))

    return []  # No path found

def is_valid_goal(goal, grid):
    y, x = goal
    # Check bounds
    if y < 0 or y >= len(grid) or x < 0 or x >= len(grid[0]):
        return False

    # Check if cell is not an obstacle (0 = free, 1 = obstacle)
    if grid[y][x] == 1:
        return False

    return True
# -----------------------------
# Grid map
# -----------------------------
original_grid = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
]

# -----------------------------
# RL Environment definition
# -----------------------------
class RealTimeEnv(AECEnv):
    metadata = {
        "render_modes": ["human"],
        "name": "real_time_env",
        "is_parallelizable": True
    }

    def __init__(self):
        super().__init__()
        self.agents = ["agent_0",]
        self.possible_agents = self.agents[:]
        self.agent_name_mapping = dict(zip(self.agents, list(range(len(self.agents)))))
        self.render_mode = "human"
        self.grid = np.array(original_grid)
        #self.agent_pos = (1.0, 1.0)
        original_grid_np = np.array(original_grid)
        while True:
            goal = (np.random.randint(10, 19), np.random.randint(10, 19))
            if is_valid_goal(goal, original_grid_np):
                self.agent_pos = (goal[1], goal[0])
                break   
        self.agent_angle = 0.0
        self.max_steps = 2000
        self.current_step = 0
        self.path = []           # List of (row, col) grid points from A*
        self.current_path_idx = 0  # Index of the current target point in path
        self.cell_size = 1.0       # size of one grid cell (assuming 1x1)


        # Action space: [angular_speed, linear_speed]
        self.action_spaces = {
            agent: spaces.Box(
                low=np.array([-1.0, 0.0]),
                high=np.array([1.0, 1.0]),
                dtype=np.float32
            ) for agent in self.agents
        }

        # Observation space: [x_pos, y_pos, angle]
        self.observation_spaces = {
            agent: spaces.Box(
                low=np.array([0, 0, -np.pi]),
                high=np.array([self.grid.shape[1], self.grid.shape[0], np.pi]),
                dtype=np.float32
            ) for agent in self.agents
        }

        self.fig, self.ax = plt.subplots()

    def reset(self, seed=None, options=None):
        self.agents = self.possible_agents[:]
        self._cumulative_rewards = {agent: 0.0 for agent in self.agents}
        self.rewards = {agent: 0.0 for agent in self.agents}
        self.terminations = {agent: False for agent in self.agents}
        self.truncations = {agent: False for agent in self.agents}
        self.infos = {agent: {} for agent in self.agents}
        self.agent_selection = self.agents[0]
        self.current_step = 0

        # Reset agent state
        #self.agent_pos = (1.0, 1.0)
        original_grid_np = np.array(original_grid)
        while True:
            goal = (np.random.randint(10, 19), np.random.randint(10, 19))
            if is_valid_goal(goal, original_grid_np):
                self.agent_pos = (goal[1], goal[0])
                break 
        self.agent_angle = np.random.uniform(-np.pi, np.pi)
        self.observations = {
            agent: self._get_obs() for agent in self.agents
        }
        self.path = []  # clear old path
        self.current_path_idx = 0
        return self.observations

    def _get_obs(self):
        return np.array([self.agent_pos[0], self.agent_pos[1], self.agent_angle], dtype=np.float32)

    def step(self, action=None):
        agent = self.agent_selection
        if self.terminations[agent] or self.truncations[agent]:
            self._was_done_step(action)
            return

        # If no path or finished path, just stay put or reset
        if not self.path or self.current_path_idx >= len(self.path):
            self.terminations[agent] = True
            self.rewards[agent] = 0
            self.observations[agent] = self._get_obs()
            if self.render_mode == "human":
                self.render()
            return

        # Current target point in the path
        target_cell = self.path[self.current_path_idx]  # (row, col)
        #print(f"Current target cell: {target_cell}")

        # Convert grid cell to continuous position in your coordinate system (center of the cell)
        target_pos = (target_cell[1] + 0.5, target_cell[0] + 0.5)  # (x, y)

        # Current agent position
        agent_x, agent_y = self.agent_pos

        # Calculate angle to target point
        desired_angle = np.arctan2(target_pos[1] - agent_y, target_pos[0] - agent_x)

        # Angle difference (normalize between -pi and pi)
        angle_diff = desired_angle - self.agent_angle
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

        # Thresholds
        angle_threshold = 0.3   # radians, how close agent angle must be to move forward
        pos_threshold = 0.15    # distance threshold to consider "arrived" at target cell

        # Decide action based on angle difference
        if abs(angle_diff) > angle_threshold:
            # Rotate towards target
            angular_speed = np.clip(angle_diff * 2, -1.0, 1.0)  # proportional controller for rotation
            linear_speed = 0.0
        else:
            # Move forward
            angular_speed = 0.0
            linear_speed = 0.1

        # Perform movement calculation here (same as your previous logic)
        self.agent_angle += angular_speed * 0.1
        self.agent_angle = np.arctan2(np.sin(self.agent_angle), np.cos(self.agent_angle))

        dx = np.cos(self.agent_angle) * linear_speed
        dy = np.sin(self.agent_angle) * linear_speed
        new_pos = (agent_x + dx, agent_y + dy)

        # Collision detection
        if not self._is_valid_position(new_pos):
            self.rewards[agent] = -10
            self.terminations[agent] = True
        else:
            self.agent_pos = new_pos
            self.rewards[agent] = linear_speed

        # Check if arrived at current target cell
        dist_to_target = np.linalg.norm(np.array(new_pos) - np.array(target_pos))
        if dist_to_target < pos_threshold:
            self.current_path_idx += 1  # Move to next target point

        self.current_step += 1
        if self.current_step >= self.max_steps:
            self.truncations[agent] = True

        self._cumulative_rewards[agent] += self.rewards[agent]
        self.observations[agent] = self._get_obs()

        if self.render_mode == "human":
            self.render()


    def _is_valid_position(self, pos):
        x, y = int(pos[0]), int(pos[1])
        if x < 0 or x >= self.grid.shape[1] or y < 0 or y >= self.grid.shape[0]:
            return False
        return self.grid[y, x] == 0

    def render(self):
        self.ax.clear()

        # Draw grid cells as white or black blocks depending on obstacle
        for y in range(self.grid.shape[0]):
            for x in range(self.grid.shape[1]):
                if self.grid[y, x] == 1:
                    # Obstacle cell: draw as black
                    self.ax.add_patch(
                        patches.Rectangle((x, y), 1, 1, facecolor="black")
                    )
                else:
                    # Free cell: draw as white with grid lines
                    self.ax.add_patch(
                        patches.Rectangle((x, y), 1, 1, facecolor="white", edgecolor="gray", linewidth=0.5)
                    )

        # ✅ Highlight current target cell in the path
        if self.path and self.current_path_idx < len(self.path):
            target_cell = self.path[self.current_path_idx]
            self.ax.add_patch(
                patches.Rectangle((target_cell[1], target_cell[0]), 1, 1, facecolor="yellow", alpha=0.4)
            )
        if self.path:
            path_coords = [(c[1] + 0.5, c[0] + 0.5) for c in self.path]
            xs, ys = zip(*path_coords)
            self.ax.plot(xs, ys, color='green', linestyle='--', alpha=0.5)


        # Agent position and orientation
        agent_x, agent_y = self.agent_pos
        self.ax.add_patch(
            patches.Circle((agent_x, agent_y), radius=0.3, color='blue')
        )

        # Draw heading arrow from agent center
        dx = np.cos(self.agent_angle) * 0.6
        dy = np.sin(self.agent_angle) * 0.6
        self.ax.arrow(agent_x, agent_y, dx, dy, head_width=0.2, head_length=0.2, fc='red', ec='red')

        # Draw grid lines for better clarity
        self.ax.set_xticks(np.arange(0, self.grid.shape[1] + 1, 1))
        self.ax.set_yticks(np.arange(0, self.grid.shape[0] + 1, 1))
        self.ax.grid(True, which='both', color='gray', linewidth=0.5, linestyle='-')

        # Invert y axis so row 0 is top row (optional)
        self.ax.invert_yaxis()

        # Set axis limits and aspect ratio
        self.ax.set_xlim(0, self.grid.shape[1])
        self.ax.set_ylim(0, self.grid.shape[0])
        self.ax.set_aspect('equal')

        plt.pause(0.01)


    def close(self):
        plt.close(self.fig)

# -----------------------------
# Main execution
# -----------------------------
def angle_diff(a, b):
    """Smallest signed angle difference (a - b) normalized to [-pi, pi]."""
    diff = a - b
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle    


if __name__ == "__main__":
    env = RealTimeEnv()
    
    # Reset environment
    env.reset()

    # Define start and goal grid cells (row, col)
    start = (int(env.agent_pos[1]), int(env.agent_pos[0]))  # convert current pos to grid coords (row, col)
    #goal = (18, 18)  # example target cell on grid
    original_grid_np = np.array(original_grid)
    while True:
        goal = (np.random.randint(10, 19), np.random.randint(10, 19))
        if is_valid_goal(goal, original_grid_np):
            break

    # Get A* path (list of (row, col) tuples)
    path = astar(env.grid, start, goal)
    if not path:
        print("No path found!")
    else:
        print("Path found:", path)

    env.path = path
    env.current_path_idx = 0

    try:
        for _ in range(1500):
            env.step(None)  # Passing None because step logic now decides action internally
            
            if all(env.terminations.values()) or all(env.truncations.values()):
                print("Episode ended. Resetting environment...")
                #print("1", all(env.terminations.values()))
                #print("2", all(env.truncations.values()))
                env.reset()
                env.path = astar(env.grid, (int(env.agent_pos[1]), int(env.agent_pos[0])), goal)
                env.current_path_idx = 0

    finally:
        env.close()

