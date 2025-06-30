import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import math
from pettingzoo import AECEnv
from gymnasium import spaces
import supersuit as ss
from stable_baselines3 import PPO
from pettingzoo.utils.conversions import aec_to_parallel

# -----------------------------
# A* Search Algorithm
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
        _, _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            x, y = neighbor
            if 0 <= x < rows and 0 <= y < cols and grid[x][y] == 0:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
    return []

# -----------------------------
# Grid Map
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
# Helper Functions
# -----------------------------
def normalize_angle(angle):
    """Normalize angle to the range [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))

# -----------------------------
# RL Environment for Parameter Tuning
# -----------------------------
class ParameterTuningEnv(AECEnv):
    metadata = {
        "render_modes": ["human"],
        "name": "parameter_tuning_env_v0",
        "is_parallelizable": True
    }

    def __init__(self, render_mode="human"):
        super().__init__()
        self.agents = ["agent_0"]
        self.possible_agents = self.agents[:]
        self.agent_name_mapping = dict(zip(self.agents, list(range(len(self.agents)))))
        
        self.grid = np.array(original_grid)
        self.max_steps = 1500
        self.render_mode = render_mode
        
        # RL state variables
        self.agent_pos = (1.0, 1.0)
        self.agent_angle = 0.0
        self.current_step = 0
        self.path = []
        self.current_path_idx = 0

        # Action Space: [angle_thresh, pos_thresh, angular_gain, linear_speed]
        self.action_spaces = {
            agent: spaces.Box(
                low=np.array([0.05, 0.1, 1.0, 0.1]),  # Min values for parameters
                high=np.array([0.5, 0.5, 5.0, 1.0]), # Max values for parameters
                dtype=np.float32
            ) for agent in self.agents
        }

        # Observation Space: [x_pos, y_pos, angle, relative_target_x, relative_target_y]
        self.observation_spaces = {
            agent: spaces.Box(
                low=np.array([0, 0, -np.pi, -self.grid.shape[1], -self.grid.shape[0]]),
                high=np.array([self.grid.shape[1], self.grid.shape[0], np.pi, self.grid.shape[1], self.grid.shape[0]]),
                dtype=np.float32
            ) for agent in self.agents
        }
        
        # Visualization
        if self.render_mode == "human":
            self.fig, self.ax = plt.subplots(figsize=(8, 8))

    def observe(self, agent):
        return self._get_obs()

    def _get_obs(self):
        """Constructs the observation for the agent."""
        if not self.path or self.current_path_idx >= len(self.path):
            # If path is finished, relative target is (0, 0)
            relative_target_pos = (0, 0)
        else:
            target_cell = self.path[self.current_path_idx]
            target_pos = (target_cell[1] + 0.5, target_cell[0] + 0.5)
            relative_target_pos = (target_pos[0] - self.agent_pos[0], target_pos[1] - self.agent_pos[1])
        
        return np.array([
            self.agent_pos[0],
            self.agent_pos[1],
            self.agent_angle,
            relative_target_pos[0],
            relative_target_pos[1]
        ], dtype=np.float32)

    def reset(self, seed=None, options=None):
        self.agents = self.possible_agents[:]
        self._cumulative_rewards = {agent: 0 for agent in self.agents}
        self.rewards = {agent: 0 for agent in self.agents}
        self.terminations = {agent: False for agent in self.agents}
        self.truncations = {agent: False for agent in self.agents}
        self.infos = {agent: {} for agent in self.agents}
        self.agent_selection = self.agents[0]
        self.current_step = 0

        # Reset agent position to a random valid start
        while True:
            start_cell = (np.random.randint(0, self.grid.shape[0]), np.random.randint(0, self.grid.shape[1]))
            if self.grid[start_cell[0], start_cell[1]] == 0:
                self.agent_pos = (float(start_cell[1]) + 0.5, float(start_cell[0]) + 0.5)
                break
        
        self.agent_angle = np.random.uniform(-np.pi, np.pi)

        # Generate a new random goal and A* path
        while True:
            goal_cell = (np.random.randint(0, self.grid.shape[0]), np.random.randint(0, self.grid.shape[1]))
            start_cell = (int(self.agent_pos[1]), int(self.agent_pos[0]))
            if self.grid[goal_cell[0], goal_cell[1]] == 0 and goal_cell != start_cell:
                self.path = astar(self.grid, start_cell, goal_cell)
                if self.path:
                    break
        
        self.current_path_idx = 0
        self.observations = {agent: self._get_obs() for agent in self.agents}
        return self.observations[self.agents[0]], {} # Return for gymnasium compatibility


    def step(self, action):
        agent = self.agent_selection
        if self.terminations[agent] or self.truncations[agent]:
            self._was_dead_step(action)
            return

        # The RL agent's action is the set of controller parameters
        angle_threshold, pos_threshold, angular_gain, linear_speed_max = action

        reward = 0
        
        if not self.path or self.current_path_idx >= len(self.path):
            self.terminations[agent] = True
            reward = 0 # No penalty or reward if path is already done
        else:
            # --- Controller Logic using parameters from the action ---
            target_cell = self.path[self.current_path_idx]
            target_pos = (target_cell[1] + 0.5, target_cell[0] + 0.5)
            
            desired_angle = np.arctan2(target_pos[1] - self.agent_pos[1], target_pos[0] - self.agent_pos[0])
            angle_diff = normalize_angle(desired_angle - self.agent_angle)

            if abs(angle_diff) > angle_threshold:
                angular_speed = np.clip(angle_diff * angular_gain, -1.0, 1.0)
                linear_speed = 0.0
            else:
                angular_speed = 0.0
                linear_speed = linear_speed_max
            
            # Update agent's state
            self.agent_angle = normalize_angle(self.agent_angle + angular_speed * 0.1)
            dx = np.cos(self.agent_angle) * linear_speed * 0.2
            dy = np.sin(self.agent_angle) * linear_speed * 0.2
            new_pos = (self.agent_pos[0] + dx, self.agent_pos[1] + dy)

            # Check for collisions
            if not self._is_valid_position(new_pos):
                reward = -10.0
                self.terminations[agent] = True
            else:
                self.agent_pos = new_pos
                # Reward shaping
                reward = linear_speed * 0.05 - 0.01 # Encourage movement, penalize time
                
                # Check for arrival at the current waypoint
                dist_to_target = np.linalg.norm(np.array(self.agent_pos) - np.array(target_pos))
                if dist_to_target < pos_threshold:
                    reward += 10.0  # Big reward for reaching a waypoint
                    self.current_path_idx += 1

        # Check for episode completion
        if self.current_path_idx >= len(self.path) and self.path:
            self.terminations[agent] = True

        self.current_step += 1
        if self.current_step >= self.max_steps:
            self.truncations[agent] = True
            
        self.rewards[agent] = reward
        self._cumulative_rewards[agent] += reward
        self.observations[agent] = self._get_obs()
        
        if self.render_mode == "human":
            self.render()

        # Return for gymnasium compatibility
        return self.observations[agent], self.rewards[agent], self.terminations[agent], self.truncations[agent], {}

    def _is_valid_position(self, pos):
        x, y = int(pos[0]), int(pos[1])
        if not (0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]):
            return False
        return self.grid[y, x] == 0

    def render(self):
        if self.render_mode != "human":
            return
        self.ax.clear()
        for y in range(self.grid.shape[0]):
            for x in range(self.grid.shape[1]):
                color = "black" if self.grid[y, x] == 1 else "white"
                self.ax.add_patch(patches.Rectangle((x, y), 1, 1, facecolor=color, edgecolor="gray"))

        if self.path:
            path_coords = [(c[1] + 0.5, c[0] + 0.5) for c in self.path]
            xs, ys = zip(*path_coords)
            self.ax.plot(xs, ys, color='green', linestyle='--', alpha=0.5, label="A* Path")
            
            if self.current_path_idx < len(self.path):
                target_cell = self.path[self.current_path_idx]
                self.ax.add_patch(patches.Rectangle((target_cell[1], target_cell[0]), 1, 1, facecolor="yellow", alpha=0.4))

        agent_x, agent_y = self.agent_pos
        self.ax.add_patch(patches.Circle((agent_x, agent_y), radius=0.3, color='blue', label="Agent"))
        dx = np.cos(self.agent_angle) * 0.6
        dy = np.sin(self.agent_angle) * 0.6
        self.ax.arrow(agent_x, agent_y, dx, dy, head_width=0.2, head_length=0.2, fc='red', ec='red')

        self.ax.set_xlim(0, self.grid.shape[1])
        self.ax.set_ylim(self.grid.shape[0], 0)
        self.ax.set_aspect('equal')
        plt.pause(0.01)

    def close(self):
        if self.render_mode == "human":
            plt.close(self.fig)

# -----------------------------
# Main Execution
# -----------------------------
if __name__ == "__main__":
    # 1. Create and wrap the environment
    env = ParameterTuningEnv(render_mode="human")
    env = aec_to_parallel(env)
    env = ss.pettingzoo_env_to_vec_env_v1(env)
    env = ss.concat_vec_envs_v1(env, 1, num_cpus=1, base_class='stable_baselines3')

    # 2. Instantiate and train the PPO model
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log="./ppo_param_tuning_logs/")
    model.learn(total_timesteps=75000)
    model.save("ppo_parameter_tuner")
    print("\n--- TRAINING COMPLETE ---")

    # 3. Evaluate the trained model
    print("\n--- EVALUATION ---")
    del model
    model = PPO.load("ppo_parameter_tuner")

    obs, _ = env.reset()
    # Store the learned parameters for analysis
    learned_params = []

    for i in range(2000):
        action, _states = model.predict(obs, deterministic=True)
        
        # Store and print the learned parameters for the current step
        learned_params.append(action[0])
        if i % 100 == 0: # Print every 100 steps
             print(f"Step {i}: Learned Params (Action) -> "
                  f"Angle Thresh: {action[0][0]:.3f}, "
                  f"Pos Thresh: {action[0][1]:.3f}, "
                  f"Angular Gain: {action[0][2]:.3f}, "
                  f"Linear Speed: {action[0][3]:.3f}")

        obs, rewards, dones, info = env.step(action)
        
        if dones.any():
            print("Episode finished. Resetting environment.")
            obs, _ = env.reset()
    
    env.close()

    # Analyze the distribution of learned parameters
    if learned_params:
        params_array = np.array(learned_params)
        mean_params = np.mean(params_array, axis=0)
        print("\n--- AVERAGE LEARNED PARAMETERS ---")
        print(f"Average Angle Threshold: {mean_params[0]:.4f}")
        print(f"Average Position Threshold: {mean_params[1]:.4f}")
        print(f"Average Angular Gain: {mean_params[2]:.4f}")
        print(f"Average Linear Speed: {mean_params[3]:.4f}")