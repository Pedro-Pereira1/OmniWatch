from stable_baselines3 import PPO
import numpy as np
from RL_gazebo_env import ROS2EnvWithPlanner  # <- ou o nome certo onde está tua classe

# mesmo grid e posições
from RL_gazebo_env import grid, start, goal

env = ROS2EnvWithPlanner(grid, start, goal)
#model = PPO("MlpPolicy", env, verbose=1)
model = PPO.load("ppo_simple_random", env=env)

obs = env.reset()
print(f"🟢 Starting test at position ({env.x:.2f}, {env.y:.2f})")

done = False
step_count = 0

while not done:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, _ = env.step(action)
    step_count += 1
    print(f"Step {step_count:02d}: pos=({env.x:.2f}, {env.y:.2f}) → goal={env.goal} | reward={reward:.2f}")

print("✅ Final goal reached.")
env.close()
