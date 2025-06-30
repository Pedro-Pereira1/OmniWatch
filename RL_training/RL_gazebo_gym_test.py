from stable_baselines3 import PPO
from RL_gazebo_treino import Simple2DRLEnv, _grid, start, goal  # Certifica que usas o mesmo ambiente

env = Simple2DRLEnv(_grid, start, goal)
model = PPO.load("ppo_simple_random", env=env)

obs = env.reset()
done = False
total_reward = 0
step_count = 0

while not done:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, _ = env.step(action)
    total_reward += reward
    step_count += 1
    env.render()  # Isto mostra a posição atual

print(f"✅ Done in {step_count} steps with total reward {total_reward:.2f}")
env.close()
