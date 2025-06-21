import subprocess
import time
import signal
import sys

car_ids = [f"car_{i+1}" for i in range(2)]
agent_processes = []
all_zones = [
    [(1.0, 1.0), (1.0, 9.0), (9.0, 9.0), (9.0, 1.0)],
    [(18.0, 18.0), (18.0, 9.0), (9.0, 9.0), (9.0, 18.0)]
]

try:
    for i, car_id in enumerate(car_ids):
        # Flatten patrol points for command line (e.g. 1.0 1.0 1.0 9.0 ...)
        patrol_points_args = [str(coord) for point in all_zones[i] for coord in point]
        
        # Launch agent with patrol points as arguments
        p = subprocess.Popen(["python3", "car_agent.py", car_id] + patrol_points_args)
        agent_processes.append(p)
        time.sleep(0.5)

    # Wait for all processes
    for p in agent_processes:
        p.wait()

except KeyboardInterrupt:
    print("Caught CTRL+C, terminating subprocesses...")
    for p in agent_processes:
        p.kill()
    sys.exit(0)
