import subprocess
import time
import signal
import sys

car_ids = [f"car{i+1}" for i in range(1)]
agent_processes = []

try:
    for car_id in car_ids:
        p = subprocess.Popen(["python3", "car_agent.py", car_id])
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
