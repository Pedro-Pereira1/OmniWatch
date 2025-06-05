import subprocess
import time
import signal
import sys

car_ids = [f"car{i+1}" for i in range(1)]
ros_processes = []

try:
    for car_id in car_ids:
        p = subprocess.Popen(["python3", "../ros2_ws/ros_car_node.py", car_id])
        ros_processes.append(p)
        time.sleep(0.5)

    # Wait for all processes
    for p in ros_processes:
        p.wait()

except KeyboardInterrupt:
    print("Caught CTRL+C, terminating subprocesses...")
    for p in ros_processes:
        p.terminate()  # or p.kill() if terminate doesn't work
    sys.exit(0)
