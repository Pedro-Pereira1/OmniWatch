import subprocess
import time
import sys

zone_car_map = {
    "zone1": ["car1", "car2"],
    # "zone2": ["car3", "car4"],
    # "zone3": ["car5"],
    # "zone4": ["car6"]
}

zone_ids = list(zone_car_map.keys())
zone_processes = []

try:
    for zone_id in zone_ids:
        cars = zone_car_map[zone_id]
        print(f"Starting Zone Manager for {zone_id} with cars {cars}")
        p = subprocess.Popen(["python3", "zone_manager.py", zone_id] + cars)
        zone_processes.append(p)
        time.sleep(0.5)

    for p in zone_processes:
        p.wait()

except KeyboardInterrupt:
    print("Caught CTRL+C, terminating zone manager subprocesses...")
    for p in zone_processes:
        p.kill()
    sys.exit(0)
