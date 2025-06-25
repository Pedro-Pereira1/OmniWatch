import subprocess
import time
import sys

# Define the four zones
zone_configs = {
    "zone_1": [0, 9, 0, 9], # Amarelo
    "zone_2": [10, 19, 0, 9], # Azul
    "zone_3": [0, 9, 10, 19], # Vermelho
    "zone_4": [10, 19, 10, 19], # Preto
}

zone_processes = []

try:
    for zone_id, bounds in zone_configs.items():
        print(f"🚀 Starting Zone Manager for {zone_id} with bounds {bounds}")
        p = subprocess.Popen(["python3", "zone_manager.py", zone_id] + list(map(str, bounds)))
        zone_processes.append(p)
        time.sleep(5)

    for p in zone_processes:
        p.wait()

except KeyboardInterrupt:
    print("🛑 Caught CTRL+C, terminating zone managers...")
    for p in zone_processes:
        p.kill()
    sys.exit(0)
