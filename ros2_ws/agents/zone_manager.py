from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import json
import asyncio


class ZoneManagerAgent(Agent):
    def __init__(self, jid, password, zone_id, zone_bounds):
        super().__init__(jid, password)
        self.zone_id = zone_id
        self.zone_bounds = zone_bounds
        self.known_cars_positions = {}  # car_jid: (x, y)

    def is_in_zone(self, position):
        x, y = position
        return (
            self.zone_bounds["x_min"] <= x <= self.zone_bounds["x_max"]
            and self.zone_bounds["y_min"] <= y <= self.zone_bounds["y_max"]
        )

    class ReceiveControlBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=5)
            if msg:
                try:
                    data = json.loads(msg.body)
                    command = data.get("command")
                    print(command)
                    if command == "threat":
                        await self.handle_threat(data.get("car_id"))
                    elif command == "plan_path":
                        await self.handle_client_request(data.get("goal"))
                    elif command == "handling_goal":
                        await self.handle_cars_response(data.get("car_id"))
                    elif command == "position_update":
                        await self.handle_position_update(data)
                    elif command == "request_ride":
                        start = data["start"]
                        end = data["end"]
                        await self.handle_ride_request(start, end)
                except Exception as e:
                    print(f"[{self.agent.zone_id}] Error processing message: {e}")
            else:
                await asyncio.sleep(1)

        
        async def handle_threat(self, malicious_car):
            print(f"[{self.agent.zone_id}] ⚠️ Threat received for {malicious_car}")

            pos = self.agent.known_cars_positions.get(malicious_car)
            if not pos:
                print(f"[{self.agent.zone_id}] No position known for {malicious_car}, ignoring threat.")
                return

            if not self.agent.is_in_zone(pos):
                print(f"[{self.agent.zone_id}] {malicious_car} is NOT in this zone. Ignoring.")
                return

            print(f"[{self.agent.zone_id}] 🚨 {malicious_car} is in this zone. Taking action.")

            cars_in_zone = [
                car_jid for car_jid, p in self.agent.known_cars_positions.items()
                if self.agent.is_in_zone(p)
            ]

            for car_jid in cars_in_zone:
                msg = Message(to=car_jid)
                if car_jid == malicious_car:
                    msg.body = json.dumps({"command": "stop"})
                    print(f"[{self.agent.zone_id}] 🔴 Sending STOP to {malicious_car}")
                else:
                    msg.body = json.dumps({"command": "quarantine", "state": True})
                    print(f"[{self.agent.zone_id}] 🟡 Sending QUARANTINE to {car_jid}")
                msg.set_metadata("performative", "inform")
                await self.send(msg)

        async def handle_client_request(self, goal):
            print(self.agent.known_cars_positions)
            cars = [
                car for car, _ in self.agent.known_cars_positions.items()
            ]
            print(f"[{self.agent.zone_id}] 🧭 Sending goal {goal} to cars in zone: {cars}")

            msg_body = json.dumps({
                "command": "plan_path_request",
                "goal": goal,
                "cars": cars
            })

            for car in cars:
                msg = Message(to=car)
                msg.body = msg_body
                msg.set_metadata("performative", "request")
                await self.send(msg)

        async def handle_cars_response(self, response):
            print(f"[{self.agent.zone_id}] ✅ Car {response} is handling the goal.")

        async def handle_position_update(self, data):
            car_id = data.get("car_id")
            pos = data.get("position", {})
            if "lat" in pos and "lon" in pos:
                self.agent.known_cars_positions[f"{car_id}@localhost"] = (pos["lat"], pos["lon"])
                print(f"[{self.agent.zone_id}] 📍 Updated {car_id} to ({pos['lat']}, {pos['lon']})")

        async def handle_ride_request(self, start, end):
            print(f"[{self.agent.zone_id}] 🚕 Ride request from {start} to {end}")
            cars = [
                car for car, _ in self.agent.known_cars_positions.items()
            ]

            msg_body = json.dumps({
                "command": "ride_request",
                "start": start,
                "end": end,
                "cars": cars
            })

            for car in cars:
                msg = Message(to=car)
                msg.body = msg_body
                msg.set_metadata("performative", "request")
                await self.send(msg)
            
    async def setup(self):
        print(f"[{self.zone_id}] 🛡️ Zone Manager Agent started.")
        self.add_behaviour(self.ReceiveControlBehaviour())


# Entry point
if __name__ == "__main__":
    import sys

    async def main():
        if len(sys.argv) != 6:
            print("Usage: python3 zone_manager.py <zone_id> <x_min> <x_max> <y_min> <y_max>")
            return

        zone_id = sys.argv[1]
        x_min = int(sys.argv[2])
        x_max = int(sys.argv[3])
        y_min = int(sys.argv[4])
        y_max = int(sys.argv[5])

        zone_bounds = {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
        }

        jid = f"{zone_id}@localhost"
        password = "pass"

        agent = ZoneManagerAgent(jid, password, zone_id, zone_bounds)
        await agent.start(auto_register=True)

        print(f"[{zone_id}] Managing bounds: {zone_bounds}")

        while agent.is_alive():
            await asyncio.sleep(1)

    asyncio.run(main())
