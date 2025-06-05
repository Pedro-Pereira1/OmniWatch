from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import asyncio
import json

class CityHallAgent(Agent):
    class MonitorCarsBehaviour(CyclicBehaviour):
        zone_map = {
            "car1": "zone1@localhost",
            "car2": "zone1@localhost",
            "car3": "zone2@localhost",
            "car4": "zone2@localhost",
            "car5": "zone3@localhost",
            "car6": "zone4@localhost"
        }

        async def run(self):
            msg = await self.receive(timeout=5)
            if msg:
                try:
                    data = json.loads(msg.body)
                    car_id = data["car_id"]
                    log = data.get("log", "")
                    print(f"[CityHall] Received from {car_id}: {log}")
                    
                    if "malicious" in log.lower():
                        zone_jid = self.zone_map.get(car_id)
                        if zone_jid:
                            print(f"[CityHall] 🚨 Detected malicious log from {car_id}. Notifying {zone_jid}")
                            alert = Message(to=zone_jid)
                            alert.body = json.dumps({"malicious_car": f"{car_id}@localhost"})
                            await self.send(alert)
                        else:
                            print(f"[CityHall] ⚠️ No zone manager found for {car_id}")
                except Exception as e:
                    print("[CityHall] Failed to parse or process message:", e)
            else:
                print("[CityHall] Waiting for car data...")

    async def setup(self):
        print("[CityHall] Agent started.")
        self.add_behaviour(self.MonitorCarsBehaviour())

if __name__ == "__main__":
    async def main():
        agent = CityHallAgent("main@localhost", "pass")
        await agent.start(auto_register=True)
        while agent.is_alive():
            await asyncio.sleep(1)
    asyncio.run(main())
