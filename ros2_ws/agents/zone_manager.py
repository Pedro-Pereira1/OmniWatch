from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import json
import asyncio

class ZoneManagerAgent(Agent):
    def __init__(self, jid, password, zone_id, managed_cars):
        super().__init__(jid, password)
        self.zone_id = zone_id
        # Ensure full JIDs
        self.managed_cars = [car if "@" in car else f"{car}@localhost" for car in managed_cars]

    class ReceiveControlBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                try:
                    data = json.loads(msg.body)
                    print(f"[{self.agent.zone_id}] Received command from CityHall: {data}")
                    malicious_car = data["malicious_car"]

                    # Stop malicious car
                    stop_msg = Message(to=malicious_car)
                    stop_msg.body = json.dumps({"command": "stop"})
                    await self.send(stop_msg)
                    print(f"[{self.agent.zone_id}] Sent STOP to {malicious_car}")

                    # Quarantine other cars in zone
                    for car in self.agent.managed_cars:
                        if car != malicious_car:
                            q_msg = Message(to=car)
                            q_msg.body = json.dumps({"command": "quarantine", "state": True})
                            await self.send(q_msg)
                            print(f"[{self.agent.zone_id}] Sent QUARANTINE to {car}")
                except Exception as e:
                    print(f"[{self.agent.zone_id}] Error processing message: {e}")
            else:
                print(f"[{self.agent.zone_id}] Waiting for messages...")
                await asyncio.sleep(1)

    async def setup(self):
        print(f"[{self.zone_id}] Zone Manager Agent started.")
        self.add_behaviour(self.ReceiveControlBehaviour())

if __name__ == "__main__":
    import sys
    import asyncio

    async def main():
        if len(sys.argv) < 3:
            print("Usage: python3 zone_manager.py <zone_id> <car1> [<car2> ...]")
            return
        zone_id = sys.argv[1]
        cars = sys.argv[2:]
        jid = f"{zone_id}@localhost"
        password = "pass"  # Replace with your actual password if needed

        agent = ZoneManagerAgent(jid, password, zone_id, cars)
        await agent.start(auto_register=True)
        print(f"Agent {jid} started managing cars: {cars}")

        while agent.is_alive():
            await asyncio.sleep(1)

    asyncio.run(main())
