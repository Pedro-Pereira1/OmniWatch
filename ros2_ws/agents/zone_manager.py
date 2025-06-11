from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import json
import asyncio

class ZoneManagerAgent(Agent):
    def __init__(self, jid, password, zone_id, managed_cars):
        super().__init__(jid, password)
        self.zone_id = zone_id
        self.managed_cars = [car if "@" in car else f"{car}@localhost" for car in managed_cars]
        self.waiting_for_response = False
        self.client_jid = None
        self.pending_request = None

    class ReceiveControlBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=5)
            #print(msg)
            if msg:
                try:
                    data = json.loads(msg.body)
                    #print(data)
                    if data.get("command") == "threat":
                        await self.handle_threat(data.get("car_id"))
                    elif data.get("command") == "plan_path":
                        await self.handle_client_request(data.get("goal"))
                    elif data.get("command") == "handling_goal":
                        await self.handle_cars_responde(data.get("car_id"))
                except Exception as e:
                    print(f"[{self.agent.zone_id}] Error processing message: {e}")
            else:
                await asyncio.sleep(1)

        async def handle_threat(self, malicious_car):
            print(f"[{self.agent.zone_id}] Threat detected: {malicious_car}")
            #for car in self.agent.managed_cars:
            #    msg = Message(to=car)
            #    if car == malicious_car:
            #        msg.body = json.dumps({"command": "stop"})
            #        print(f"[{self.agent.zone_id}] Sent STOP to {car}")
            #    else:
            #        msg.body = json.dumps({"command": "quarantine", "state": True})
            #        print(f"[{self.agent.zone_id}] Sent QUARANTINE to {car}")
            #    await self.send(msg)

        async def handle_client_request(self, goal):
            print(f"[{self.agent.zone_id}] 🧭 Forwarded goal {goal} to cars: {self.agent.managed_cars}")
            msg_body = json.dumps({
                "command": "plan_path_request",
                "goal": goal,
                "cars": self.agent.managed_cars
            })

            for car in self.agent.managed_cars:
                msg = Message(to=car)
                msg.body = msg_body
                msg.set_metadata("performative", "request")
                await self.send(msg)

        async def handle_cars_responde(self, response):
            print(f"[{self.agent.zone_id}] Received response from car: {response} - I'm going")


    async def setup(self):
        print(f"[{self.zone_id}] Zone Manager Agent started.")
        self.add_behaviour(self.ReceiveControlBehaviour())


# Entry point
if __name__ == "__main__":
    import sys

    async def main():
        if len(sys.argv) < 3:
            print("Usage: python3 zone_manager.py <zone_id> <car1> [<car2> ...]")
            return

        zone_id = sys.argv[1]
        cars = sys.argv[2:]
        jid = f"{zone_id}@localhost"
        password = "pass"

        agent = ZoneManagerAgent(jid, password, zone_id, cars)
        await agent.start(auto_register=True)
        print(f"[{zone_id}] Managing cars: {cars}")

        while agent.is_alive():
            await asyncio.sleep(1)

    asyncio.run(main())
