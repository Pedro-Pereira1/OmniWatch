from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import asyncio
import json

class ReceiverAgent(Agent):
    class ReceiveBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=5)
            if msg:
                try:
                    data = json.loads(msg.body)
                    print(f"Received from {data['car_id']}: {data}")
                except Exception as e:
                    print("Failed to parse message:", e)
            else:
                print("Waiting for data...")

    async def setup(self):
        print("ReceiverAgent started.")
        self.add_behaviour(self.ReceiveBehaviour())

if __name__ == "__main__":
    async def main():
        agent = ReceiverAgent("main@localhost", "pass")
        await agent.start(auto_register=True)
        while agent.is_alive():
            await asyncio.sleep(1)
    asyncio.run(main())
