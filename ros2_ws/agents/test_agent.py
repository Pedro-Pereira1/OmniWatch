import asyncio
from spade.agent import Agent
from spade.behaviour import OneShotBehaviour
from spade.message import Message
import json

class TestAgent(Agent):
    class SendGoalBehaviour(OneShotBehaviour):
        async def run(self):
            goal = [18.0, 18.0]
            data = {
                "command": "plan_path",
                "goal": goal
            }
            msg = Message(to="car_1@localhost")
            msg.body = json.dumps(data)
            msg.set_metadata("performative", "request")
            await self.send(msg)
            print("Mensagem enviada ao agente do carro.")
            await asyncio.sleep(1)
            await self.agent.stop()

    async def setup(self):
        self.add_behaviour(self.SendGoalBehaviour())

async def main():
    agent = TestAgent("test@localhost", "pass")
    await agent.start(auto_register=True)
    await asyncio.sleep(3)
    await agent.stop()

if __name__ == "__main__":
    asyncio.run(main())
