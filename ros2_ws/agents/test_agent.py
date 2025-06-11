import asyncio
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import json

class TestAgent(Agent):
    class SendGoalBehaviour(CyclicBehaviour):
        async def run(self):
            goal = [10.0, 15.0]
            data = {
                "command": "plan_path",
                "goal": goal
            }
            msg = Message(to="car_1@localhost")
            msg.body = json.dumps(data)
            msg.set_metadata("performative", "request")
            await self.send(msg)
            print("Mensagem enviada ao agente do carro.")
            await asyncio.sleep(5)  # Espera 5s antes de enviar novamente

    async def setup(self):
        self.add_behaviour(self.SendGoalBehaviour())

async def main():
    agent = TestAgent("test@localhost", "pass")
    await agent.start(auto_register=True)

    # Mantém o agente ativo até Ctrl+C
    try:
        await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("Ctrl+C recebido. A terminar...")
    await agent.stop()

if __name__ == "__main__":
    asyncio.run(main())
