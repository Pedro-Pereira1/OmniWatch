import asyncio
from spade.agent import Agent
from spade.behaviour import OneShotBehaviour
from spade.message import Message
import json

class ClientAgent(Agent):
    class UserInputBehaviour(OneShotBehaviour):
        async def run(self):
            print("=== Client Agent ===")
            try:
                x = 1.0
                y = 1.0
                zone = "zone1"  # Hard7oded zone for simplicity

                zone_jid = f"{zone}@localhost"
                data = {
                    "command": "plan_path",
                    "goal": [x, y]
                }

                msg = Message(to=zone_jid)
                msg.body = json.dumps(data)
                msg.set_metadata("performative", "request")

                await self.send(msg)
                print(f"📤 Sent goal {data['goal']} to {zone_jid}")
            except Exception as e:
                print(f"⚠️ Error collecting or sending input: {e}")

            await self.agent.stop()  # Exit after sending

    async def setup(self):
        print("🟢 ClientAgent started.")
        self.add_behaviour(self.UserInputBehaviour())

# Entry point
if __name__ == "__main__":
    import sys
    import getpass

    async def main():
        # Login with username/pass or hardcoded credentials
        jid = "client@localhost"
        password = "pass"

        client = ClientAgent(jid, password)
        await client.start(auto_register=True)
        while client.is_alive():
            await asyncio.sleep(1)

    asyncio.run(main())
