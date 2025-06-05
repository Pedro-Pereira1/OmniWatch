import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg, Float32
from geometry_msgs.msg import Twist
from threading import Thread
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import asyncio
import json
import sys

class ROSCarListener(Node):
    def __init__(self, car_id):
        super().__init__(f'{car_id}_listener')
        prefix = f'{car_id}/'

        self.log_data = "No logs yet"
        self.weight_data = 0.0
        self.position_data = (0.0, 0.0)

        self.create_subscription(StringMsg, prefix + 'logs', self.log_callback, 10)
        self.create_subscription(Float32, prefix + 'weight', self.weight_callback, 10)
        self.create_subscription(Twist, prefix + 'position', self.position_callback, 10)

    def log_callback(self, msg):
        self.log_data = msg.data

    def weight_callback(self, msg):
        self.weight_data = msg.data

    def position_callback(self, msg: Twist):
        self.position_data = (msg.linear.x, msg.linear.y)

def start_ros_node(agent, car_id):
    rclpy.init()
    agent.ros_node = ROSCarListener(car_id)
    try:
        rclpy.spin(agent.ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        agent.ros_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"[{car_id}] Shutdown warning: {e}")

class CarAgent(Agent):
    def __init__(self, jid, password, car_id):
        super().__init__(jid, password)
        self.car_id = car_id
        self.quarantine = False
        self._stop_requested = False

    class SendDataBehaviour(CyclicBehaviour):
        async def run(self):
            if self.agent._stop_requested:
                print(f"[{self.agent.car_id}] STOP requested, halting data transmission.")
                await asyncio.sleep(50)
            else:
                data = {
                    "car_id": self.agent.car_id,
                    "log": self.agent.ros_node.log_data,
                    "weight": self.agent.ros_node.weight_data,
                    "position": {
                        "lat": self.agent.ros_node.position_data[0],
                        "lon": self.agent.ros_node.position_data[1]
                    },
                    "quarantine": self.agent.quarantine
                }

                msg = Message(
                    to="main@localhost",
                    body=json.dumps(data),
                    metadata={"performative": "inform"}
                )
                print(f"[{self.agent.car_id}] Sending: {msg.body}")
                await self.send(msg)

                await asyncio.sleep(10 if self.agent.quarantine else 5)

    class ControlBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=5)
            if msg:
                try:
                    data = json.loads(msg.body)
                    cmd = data.get("command")
                    if cmd == "stop":
                        print(f"[{self.agent.car_id}] Received STOP command.")
                        self.agent._stop_requested = True
                    elif cmd == "quarantine":
                        print(f"[{self.agent.car_id}] Quarantine set to: {self.agent.quarantine}")
                        self.agent.quarantine = data.get("state", True)
                except Exception as e:
                    print(f"[{self.agent.car_id}] Error handling control message: {e}")

    async def setup(self):
        print(f"[{self.car_id}] Agent started")
        thread = Thread(target=start_ros_node, args=(self, self.car_id), daemon=True)
        thread.start()
        await asyncio.sleep(2)
        self.add_behaviour(self.SendDataBehaviour())
        self.add_behaviour(self.ControlBehaviour())


if __name__ == "__main__":
    async def main():
        car_id = sys.argv[1]
        agent = CarAgent(f"{car_id}@localhost", "pass", car_id)
        await agent.start(auto_register=True)
        while agent.is_alive():
            await asyncio.sleep(1)
    asyncio.run(main())
