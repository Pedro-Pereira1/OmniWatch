import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from threading import Thread
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
import asyncio

class ROSListener(Node):
    def __init__(self):
        super().__init__('ros_listener')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10
        )
        self.latest_data = "No data yet"

    def listener_callback(self, msg):
        self.get_logger().info(f"Received from ROS2: {msg.data}")
        self.latest_data = msg.data

def start_ros_node(agent):
    rclpy.init()
    agent.ros_node = ROSListener()
    rclpy.spin(agent.ros_node)

class PublisherAgent(Agent):
    class PublishROSData(CyclicBehaviour):
        async def run(self):
            msg = self.agent.ros_node.latest_data
            print(f"[SPADE] Sending to receiver: {msg}")
            await self.send(self.agent.create_message(
                to="receiver@localhost",
                body=msg
            ))
            await asyncio.sleep(2)

    async def setup(self):
        print("PublisherAgent started")
        self.ros_thread = Thread(target=start_ros_node, args=(self,))
        self.ros_thread.start()
        await asyncio.sleep(2)
        self.add_behaviour(self.PublishROSData())

if __name__ == "__main__":
    async def main():
        agent = PublisherAgent("publisher@localhost", "your_password")
        await agent.start(auto_register=True)
        await agent.stop()

    asyncio.run(main())