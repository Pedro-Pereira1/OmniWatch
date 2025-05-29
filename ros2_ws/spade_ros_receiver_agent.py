import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
from threading import Thread
import asyncio

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('ros_receiver_node_')
        self.publisher_ = self.create_publisher(String, 'spade_to_ros_topic', 10)

    def publish(self, msg_data):
        msg = String()
        msg.data = msg_data
        self.publisher_.publish(msg)
        self.get_logger().info(f"[ROS2] Published: {msg_data}")

def start_ros_node(agent):
    rclpy.init()
    agent.ros_node = ROSPublisher()
    rclpy.spin(agent.ros_node)

class ReceiverAgent(Agent):
    class ReceiveAndPublishBehaviour(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=10)
            if msg:
                print(f"[SPADE] Received: {msg.body}")
                self.agent.ros_node.publish(msg.body)
            else:
                print("No message received...")

    async def setup(self):
        print("ReceiverAgent with ROS2 Publisher started")
        self.ros_thread = Thread(target=start_ros_node, args=(self,))
        self.ros_thread.start()
        await asyncio.sleep(2)
        self.add_behaviour(self.ReceiveAndPublishBehaviour())

if __name__ == "__main__":
    async def main():
        agent = ReceiverAgent("receiver@localhost", "your_password")
        await agent.start(auto_register=True)
        while agent.is_alive():
            await asyncio.sleep(1)

    asyncio.run(main())