from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
import asyncio
import json
import pandas as pd
import joblib
import numpy as np
from sklearn.preprocessing import LabelEncoder
from io import StringIO

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

                    sample_df = pd.read_json(StringIO(log), typ='frame')
                    sample_df = sample_df[self.agent.model.get_booster().feature_names]
                    probs = self.agent.model.predict_proba(sample_df)[0]
                    pred_index = probs.argmax()
                    pred_class = self.agent.le.inverse_transform([pred_index])[0]
                    print(f"[CityHall] Prediction for {car_id}: {pred_class}")

                    if not "benign" in pred_class.lower():
                        zone_jid = self.zone_map.get(car_id)
                        if zone_jid:
                            print(f"[CityHall] 🚨 Detected malicious log from {car_id}. Notifying {zone_jid}")
                            msg = Message(to=zone_jid)
                            data = {
                                "command": "threat",
                                "car_id": car_id,
                            }
                            msg.body = json.dumps(data)
                            msg.set_metadata("performative", "request")
                            await self.send(alert)
                        else:
                            print(f"[CityHall] ⚠️ No zone manager found for {car_id}")
                except Exception as e:
                    print("[CityHall] Failed to parse or process message:", e)
            else:
                print("[CityHall] Waiting for car data...")

    async def setup(self):
        df = pd.read_parquet("models/cic-collection_reduzido.parquet")
        self.X = df.drop(columns=["Label", "ClassLabel"])
        self.y_label = df["Label"]
        self.le = LabelEncoder()
        self.le.fit(self.y_label)
        self.model = joblib.load("models/xgb_model.joblib")

        self.add_behaviour(self.MonitorCarsBehaviour())
        print("[CityHall] Agent started.")

if __name__ == "__main__":
    async def main():
        agent = CityHallAgent("main@localhost", "pass")
        await agent.start(auto_register=True)
        while agent.is_alive():
            await asyncio.sleep(1)
    asyncio.run(main())
