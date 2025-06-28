from spade.agent import Agent
from spade.behaviour import CyclicBehaviour, PeriodicBehaviour
from spade.message import Message
import asyncio
import json
import pandas as pd
import joblib
from sklearn.preprocessing import LabelEncoder
from io import StringIO
from datetime import datetime


class CityHallAgent(Agent):
    class MonitorCarsBehaviour(PeriodicBehaviour):
        zone_managers = [
            "zone_1@localhost",
            "zone_2@localhost",
            "zone_3@localhost",
            "zone_4@localhost"
        ]

        async def run(self):
            """
            print("A infetar o carro 1...")
            for zone_jid in self.zone_managers:
                threat_msg = Message(to=zone_jid)
                threat_msg.body = json.dumps({
                    "command": "threat",
                    "car_id": "car_1@localhost"
                    })
                threat_msg.set_metadata("performative", "request")
                await self.send(threat_msg)
            """
            msg = await self.receive()
            if msg:
                try:
                    data = json.loads(msg.body)
                    car_jid = data["car_id"]
                    log = data.get("log", "")
                    print(f"[CityHall] 📩 Received log from {car_jid}")
                    
                    
                    # Parse log data
                    sample_df = pd.read_json(StringIO(log), typ='frame')
                    sample_df = sample_df[self.agent.model.get_booster().feature_names]

                    # Predict
                    probs = self.agent.model.predict_proba(sample_df)[0]
                    pred_index = probs.argmax()
                    pred_class = self.agent.le.inverse_transform([pred_index])[0]
                    print(f"[{datetime.now().isoformat()}][CityHall] 🔍 Prediction for {car_jid}: {pred_class}")
                    
                    # If threat detected, notify all zone managers
                    if "benign" not in pred_class.lower():
                        print(f"[CityHall] 🚨 Malicious activity detected from {car_jid}, broadcasting to zones...")
                        for zone_jid in self.zone_managers:
                            threat_msg = Message(to=zone_jid)
                            threat_msg.body = json.dumps({
                                "command": "threat",
                                "car_id": f"{car_jid}@localhost"
                            })
                            threat_msg.set_metadata("performative", "request")
                            await self.send(threat_msg)
                            print(f"[CityHall] 🔔 Notified {zone_jid} about threat from {car_jid}")
                            
                    else:
                        print(f"[CityHall] ✅ Log from {car_jid} considered benign.")
                except Exception as e:
                    print("[CityHall] ❌ Failed to process message:", e)
            else:
                print("[CityHall] ⏳ Waiting for car data...")
            
    async def setup(self):
        print("[CityHall] 🏛️ Loading model...")
        df = pd.read_parquet("models/cic-collection_reduzido.parquet")
        self.X = df.drop(columns=["Label", "ClassLabel"])
        self.y_label = df["Label"]
        self.le = LabelEncoder()
        self.le.fit(self.y_label)
        self.model = joblib.load("models/xgb_model.joblib")

        self.add_behaviour(self.MonitorCarsBehaviour(period=1))
        print("[CityHall] 🟢 City Hall Agent is up and running.")


if __name__ == "__main__":
    async def main():
        agent = CityHallAgent("main@localhost", "pass")
        await agent.start(auto_register=True)
        while agent.is_alive():
            await asyncio.sleep(1)

    asyncio.run(main())