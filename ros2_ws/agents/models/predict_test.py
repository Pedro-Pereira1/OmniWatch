from sklearn.preprocessing import LabelEncoder
import pandas as pd
import joblib
import numpy as np

# 1. Carregar dataset
df = pd.read_parquet("cic-collection_reduzido.parquet")
X = df.drop(columns=["Label", "ClassLabel"])
y_label = df["Label"]

# 2. Reconstruir o LabelEncoder
le = LabelEncoder()
le.fit(y_label)

# 3. Carregar modelo
model = joblib.load("xgb_model.joblib")

# 4. Prever
index = 1000
sample = X.iloc[[index]]
probs = model.predict_proba(sample)[0]

# 5. Obter a classe mais provável
pred_index = probs.argmax()
pred_class = le.inverse_transform([pred_index])[0]

# 6. Obter a classe real
real_class = y_label.iloc[index]

# 7. Mostrar resultado
print(f"➡️ Previsão: {pred_class}")
print(f"✅ Real: {real_class}")

# 8. Mostrar a classe com maior probabilidade (top-1)
top_indices = np.argsort(probs)[::-1][:1]
for i in top_indices:
    name = le.inverse_transform([i])[0]
    print(f" - {name}: {probs[i]:.4f}")
