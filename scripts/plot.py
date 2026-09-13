import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("data/data.csv")

plt.figure(figsize=(10, 6))
plt.plot(df["iteration"], df["true_state"], marker="o", label="True state")
plt.plot(df["iteration"], df["measured_state"], marker="o", label="Measured state")
plt.plot(df["iteration"], df["estimated_state"], marker="o", label="Estimated state")

plt.xlabel("Iteration")
plt.ylabel("Temperature (°C)")
plt.legend()
plt.grid(True)
plt.show()
