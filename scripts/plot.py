import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("data/data.csv")

plt.figure(figsize=(10, 6))
plt.plot(df["true_x"], df["true_y"], linestyle="--", label="True trajectory")
plt.plot(df["measured_x"], df["measured_y"], alpha=0.5, label="Measured trajectory")
plt.plot(df["estimated_x"], df["estimated_y"], linewidth=2, label="Estimated trajectory")

plt.xlabel("$x$ (m)")
plt.ylabel("$y$ (m)")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()
