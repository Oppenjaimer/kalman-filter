import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv("data/data.csv")

# Calculate standard deviation
df["sigma_x"] = np.sqrt(df["variance_x"])
df["sigma_y"] = np.sqrt(df["variance_y"])

plt.figure(figsize=(12, 10))

# Plot 2D trajectory
plt.subplot(2, 1, 1)
plt.plot(df["true_x"], df["true_y"], linestyle="--", label="True trajectory")
plt.plot(df["estimated_x"], df["estimated_y"], linewidth=2, zorder=10, label="Estimated trajectory")
plt.plot(df["measured_x"], df["measured_y"], alpha=0.5, linestyle="", marker="o", label="Measured trajectory")
plt.title("2D Trajectory")
plt.xlabel("$x$ (m)")
plt.ylabel("$y$ (m)")
plt.legend()
plt.grid(True)
plt.axis("equal")

# Plot X position vs time with 3σ bounds
plt.subplot(2, 2, 3)
plt.plot(df["iteration"], df["true_x"], linestyle="--", label="True X")
plt.plot(df["iteration"], df["estimated_x"], label="Estimated X")
plt.fill_between(df["iteration"],
                 df["estimated_x"] - 3 * df["sigma_x"],
                 df["estimated_x"] + 3 * df["sigma_x"],
                 alpha=0.2, label=r"$3\sigma$ bound")
plt.title("X Position")
plt.xlabel("Iteration")
plt.ylabel("$x$ (m)")
plt.legend()
plt.grid(True)

# Plot Y position vs time with 3σ bounds
plt.subplot(2, 2, 4)
plt.plot(df["iteration"], df["true_y"], linestyle="--", label="True Y")
plt.plot(df["iteration"], df["estimated_y"], label="Estimated Y")
plt.fill_between(df["iteration"],
                 df["estimated_y"] - 3 * df["sigma_y"],
                 df["estimated_y"] + 3 * df["sigma_y"],
                 alpha=0.2, label=r"$3\sigma$ bound")
plt.title("Y Position")
plt.xlabel("Iteration")
plt.ylabel("$y$ (m)")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
