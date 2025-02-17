import pandas as pd
import matplotlib.pyplot as plt
import argparse

# Set up argument parsing
parser = argparse.ArgumentParser(description="Plot simulation data from a CSV file.")
parser.add_argument("filepath", help="Path to the CSV file.")
args = parser.parse_args()

# Load data
try:
    df = pd.read_csv(args.filepath)
except FileNotFoundError:
    print(f"Error: File not found at {args.filepath}")
    exit(1)
except Exception as e:
    print(f"Error reading CSV: {e}")
    exit(1)

m1_displacement = 0.5
m2_displacement = 0.2

# Plot disturbances
plt.figure(figsize=(10, 6))
plt.plot(df["time"], df['xg'], label="xg (Displacement)")
plt.plot(df["time"], df['xg_dot'], label="xg_dot (Velocity)") 

plt.xlabel("Time (s)")
plt.ylabel("")
plt.title("Disturbance Over Time")
plt.legend()
plt.grid()

# Plot displacements (x1 and x2) with offsets
plt.figure(figsize=(10, 6))
plt.plot(df["time"], df["x1"] + m1_displacement + df['xg'], label="x1 (Sprung Mass + 0.6m)")  # Offset x1
plt.plot(df["time"], df["x2"] + m2_displacement + df['xg'], label="x2 (Unsprung Mass + 0.2m)") # Offset x2
# plt.plot(df["time"], df["x1"], label="x1 (Sprung Mass + 0.6m)")  # Offset x1
# plt.plot(df["time"], df["x2"], label="x2 (Unsprung Mass + 0.2m)") # Offset x2

plt.xlabel("Time (s)")
plt.ylabel("Displacement (m)")
plt.title("Displacements Over Time")
plt.legend()
plt.grid()

# Plot force from active suspension 
plt.figure(figsize=(10, 6)) 
plt.plot(df["time"], df["F"], label="Active Suspension Force [N]")

plt.xlabel("Time [s]")
plt.ylabel("Foce [N]")
plt.title("Force Over Time")
plt.legend()
plt.grid()

# Plot velocities (x1_dot and x2_dot)
plt.figure(figsize=(10, 6))  # New figure for velocities
plt.plot(df["time"], df["x1_dot"], label="x1_dot (Sprung Mass Velocity)")
plt.plot(df["time"], df["x2_dot"], label="x2_dot (Unsprung Mass Velocity)")

plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocities Over Time")
plt.legend()
plt.grid()

plt.show()