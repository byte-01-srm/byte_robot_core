import pandas as pd
import matplotlib.pyplot as plt

# === CHANGE THIS TO YOUR CSV FILE PATH ===
file_path = "/home/byte/ak60_motor_control/multiprocess_code/all_4_legs/cur_test/motor_currents.csv"

# Load data
df = pd.read_csv(file_path)

# Convert time to relative time (start from 0)
df["time"] = df["timestamp_s"] - df["timestamp_s"].iloc[0]

# Motor groupings
groups = {
    "Hip": ["m1_a", "m4_a", "m7_a", "m10_a"],
    "Thigh": ["m2_a", "m5_a", "m8_a", "m11_a"],
    "Knee": ["m3_a", "m6_a", "m9_a", "m12_a"]
}

# Plot each group
for title, motors in groups.items():
    plt.figure()

    for motor in motors:
        plt.plot(df["time"], df[motor], label=motor)

    plt.xlabel("Time (s)")
    plt.ylabel("Current (A)")
    plt.title(title)
    plt.legend()
    plt.grid()

    plt.show()