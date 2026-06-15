import os  
import pandas as pd
import matplotlib.pyplot as plt

log_path = os.path.join("logs", "hover.csv") 

if not os.path.exists(log_path):
    print(f"Error: Log data file not found at {log_path}. Run sim_binary.exe first.")
    exit(1) 

# Read the logged data telemetry
df = pd.read_csv(log_path)

plt.figure(figsize=(12, 8))

# Subplot 1: Pitch Angle Tracking (Setpoint vs Mahony Estimate) sensor fusion attitude estimator
plt.subplot(2, 1, 1)
plt.plot(df['time_s'], df['setpoint_pitch'], 'r--', label='Target Pitch Setpoint (deg)')
plt.plot(df['time_s'], df['pitch_est'], 'b-', label='Mahony Filter Estimate (deg)')
plt.grid(True, linestyle=':')
plt.title("Avionics Telemetry: Pitch Step Response & Level Attenuation")
plt.ylabel("Angle (Degrees)")
plt.legend(loc="upper right")

# Subplot 2: Dynamic Motor Allocations
plt.subplot(2, 1, 2)
plt.plot(df['time_s'], df['motor1'], label='Motor 1 (FR)')
plt.plot(df['time_s'], df['motor4'], label='Motor 4 (FL)')
plt.grid(True, linestyle=':')
plt.xlabel("Simulation Time (Seconds)")
plt.ylabel("Normalized Throttle Command (0.0 - 1.0)")
plt.legend(loc="upper right")

plt.tight_layout()
print("Generating visualization summary plot: logs/flight_analysis.png")
plt.savefig(os.path.join("logs", "flight_analysis.png"), dpi=300)
plt.show()
