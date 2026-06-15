import argparse
import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_scenario(csv_file):
    # Check if file exists
    if not os.path.exists(csv_file):
        print(f"[ERROR] CSV file '{csv_file}' not found.")
        return

    # Load telemetry data
    df = pd.read_csv(csv_file)
    time = df['time_s']

    # Initialize a 4-panel figure, sharing the X-axis (Time)
    fig, axs = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    fig.suptitle(f'Flight Telemetry Analysis: {os.path.basename(csv_file)}', fontsize=16, fontweight='bold')

    # Color standardization: Roll=Red, Pitch=Green, Yaw=Blue
    c_roll, c_pitch, c_yaw = '#e74c3c', '#2ecc71', '#3498db'

    # ==========================================
    # Panel A: Attitude (Roll/Pitch/Yaw)
    # ==========================================
    axs[0].plot(time, df['roll_true'], color=c_roll, linestyle='-', label='True Roll')
    axs[0].plot(time, df['roll_est'], color=c_roll, linestyle='--', alpha=0.7, label='Est Roll')
    
    axs[0].plot(time, df['pitch_true'], color=c_pitch, linestyle='-', label='True Pitch')
    axs[0].plot(time, df['pitch_est'], color=c_pitch, linestyle='--', alpha=0.7, label='Est Pitch')
    
    axs[0].plot(time, df['yaw_true'], color=c_yaw, linestyle='-', label='True Yaw')
    axs[0].plot(time, df['yaw_est'], color=c_yaw, linestyle='--', alpha=0.7, label='Est Yaw')
    
    # ±2° Pass Threshold Lines
    axs[0].axhline(y=2, color='gray', linestyle=':', alpha=0.8, label='±2° Pass Band')
    axs[0].axhline(y=-2, color='gray', linestyle=':', alpha=0.8)
    
    axs[0].set_title('Panel A: Attitude Verification', loc='left', fontweight='bold')
    axs[0].set_ylabel('Angle (deg)')
    axs[0].legend(loc='upper right', bbox_to_anchor=(1.15, 1), fontsize='small')
    axs[0].grid(True, alpha=0.3)

    # ==========================================
    # Panel B: Angular Rates (Setpoint vs Measured)
    # ==========================================
    axs[1].plot(time, df['rate_roll'], color=c_roll, linestyle='-', label='Meas Rate Roll')
    axs[1].plot(time, df['setpoint_roll'], color=c_roll, linestyle=':', linewidth=2, label='SP Roll')
    
    axs[1].plot(time, df['rate_pitch'], color=c_pitch, linestyle='-', label='Meas Rate Pitch')
    axs[1].plot(time, df['setpoint_pitch'], color=c_pitch, linestyle=':', linewidth=2, label='SP Pitch')
    
    axs[1].plot(time, df['rate_yaw'], color=c_yaw, linestyle='-', label='Meas Rate Yaw')
    axs[1].plot(time, df['setpoint_yaw'], color=c_yaw, linestyle=':', linewidth=2, label='SP Yaw')

    axs[1].set_title('Panel B: Angular Rates Tracking', loc='left', fontweight='bold')
    axs[1].set_ylabel('Rate (deg/s)')
    axs[1].legend(loc='upper right', bbox_to_anchor=(1.15, 1), fontsize='small')
    axs[1].grid(True, alpha=0.3)

    # ==========================================
    # Panel C: Motor Outputs (0-2047 Scale)
    # ==========================================
    # Normalizing back to the 0-2047 DShot/Throttle scale as requested
    axs[2].plot(time, df['motor1'] * 2047, label='Motor 1', alpha=0.8)
    axs[2].plot(time, df['motor2'] * 2047, label='Motor 2', alpha=0.8)
    axs[2].plot(time, df['motor3'] * 2047, label='Motor 3', alpha=0.8)
    axs[2].plot(time, df['motor4'] * 2047, label='Motor 4', alpha=0.8)

    axs[2].set_title('Panel C: Actuator Authority', loc='left', fontweight='bold')
    axs[2].set_ylabel('Throttle (0-2047)')
    axs[2].set_ylim(0, 2100)
    axs[2].legend(loc='upper right', bbox_to_anchor=(1.15, 1), fontsize='small')
    axs[2].grid(True, alpha=0.3)

    # ==========================================
    # Panel D: Position / Altitude
    # ==========================================
    axs[3].plot(time, df['pos_z'], color='#8e44ad', linewidth=2, label='Z Position (Drift)')
    axs[3].set_title('Panel D: Altitude Drift Analytics', loc='left', fontweight='bold')
    axs[3].set_ylabel('Altitude (m)')
    axs[3].set_xlabel('Time (s)', fontweight='bold')
    axs[3].legend(loc='upper right', bbox_to_anchor=(1.15, 1), fontsize='small')
    axs[3].grid(True, alpha=0.3)

    # Final visual adjustments to fit legends
    plt.tight_layout()
    plt.subplots_adjust(right=0.85, top=0.95)

    # Save output alongside the CSV
    out_file = csv_file.replace('.csv', '_plot.png')
    plt.savefig(out_file, dpi=300)
    print(f"[SUCCESS] Multi-panel plot generated: {out_file}")
    
    # Close plot to free memory
    plt.close()

if __name__ == "__main__":
    # Built-in argparse handles the --help requirement automatically
    parser = argparse.ArgumentParser(
        description="Generates a 4-panel matplotlib figure from a scenario CSV log."
    )
    parser.add_argument(
        "csv_file", 
        type=str, 
        help="Path to the input scenario CSV file (e.g., logs/scenario_1_tilt.csv)"
    )
    args = parser.parse_args()
    
    plot_scenario(args.csv_file)