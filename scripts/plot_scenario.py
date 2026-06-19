import argparse
import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_scenario(csv_file):
    # Check file
    if not os.path.exists(csv_file):
        print(f"[ERROR] CSV file '{csv_file}' not found.")
        return

    # Load telemetry
    df = pd.read_csv(csv_file)
    time = df['time_s']

    # Initialize 4-panel figure
    fig, axs = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    fig.suptitle(f'Flight Telemetry Analysis: {os.path.basename(csv_file)}', fontsize=16, fontweight='bold')

    c_roll, c_pitch, c_yaw = '#e74c3c', '#2ecc71', '#3498db'

    # Panel A: Attitude
    axs[0].plot(time, df['roll_true'], color=c_roll, linestyle='-', label='True Roll')
    axs[0].plot(time, df['roll_est'], color=c_roll, linestyle='--', alpha=0.7, label='Est Roll')
    axs[0].plot(time, df['pitch_true'], color=c_pitch, linestyle='-', label='True Pitch')
    axs[0].plot(time, df['pitch_est'], color=c_pitch, linestyle='--', alpha=0.7, label='Est Pitch')
    axs[0].plot(time, df['yaw_true'], color=c_yaw, linestyle='-', label='True Yaw')
    axs[0].plot(time, df['yaw_est'], color=c_yaw, linestyle='--', alpha=0.7, label='Est Yaw')
    axs[0].axhline(y=2, color='gray', linestyle=':', alpha=0.8, label='±2° Pass Band')
    axs[0].axhline(y=-2, color='gray', linestyle=':', alpha=0.8)
    axs[0].set_title('Panel A: Attitude Verification', loc='left', fontweight='bold')
    axs[0].set_ylabel('Angle (deg)')
    axs[0].grid(True, alpha=0.3)
    axs[0].legend(loc='center left', bbox_to_anchor=(1, 0.5))

    # Panel B: Angular Rates
    axs[1].plot(time, df['rate_roll'], color=c_roll, linestyle='-', label='Meas Rate Roll')
    axs[1].plot(time, df['setpoint_roll'], color=c_roll, linestyle=':', label='SP Roll')
    axs[1].plot(time, df['rate_pitch'], color=c_pitch, linestyle='-', label='Meas Rate Pitch')
    axs[1].plot(time, df['setpoint_pitch'], color=c_pitch, linestyle=':', label='SP Pitch')
    axs[1].plot(time, df['rate_yaw'], color=c_yaw, linestyle='-', label='Meas Rate Yaw')
    axs[1].plot(time, df['setpoint_yaw'], color=c_yaw, linestyle=':', label='SP Yaw')
    axs[1].set_title('Panel B: Angular Rates Tracking', loc='left', fontweight='bold')
    axs[1].set_ylabel('Rate (deg/s)')
    axs[1].grid(True, alpha=0.3)
    axs[1].legend(loc='center left', bbox_to_anchor=(1, 0.5))

    # Panel C: Actuator Authority (Plotting RAW DShot Payload values)
    # Changed from motor_norm * 2047 to directly plotting dshot columns [48-2047]

    # ==========================================
    # Panel C: Actuator Authority (Normalized Motor Thrust)
    # ==========================================
    # Plotting the 'motor' columns which contain pure floats (0.0 to 1.0)
    axs[2].plot(time, df['motor1'], label='Motor 1', alpha=0.8)
    axs[2].plot(time, df['motor2'], label='Motor 2', alpha=0.8)
    axs[2].plot(time, df['motor3'], label='Motor 3', alpha=0.8)
    axs[2].plot(time, df['motor4'], label='Motor 4', alpha=0.8)
    
    axs[2].set_title('Panel C: Actuator Authority (Normalized Thrust)', loc='left', fontweight='bold')
    axs[2].set_ylabel('Thrust (0.0 - 1.0)')
    
    # Strictly lock the Y-axis to 0.0 -> 1.05 range for perfect scaling
    axs[2].set_ylim(-0.05, 1.05) 
    
    axs[2].grid(True, alpha=0.3)
    axs[2].legend(loc='center left', bbox_to_anchor=(1, 0.5))
    # Panel D: Position
    axs[3].plot(time, df['pos_z'], color='#8e44ad', linewidth=2, label='Z Position')
    axs[3].set_title('Panel D: Altitude Drift Analytics', loc='left', fontweight='bold')
    axs[3].set_ylabel('Altitude (m)')
    axs[3].set_xlabel('Time (s)', fontweight='bold')
    axs[3].grid(True, alpha=0.3)
    axs[3].legend(loc='center left', bbox_to_anchor=(1, 0.5))

    plt.tight_layout()
    plt.subplots_adjust(right=0.85)

    out_file = csv_file.replace('.csv', '_plot.png')
    plt.savefig(out_file, dpi=300)
    print(f"[SUCCESS] Plot generated: {out_file}")
    plt.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Deterministic Telemetry Plotter")
    parser.add_argument("csv_file", type=str, help="Path to scenario CSV")
    args = parser.parse_args()
    plot_scenario(args.csv_file)