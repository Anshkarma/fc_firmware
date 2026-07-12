import numpy as np
import pandas as pd
import subprocess
import os
import re

# ==============================================================================
# FIRMWARE RE-COMPILATION AND RUNNING SYSTEM
# ==============================================================================
def compile_and_run(kp, ki, kd, scenario="tilt_recovery"):
    """
    Surgically injects PID gains into control.c, compilation paths fixed 
    to prevent directory traversal failures under Windows environment.
    """
    control_file_path = "src/control.c"
    
    # 1. Read existing firmware core architecture
    if not os.path.exists(control_file_path):
        print(f"[FATAL] Target code array not found at: {control_file_path}")
        return None
        
    with open(control_file_path, "r") as f:
        content = f.read()
    
    # Overwrite parameter matching matrix
    content = re.sub(r"float\s+kp_roll_rate\s*=\s*[\d\.]+f;", f"float kp_roll_rate = {kp:.4f}f;", content)
    content = re.sub(r"float\s+ki_roll_rate\s*=\s*[\d\.]+f;", f"float ki_roll_rate = {ki:.4f}f;", content)
    content = re.sub(r"float\s+kd_roll_rate\s*=\s*[\d\.]+f;", f"float kd_roll_rate = {kd:.4f}f;", content)
    
    with open(control_file_path, "w") as f:
        f.write(content)
        
    # 2. Hardened Compilation Vector with Strict Include Directory Flags (-I)
    # Fixed paths to explicitly query drivers_host/ and include headers path
    compile_cmd = [
        "gcc", 
        "sim/sim_main.c", "sim/plant.c", "sim/scenarios.c", 
        "drivers_host/time_mock.c", "drivers_host/motor_mock.c", 
        "src/control.c", "src/attitude.c", "src/modes.c", "src/dshot.c", 
        "-I.", "-Isrc", "-Isim", "-Idrivers_host", "-Ihal",
        "-o", "sim_engine.exe", "-lm"
    ]
    
    try:
        subprocess.run(compile_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[FATAL] Compilation Failure:\n{e.stderr.decode()}")
        return None

    # 3. Clean environment log footprints
    csv_path = f"logs/{scenario}.csv"
    if os.path.exists(csv_path):
        os.remove(csv_path)

    # 4. Fire the simulation binary execution block
    try:
        subprocess.run(["sim_engine.exe", "--scenario", scenario], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        if not os.path.exists(csv_path):
            return None
        return pd.read_csv(csv_path)
    except Exception as e:
        print(f"[ERROR] Simulation execution failed: {e}")
        return None

# ==============================================================================
# COST EVALUATION METRIC (Integral Absolute Error Vector)
# ==============================================================================
def calculate_cost(df):
    if df is None or df.empty:
        return float('inf')
    
    time = df['time_s'].values
    roll_error = df['roll_true'].values - df['roll_est'].values
    
    # Step metric 1: Tracking stability integration
    iae = np.trapz(np.abs(roll_error), time)
    
    # Step metric 2: Settling window overshoot checks
    settling_window = time > 1.5
    overshoot = np.max(np.abs(roll_error[settling_window])) if np.any(settling_window) else 0.0
    
    # Step metric 3: Dynamic stability tracking via pure motor float outputs
    m1 = df['motor1'].values
    actuator_jitter = np.sum(np.diff(m1) ** 2)
    
    return (iae * 15.0) + (overshoot * 8.0) + (actuator_jitter * 3.0)

# ==============================================================================
# TUNING MATRIX INTERFACE
# ==============================================================================
def autotune():
    print("[INIT] Starting Firmware-Level PID Autotuning Sequence...")
    
    # Clean injection targets
    best_gains = np.array([0.150, 0.025, 0.005])  # [Kp, Ki, Kd]
    step_sizes = np.array([0.020, 0.005, 0.001])
    
    df = compile_and_run(best_gains[0], best_gains[1], best_gains[2])
    best_cost = calculate_cost(df)
    
    print(f"[BASE] Starting Fitness Score: {best_cost:.4f} | Kp={best_gains[0]:.3f}, Ki={best_gains[1]:.3f}, Kd={best_gains[2]:.3f}")
    
    max_cycles = 15
    for cycle in range(1, max_cycles + 1):
        improved = False
        print(f"\n--- Optimization Tuning Step: Layer {cycle} ---")
        
        for i in range(3):
            for direction in [-1, 1]:
                test_gains = best_gains.copy()
                test_gains[i] += direction * step_sizes[i]
                
                if test_gains[i] < 0.0: continue
                
                df_test = compile_and_run(test_gains[0], test_gains[1], test_gains[2])
                test_cost = calculate_cost(df_test)
                
                if test_cost < best_cost:
                    best_cost = test_cost
                    best_gains = test_gains
                    improved = True
                    print(f"[PROGRESS] Cost Matrix Dropped to {best_cost:.4f} -> Gains Locked: Kp={best_gains[0]:.3f}, Ki={best_gains[1]:.3f}, Kd={best_gains[2]:.3f}")
        
        if not improved:
            step_sizes *= 0.5
            print(f"[INFO] Precision boundary met. Shrinking search vector grids.")
            if np.max(step_sizes) < 0.0002:
                print("[TERMINATE] Local minima stability confirmed.")
                break
                
    print("\n==============================================================================")
    print(f"[SUCCESS] Mathematical tuning completed.")
    print(f"Inject these optimal matrix variables into control.c -> Kp: {best_gains[0]:.4f} | Ki: {best_gains[1]:.4f} | Kd: {best_gains[2]:.4f}")
    print("==============================================================================")

if __name__ == "__main__":
    autotune()