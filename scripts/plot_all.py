import os
import glob
import argparse
import subprocess

def process_all_logs(logs_dir):
    if not os.path.exists(logs_dir):
        print(f"[ERROR] Directory '{logs_dir}' not found.")
        return

    # Find all CSV files in the logs directory
    csv_files = glob.glob(os.path.join(logs_dir, '*.csv'))
    
    if not csv_files:
        print(f"[INFO] No CSV files found in {logs_dir}.")
        return

    print(f"[BATCH] Found {len(csv_files)} logs. Commencing batch plot generation...")
    
    for csv_file in csv_files:
        # Call the single-plot script as a subprocess
        print(f"-> Processing: {csv_file}")
        subprocess.run(["python", "scripts/plot_scenario.py", csv_file])
        
    print("\n[SUCCESS] All scenarios processed end-to-end.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Batch process all CSV logs and generate their multi-panel plots."
    )
    parser.add_argument(
        "--logs_dir", 
        type=str, 
        default="logs", 
        help="Path to the directory containing CSV logs (default: 'logs')"
    )
    args = parser.parse_args()
    
    process_all_logs(args.logs_dir)