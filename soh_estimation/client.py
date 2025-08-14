import sys
import numpy as np
import csv
from plot_utils import plot_results
from soh_estimation.soh_kalman_filter import BatteryCapacityEKF
from soc_estimation.dekf import CombinedKalmanFilter


def load_csv(filename, max_rows=1000000):
    data = []
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            first_row = next(reader)
            try:
                value = float(first_row[0])
                data.append(value)
            except ValueError:
                pass
            
            for idx, row in enumerate(reader):
                if idx >= max_rows - 1:
                    print(f"  Limited to {max_rows} rows")
                    break
                if row:
                    try:
                        data.append(float(row[0]))
                    except ValueError:
                        continue
                        
    except FileNotFoundError:
        print(f"Error: File {filename} not found")
        sys.exit(1)
        
    return np.array(data)


def test_kalman_filter(current_file, soc_file, ah_file, initial_capacity=44.0):
    # Load CSV files
    current = load_csv(current_file)
    soc = load_csv(soc_file)
    ah_reference = load_csv(ah_file)
    
    min_len = min(len(current), len(soc), len(ah_reference))
    if len(current) != len(soc) or len(current) != len(ah_reference):
        print(f"\nWarning: Data length mismatch")
        print(f"  Current: {len(current)}")
        print(f"  SOC: {len(soc)}")
        print(f"  Reference: {len(ah_reference)}")
        print(f"  Using first {min_len} samples")
        
    current = current[:min_len]
    soc = soc[:min_len]
    ah_reference = ah_reference[:min_len]
    
    time = np.arange(len(current)) * 1.0
    
    print(f"\nData Statistics:")
    print(f"  Samples: {len(current)}")
    print(f"  Duration: {time[-1]/3600:.2f} hours")
    print(f"  Current: {np.min(current):.2f} to {np.max(current):.2f} A (mean: {np.mean(current):.2f})")
    print(f"  SOC: {np.min(soc):.3f} to {np.max(soc):.3f} (mean: {np.mean(soc):.3f})")
    print(f"  Reference: {np.min(ah_reference):.2f} to {np.max(ah_reference):.2f} Ah")
    
    print("\n" + "="*50)
    print("RUNNING KALMAN FILTER")
    print("="*50)
    
    print(f"Initial capacity: {initial_capacity} Ah")
    
    
    #kf = BatteryCapacityEKF()
    kf = CombinedKalmanFilter()
    
    capacity_estimates = []
    covariance_history = []
    
    print("Processing...")
    for i in range(len(current)):
        curr = -current[i]
        capacity = kf.step(curr, soc[i])
        
        capacity_estimates.append(capacity)
        covariance_history.append(kf._P)
        
        if (i + 1) % 50000 == 0:
            print(f"  Processed {i+1}/{len(current)} samples...")
            print(f"    Current estimate: {capacity} Ah")
    
    # Convert to arrays
    capacity_estimates = np.array(capacity_estimates)
    covariance_history = np.array(covariance_history)
    
    print(f"\nProcessing complete!")
    print(f"Final capacity estimate: {capacity_estimates[-1]} Ah")
    
    print("\nGenerating plots...")
    plot_results(time, current, soc, capacity_estimates, ah_reference, covariance_history)
    
    return capacity_estimates, ah_reference


if __name__ == "__main__":
    
    current_file = "export_soh_i.csv"
    soc_file =  "export_soh_soc.csv"
    ah_file =   "export_soh_ah.csv"
    
    test_kalman_filter(current_file, soc_file, ah_file, initial_capacity=44.0)