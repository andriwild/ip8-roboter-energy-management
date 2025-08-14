# Launch the file with:
# . install/setup.zsh
# py -m src.bms.test.sim


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from bms.soc_kalman_filter import StateOfChargeFilter
from bms.dual_kalman_filter import DualKalmanFilter
from bms.soh_kalman_filter import CapacityFilter

def process_and_plot(voltage_file, current_file, soc_ref_file, capacity_ah=44.0, dt=0.1):
    print(f"Reading data files...")
    voltage_data = pd.read_csv(voltage_file, header=None, names=['voltage'])
    current_data = pd.read_csv(current_file, header=None, names=['current'])
    soc_ref_data = pd.read_csv(soc_ref_file, header=None, names=['soc_ref'])
    
    # ensure same length
    min_length = min(len(voltage_data), len(current_data))
    if soc_ref_data is not None:
        min_length = min(min_length, len(soc_ref_data))
    
    voltage_data = voltage_data.iloc[:min_length]
    current_data = current_data.iloc[:min_length]
    if soc_ref_data is not None:
        soc_ref_data = soc_ref_data.iloc[:min_length]
    
    print(f"Processing {min_length} data points...")
    
    # create time array
    time = np.arange(min_length) * dt
    
    # initialize Kalman filter
    initial_soc = soc_ref_data.iloc[0]['soc_ref'] if soc_ref_data is not None else 0.5

    ekf = StateOfChargeFilter(
            P = np.diag([1e-6, 1e-6, 1.0]),     # state covariance matrix (3x3)
            Q = np.diag([1e-4, 1e-4, 1e-4]),    # process noise covariance (3x3)
            R = np.array([[0.4]]),              # measurement noise covariance (1x1)
            H = np.array([[1.0, -1.0, -1.0]]),  # measurement matrix (3x1)
            dt = dt, 
            initial_soc = initial_soc,
            )

    soh_kf = CapacityFilter()

    kf = DualKalmanFilter(ekf, soh_kf)
    
    soc_kalman  = []
    soh_kalman  = []
    
    for i in range(min_length):
        u_meas = voltage_data.iloc[i]['voltage']
        i_meas = current_data.iloc[i]['current']
        i_meas = -i_meas

        # kf.predict(i_meas, capacity_ah)
        # kf.update([u_meas, i_meas])
        soc, soh = kf.step(i_meas, u_meas )
        
        #soc_kalman.append(kf.x[0])
        soc_kalman.append(soc)
        soh_kalman.append(soh)
        
        if i % 1000 == 0 and i > 0:
            print(f"Processed {i}/{min_length} samples...")
    

    # create plots
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    
    # 1. Current
    axes[0].plot(time, current_data['current'], 'g-', linewidth=0.5)
    axes[0].set_ylabel('Current (A)')
    axes[0].set_title('Battery Current from export_i.csv')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    # 2. Voltage
    axes[1].plot(time, voltage_data['voltage'], 'b-', linewidth=0.5)
    axes[1].set_ylabel('Voltage (V)')
    axes[1].set_title('Battery Voltage from export_u.csv')
    axes[1].grid(True, alpha=0.3)
    
    # 3. SOC Comparison
    axes[2].plot(time, soc_kalman, 'b-', linewidth=1.5, label='Kalman Filter')
    if soc_ref_data is not None:
        axes[2].plot(time, soc_ref_data['soc_ref'], 'r--', linewidth=1.5, label='MATLAB Reference')
    axes[2].set_ylabel('SOC')
    axes[2].set_title('State of Charge Comparison')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim(0, 1)
    
    # 4. Uncertainty
    axes[3].plot(time, soh_kalman, 'purple', linewidth=1.5)
    axes[3].set_ylabel('SOC Variance')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title('SOH')
    axes[3].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    if soc_ref_data is not None:
        errors = np.array(soc_kalman) - np.array(soc_ref_data['soc_ref'])
        print(f"\n=== Results ===")
        print(f"Mean Absolute Error: {np.mean(np.abs(errors)):.4f}")
        print(f"RMS Error: {np.sqrt(np.mean(errors**2)):.4f}")
        print(f"Max Absolute Error: {np.max(np.abs(errors)):.4f}")


if __name__ == "__main__":
    dir = "./src/bms/resource"
    voltage_file = f"{dir}/export_u.csv"
    current_file = f"{dir}/export_i.csv"
    soc_ref_file = f"{dir}/export_soc.csv"
    
    if os.path.exists(voltage_file) and os.path.exists(current_file):
        process_and_plot(
            voltage_file=voltage_file,
            current_file=current_file,
            soc_ref_file=soc_ref_file,
            capacity_ah=44.0,
            dt=1.0
        )
    else:
        print("ERROR: Required CSV files not found!")
