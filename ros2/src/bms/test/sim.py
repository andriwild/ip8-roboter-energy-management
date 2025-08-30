# source install/setup.zsh
# py -m src.bms.test.sim

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from bms.soc_kalman_filter import StateOfChargeFilter
from bms.dual_kalman_filter import DualKalmanFilter
from bms.soh_kalman_filter import CapacityFilter

FILTER_CONFIGS = [
    {
        'name': 'Standard',
        'color': 'blue',
        'linestyle': '-',
        'P': np.diag([1e-6, 1e-6, 1.0]),
        'Q': np.diag([1e-4, 1e-4, 1e-4]),
        'R': np.array([[0.7]]),
        'H': np.array([[1.0, -1.0, -1.0]]),
        'soh_Q': 1e-5,
        'soh_R': 5e-3,
        'soh_P': 1e-8,
        'capacity_ah': 44.0
    },
    # {
    #     'name': 'High Process Noise',
    #     'color': 'green',
    #     'linestyle': '--',
    #     'P': np.diag([1e-6, 1e-6, 1.0]),
    #     'Q': np.diag([0.5e-5, 0.5e-5, 0.5e-5]),
    #     'R': np.array([[2.0]]),
    #     'H': np.array([[1.0, -1.0, -1.0]]),
    #     'soh_Q': 1e-5,
    #     'soh_R': 5e-3,
    #     'soh_P': 1e-8,
    #     'capacity_ah': 44.0
    # },
    # {
    #     'name': 'Low Measurement Noise',
    #     'color': 'purple',
    #     'linestyle': '-.',
    #     'P': np.diag([1e-6, 1e-6, 1.0]),
    #     'Q': np.diag([0.5e-5, 0.5e-5, 0.5e-5]),
    #     'R': np.array([[1.5]]),
    #     'H': np.array([[1.0, -1.0, -1.0]]),
    #     'soh_Q': 1e-5,
    #     'soh_R': 5e-3,
    #     'soh_P': 1e-8,
    #     'capacity_ah': 44.0
    # },
    # {
    #     'name': 'Aggressive Tuning',
    #     'color': 'orange',
    #     'linestyle': ':',
    #     'P': np.diag([1e-4, 1e-4, 0.5]),
    #     'Q': np.diag([0.5e-5, 0.5e-5, 0.5e-5]),
    #     'R': np.array([[1.0]]),
    #     'H': np.array([[1.0, -1.0, -1.0]]),
    #     'soh_Q': 1e-4,
    #     'soh_R': 4e-3,
    #     'soh_P': 1e-6,
    #     'capacity_ah': 44.0
    # },
    {
        'name': 'High Process Noise',
        'color': 'black',
        'linestyle': '--',
        'P': np.diag([1e-6, 1e-6, 1.0]),
        'Q': np.diag([1e-6, 1e-6, 1e-6]),
        'R': np.array([[2.0]]),
        'H': np.array([[1.0, -1.0, -1.0]]),
        'soh_Q': 1e-5,
        'soh_R': 5e-3,
        'soh_P': 1e-8,
        'capacity_ah': 44.0
    },
    {
        'name': 'High Process Noise',
        'color': 'black',
        'linestyle': ':',
        'P': np.diag([1e-6, 1e-6, 1.0]),
        'Q': np.diag([1e-6, 1e-6, 1e-6]),
        'R': np.array([[4.0]]),
        'H': np.array([[1.0, -1.0, -1.0]]),
        'soh_Q': 1e-5,
        'soh_R': 5e-3,
        'soh_P': 1e-8,
        'capacity_ah': 44.0
    },
    {
        'name': 'High Process Noise',
        'color': 'orange',
        'linestyle': '-',
        'P': np.diag([1e-6, 1e-6, 1.0]),
        'Q': np.diag([1e-7, 1e-7, 1e-7]),
        'R': np.array([[4.0]]),
        'H': np.array([[1.0, -1.0, -1.0]]),
        'soh_Q': 1e-5,
        'soh_R': 5e-3,
        'soh_P': 1e-8,
        'capacity_ah': 44.0
    },
    {
        'name': 'High Process Noise',
        'color': 'orange',
        'linestyle': ':',
        'P': np.diag([1e-6, 1e-6, 1.0]),
        'Q': np.diag([1e-7, 1e-7, 1e-7]),
        'R': np.array([[3.0]]),
        'H': np.array([[1.0, -1.0, -1.0]]),
        'soh_Q': 1e-5,
        'soh_R': 5e-3,
        'soh_P': 1e-8,
        'capacity_ah': 44.0
    },
    {
        'name': 'Low Measurement Noise',
        'color': 'gray',
        'linestyle': '-.',
        'P': np.diag([1e-6, 1e-6, 1.0]),
        'Q': np.diag([1e-6, 1e-6, 1e-6]),
        'R': np.array([[1.5]]),
        'H': np.array([[1.0, -1.0, -1.0]]),
        'soh_Q': 1e-5,
        'soh_R': 5e-3,
        'soh_P': 1e-8,
        'capacity_ah': 44.0
    },
    # {
    #     'name': 'Aggressive Tuning',
    #     'color': 'pink',
    #     'linestyle': ':',
    #     'P': np.diag([1e-4, 1e-4, 0.5]),
    #     'Q': np.diag([1e-6, 1e-6, 1e-6]),
    #     'R': np.array([[1.0]]),
    #     'H': np.array([[1.0, -1.0, -1.0]]),
    #     'soh_Q': 1e-4,
    #     'soh_R': 4e-3,
    #     'soh_P': 1e-6,
    #     'capacity_ah': 44.0
    # }
]

def create_filter_from_config(config, initial_soc, dt):
    ekf = StateOfChargeFilter(
        P=config['P'],
        Q=config['Q'],
        R=config['R'],
        H=config['H'],
        dt=dt,
        initial_soc=initial_soc
    )
    
    soh_kf = CapacityFilter(
        Q=config['soh_Q'],
        R=config['soh_R'],
        P=config['soh_P']
    )
    
    return DualKalmanFilter(ekf, soh_kf)

def process_and_plot(voltage_file, current_file, soc_ref_file, soc_true_file=None, 
                    capacity_ah=44.0, dt=0.1):
    voltage_data = pd.read_csv(voltage_file, header=None, names=['voltage'])
    current_data = pd.read_csv(current_file, header=None, names=['current'])

    soc_ref_data = None
    if soc_ref_file and os.path.exists(soc_ref_file):
        soc_ref_data = pd.read_csv(soc_ref_file, header=None, names=['soc_ref'])
        print(f"Loaded SOC true data from {soc_ref_data}")
    
    soc_true_data = None
    if soc_true_file and os.path.exists(soc_true_file):
        soc_true_data = pd.read_csv(soc_true_file, header=None, names=['soc_true'])
        print(f"Loaded SOC true data from {soc_true_file}")
    
    min_length = min(len(voltage_data), len(current_data))
    if soc_ref_data is not None:
        min_length = min(min_length, len(soc_ref_data))
    if soc_true_data is not None:
        min_length = min(min_length, len(soc_true_data))
    
    voltage_data = voltage_data.iloc[:min_length]
    current_data = current_data.iloc[:min_length]

    if soc_ref_data is not None:
        soc_ref_data = soc_ref_data.iloc[:min_length]
    if soc_true_data is not None:
        soc_true_data = soc_true_data.iloc[:min_length]
    
    print(f"Processing {min_length} data points with {len(FILTER_CONFIGS)} filter configurations...")
    
    time = np.arange(min_length) * dt
    
    initial_soc = soc_ref_data.iloc[0]['soc_ref'] if soc_ref_data is not None else 0.4
    initial_soc = 0.5
    
    filters = []
    results = []
    
    for config in FILTER_CONFIGS:
        kf = create_filter_from_config(config, initial_soc, dt)
        filters.append(kf)
        results.append({
            'name': config['name'],
            'color': config['color'],
            'linestyle': config['linestyle'],
            'capacity_ah': config.get('capacity_ah', capacity_ah),
            'soc': [],
            'soh': []
        })
    
    for i in range(min_length):
        u_meas = voltage_data.iloc[i]['voltage']
        i_meas = current_data.iloc[i]['current']
        
        for j, kf in enumerate(filters):
            soc, soh = kf.step(i_meas, u_meas)
            results[j]['soc'].append(soc)
            results[j]['soh'].append(soh)
        
        if i % 1000 == 0 and i > 0:
            print(f"Processed {i}/{min_length} samples...")
    
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    
    axes[0].plot(time, current_data['current'], 'g-', linewidth=0.5)
    axes[0].set_ylabel('Current (A)')
    axes[0].set_title('Battery Current')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    axes[1].plot(time, voltage_data['voltage'], 'b-', linewidth=0.5)
    axes[1].set_ylabel('Voltage (V)')
    axes[1].set_title('Battery Voltage')
    axes[1].grid(True, alpha=0.3)
    
    for result in results:
        axes[2].plot(time, result['soc'], 
                    color=result['color'], 
                    linestyle=result['linestyle'],
                    linewidth=1.5, 
                    label=f'KF: {result["name"]}',
                    alpha=0.8)
    
    if soc_ref_data is not None:
        axes[2].plot(time, soc_ref_data['soc_ref'], 'r--', 
                    linewidth=1.5, label='MATLAB Reference', alpha=0.7)
    if soc_true_data is not None:
        axes[2].plot(time, soc_true_data['soc_true'], 'k--', 
                    linewidth=2, label='SOC True', alpha=0.6)
    
    axes[2].set_ylabel('SOC')
    axes[2].set_title('State of Charge Comparison')
    axes[2].legend(loc='best', ncol=2, fontsize=8)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim(0, 1)
    
    for result in results:
        axes[3].plot(time, result['soh'], 
                    color=result['color'],
                    linestyle=result['linestyle'],
                    linewidth=1.5, 
                    label=f'{result["name"]}',
                    alpha=0.8)
    
    axes[3].set_ylabel('SOH (Capacity in Ah)')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_title('State of Health - Battery Capacity Estimation')
    axes[3].legend(loc='best', ncol=2, fontsize=8)
    axes[3].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    

if __name__ == "__main__":
    dir = "./src/bms/resource"
    #dir = "/home/andri/repos/ip8-roboter-energy-management/ros2/src/bms/resource/test_realistic_current_profile"
    #dir = "/home/andri/repos/ip8-roboter-energy-management/ros2/src/bms/test/export"
    dir = "/home/andri/repos/ip8-roboter-energy-management/ros2/src/bms/test/long_term"
    
    voltage_file = f"{dir}/export_u.csv"
    current_file = f"{dir}/export_i.csv"
    soc_ref_file = f"{dir}/export_soc.csv"
    soc_true_file = f"{dir}/export_soc_true.csv"
    
    if os.path.exists(voltage_file) and os.path.exists(current_file):
        process_and_plot(
            voltage_file=voltage_file,
            current_file=current_file,
            soc_ref_file=soc_ref_file,
            soc_true_file=soc_true_file,
            capacity_ah=44.0,
            dt=1.0
        )
    else:
        print("ERROR: Required CSV files not found!")