"""
Main script for SOC estimation with visualization
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
from kalman_filter import BatterySOCKalmanFilter

def process_csv_data(voltage_file, current_file, kf, capacity_ah=50.0, soc_ref_file=None):
    """
    Process CSV files with voltage and current data
    
    Args:
        voltage_file: Path to voltage CSV file
        current_file: Path to current CSV file
        kf: BatterySOCKalmanFilter instance
        capacity_ah: Battery capacity in Ah
        soc_ref_file: Path to reference SOC CSV file (optional)
        
    Returns:
        dict: Results dictionary with time series data
    """
    # Read CSV files
    print(f"Reading voltage data from: {voltage_file}")
    voltage_data = pd.read_csv(voltage_file, header=None, names=['voltage'])
    
    print(f"Reading current data from: {current_file}")
    current_data = pd.read_csv(current_file, header=None, names=['current'])
    
    # Read reference SOC if provided
    soc_ref_data = None
    if soc_ref_file and os.path.exists(soc_ref_file):
        print(f"Reading reference SOC data from: {soc_ref_file}")
        soc_ref_data = pd.read_csv(soc_ref_file, header=None, names=['soc_ref'])
    
    # Check if data lengths match
    min_length = min(len(voltage_data), len(current_data))
    if soc_ref_data is not None:
        min_length = min(min_length, len(soc_ref_data))
    
    if len(voltage_data) != len(current_data):
        print(f"Warning: Data length mismatch! Voltage: {len(voltage_data)}, Current: {len(current_data)}")
    
    voltage_data = voltage_data.iloc[:min_length]
    current_data = current_data.iloc[:min_length]
    if soc_ref_data is not None:
        soc_ref_data = soc_ref_data.iloc[:min_length]
    
    print(f"Using {min_length} samples")
    
    # Storage for results
    results = {
        'time': [],
        'voltage_measured': [],
        'current_measured': [],
        'soc': [],
        'soc_ref': [],
        'soc_error': [],
        'v_rc1': [],
        'v_rc2': [],
        'uncertainty': [],
        'voltage_predicted': [],
        'voltage_error': [],
        'innovation': [],
        'kalman_gain_soc': [],
        'kalman_gain_v1': [],
        'kalman_gain_v2': []
    }
    
    # If we have reference SOC, update initial state
    if soc_ref_data is not None:
        initial_soc_ref = soc_ref_data.iloc[0]['soc_ref']
        kf.x[0] = initial_soc_ref
        print(f"Updated initial SOC to match reference: {initial_soc_ref:.3f}")
    
    print(f"\nProcessing {min_length} data points...")
    print(f"Initial SOC: {kf.x[0]:.3f}")
    print(f"Sample time: {kf.dt}s")
    print(f"Battery capacity: {capacity_ah} Ah")
    
    # Process each data point
    for i in range(min_length):
        voltage = voltage_data.iloc[i]['voltage']
        current = current_data.iloc[i]['current']
        
        # Get predicted voltage before update
        v_pred = kf.get_predicted_voltage(current)
        
        # Calculate innovation before update
        innovation = voltage - v_pred
        
        # Estimate SOC
        state = kf.estimate_soc(voltage, current, capacity_ah)
        
        # Store results
        results['time'].append(i * kf.dt)
        results['voltage_measured'].append(voltage)
        results['current_measured'].append(current)
        results['soc'].append(state['soc'])
        results['v_rc1'].append(state['v_rc1'])
        results['v_rc2'].append(state['v_rc2'])
        results['uncertainty'].append(state['covariance'][0])
        results['voltage_predicted'].append(v_pred)
        results['voltage_error'].append(voltage - v_pred)
        results['innovation'].append(innovation)
        
        # Store Kalman gains
        if state['kalman_gain'] is not None:
            results['kalman_gain_soc'].append(state['kalman_gain'][0, 0])
            results['kalman_gain_v1'].append(state['kalman_gain'][1, 0])
            results['kalman_gain_v2'].append(state['kalman_gain'][2, 0])
        else:
            results['kalman_gain_soc'].append(0)
            results['kalman_gain_v1'].append(0)
            results['kalman_gain_v2'].append(0)
        
        # Store reference SOC and error if available
        if soc_ref_data is not None:
            soc_ref = soc_ref_data.iloc[i]['soc_ref']
            results['soc_ref'].append(soc_ref)
            results['soc_error'].append(state['soc'] - soc_ref)
        else:
            results['soc_ref'].append(None)
            results['soc_error'].append(None)
        
        # Print progress every 1000 samples
        if i % 1000 == 0 and i > 0:
            error_str = f"Err: {results['soc_error'][-1]:.4f}" if soc_ref_data is not None else ""
            print(f"Processed {i}/{min_length} | "
                  f"SOC: {state['soc']:.3f} | "
                  f"V_err: {voltage - v_pred:.3f}V | "
                  f"{error_str}")
    
    # Calculate final statistics
    total_charge = np.trapz(current_data['current'], dx=kf.dt) / 3600  # Ah
    soc_change = results['soc'][-1] - results['soc'][0]
    
    print(f"\n=== Processing Complete ===")
    print(f"Final SOC: {results['soc'][-1]:.3f}")
    print(f"SOC change: {soc_change:.3f}")
    print(f"Total charge: {total_charge:.2f} Ah")
    print(f"Average current: {np.mean(current_data['current']):.2f} A")
    print(f"Voltage range: {np.min(voltage_data['voltage']):.2f} - {np.max(voltage_data['voltage']):.2f} V")
    
    if soc_ref_data is not None:
        soc_errors = [e for e in results['soc_error'] if e is not None]
        print(f"\n=== SOC Estimation Accuracy ===")
        print(f"Mean absolute error: {np.mean(np.abs(soc_errors)):.4f}")
        print(f"RMS error: {np.sqrt(np.mean(np.array(soc_errors)**2)):.4f}")
        print(f"Max absolute error: {np.max(np.abs(soc_errors)):.4f}")
    
    return results


def plot_results_extended(results):
    """Plot extended estimation results with reference comparison"""
    
    # Set plot style
    plt.style.use('seaborn-v0_8-darkgrid')
    
    # Create figure with subplots
    fig = plt.figure(figsize=(16, 12))
    
    # 1. Voltage comparison
    ax1 = plt.subplot(3, 2, 1)
    ax1.plot(results['time'], results['voltage_measured'], 'b-', alpha=0.8, 
             label='Measured', linewidth=1.5)
    ax1.plot(results['time'], results['voltage_predicted'], 'r--', alpha=0.8, 
             label='Predicted', linewidth=1.5)
    ax1.set_ylabel('Voltage (V)')
    ax1.set_title('Battery Voltage', fontsize=12, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. Current (with all data points)
    ax2 = plt.subplot(3, 2, 2)
    # Plot every point to ensure we see all data
    ax2.plot(results['time'], results['current_measured'], 'g-', linewidth=0.5, alpha=0.8)
    ax2.set_ylabel('Current (A)')
    ax2.set_title('Battery Current', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # 3. SOC comparison
    ax3 = plt.subplot(3, 2, 3)
    ax3.plot(results['time'], results['soc'], 'b-', linewidth=2, label='Kalman Filter')
    if results['soc_ref'][0] is not None:
        ax3.plot(results['time'], results['soc_ref'], 'r--', linewidth=2, label='MATLAB Reference')
    ax3.set_ylabel('SOC')
    ax3.set_title('State of Charge Comparison', fontsize=12, fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_ylim(0, 1)
    
    # 4. SOC Error
    if results['soc_ref'][0] is not None:
        ax4 = plt.subplot(3, 2, 4)
        ax4.plot(results['time'], np.array(results['soc_error']) * 100, 'r-', linewidth=1)
        ax4.set_ylabel('SOC Error (%)')
        ax4.set_title('SOC Estimation Error', fontsize=12, fontweight='bold')
        ax4.grid(True, alpha=0.3)
        ax4.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        # Add error statistics
        errors_percent = np.array(results['soc_error']) * 100
        ax4.text(0.02, 0.98, f'Mean: {np.mean(errors_percent):.2f}%\n'
                            f'Std: {np.std(errors_percent):.2f}%\n'
                            f'Max: {np.max(np.abs(errors_percent)):.2f}%',
                transform=ax4.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 5. Covariance (Uncertainty)
    ax5 = plt.subplot(3, 2, 5)
    ax5.semilogy(results['time'], results['uncertainty'], 'b-', linewidth=1.5)
    ax5.set_ylabel('SOC Variance')
    ax5.set_xlabel('Time (s)')
    ax5.set_title('SOC Estimation Uncertainty (Convergence)', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    
    # 6. Kalman Gains
    ax6 = plt.subplot(3, 2, 6)
    ax6.plot(results['time'], results['kalman_gain_soc'], 'b-', label='K_SOC', linewidth=1.5)
    ax6.plot(results['time'], results['kalman_gain_v1'], 'r-', label='K_V1', linewidth=1.5)
    ax6.plot(results['time'], results['kalman_gain_v2'], 'g-', label='K_V2', linewidth=1.5)
    ax6.set_ylabel('Kalman Gain')
    ax6.set_xlabel('Time (s)')
    ax6.set_title('Kalman Gain Evolution', fontsize=12, fontweight='bold')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Additional diagnostic plots
    fig2, axes = plt.subplots(2, 2, figsize=(12, 8))
    
    # 1. Voltage Error/Innovation
    axes[0, 0].plot(results['time'], results['voltage_error'], 'b-', alpha=0.7, linewidth=1)
    axes[0, 0].set_ylabel('Voltage Error (V)')
    axes[0, 0].set_title('Voltage Prediction Error (Innovation)', fontweight='bold')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # 2. RC Voltages
    axes[0, 1].plot(results['time'], results['v_rc1'], label='V_RC1', linewidth=1.5)
    axes[0, 1].plot(results['time'], results['v_rc2'], label='V_RC2', linewidth=1.5)
    axes[0, 1].set_ylabel('Voltage (V)')
    axes[0, 1].set_title('RC Circuit Voltages', fontweight='bold')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # 3. Histogram of voltage errors
    axes[1, 0].hist(results['voltage_error'], bins=50, edgecolor='black', alpha=0.7)
    axes[1, 0].set_xlabel('Voltage Error (V)')
    axes[1, 0].set_ylabel('Frequency')
    axes[1, 0].set_title('Distribution of Voltage Errors', fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3)
    # Add statistics
    axes[1, 0].text(0.02, 0.98, f'Mean: {np.mean(results["voltage_error"]):.3f}V\n'
                                f'Std: {np.std(results["voltage_error"]):.3f}V',
                    transform=axes[1, 0].transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # 4. SOC vs Voltage scatter
    axes[1, 1].scatter(results['soc'], results['voltage_measured'], alpha=0.1, s=1)
    axes[1, 1].set_xlabel('SOC')
    axes[1, 1].set_ylabel('Voltage (V)')
    axes[1, 1].set_title('SOC vs Terminal Voltage', fontweight='bold')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Current analysis plot
    fig3, axes = plt.subplots(2, 1, figsize=(12, 6))
    
    # Full current profile
    axes[0].plot(results['time'], results['current_measured'], 'g-', linewidth=0.5)
    axes[0].set_ylabel('Current (A)')
    axes[0].set_title('Full Current Profile', fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Zoomed current (first 1000 points)
    zoom_end = min(1000, len(results['time']))
    axes[1].plot(results['time'][:zoom_end], results['current_measured'][:zoom_end], 
                 'g-', linewidth=1.5, marker='o', markersize=2, alpha=0.7)
    axes[1].set_ylabel('Current (A)')
    axes[1].set_xlabel('Time (s)')
    axes[1].set_title('Zoomed Current Profile (First 1000 samples)', fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    plt.show()


# Main execution
if __name__ == "__main__":
    # File paths
    voltage_file = "export_u.csv"
    current_file = "export_i.csv"
    soc_ref_file = "export_soc.csv"
    
    if os.path.exists(voltage_file) and os.path.exists(current_file):
        print("=== Battery SOC Estimation with Extended Kalman Filter ===\n")
        
        # Create filter instance
        initial_soc = 0.5  # Default
        if os.path.exists(soc_ref_file):
            # Read first value from reference file
            soc_ref_df = pd.read_csv(soc_ref_file, header=None, nrows=1)
            initial_soc = float(soc_ref_df.iloc[0, 0])
            print(f"Using initial SOC from reference: {initial_soc:.3f}")
        
        # Initialize Kalman filter
        kf = BatterySOCKalmanFilter(dt=0.1, initial_soc=initial_soc)
        
        # Process CSV data
        results = process_csv_data(
            voltage_file=voltage_file,
            current_file=current_file,
            kf=kf,
            capacity_ah=44.0,  # Battery capacity in Ah
            soc_ref_file=soc_ref_file if os.path.exists(soc_ref_file) else None
        )
        
        # Save results to CSV
        results_df = pd.DataFrame(results)
        results_df.to_csv('soc_estimation_results.csv', index=False)
        print(f"\nResults saved to: soc_estimation_results.csv")
        
        # Create visualizations
        print("\nGenerating plots...")
        plot_results_extended(results)
        
    else:
        print("ERROR: Required CSV files not found!")
        print(f"Please ensure the following files exist in the current directory:")
        print(f"  - {voltage_file}")
        print(f"  - {current_file}")
        print(f"  - {soc_ref_file} (optional, for comparison)")
