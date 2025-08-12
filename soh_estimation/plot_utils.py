import numpy as np
import matplotlib.pyplot as plt


def plot_results(time, current, soc, capacity_estimates, capacity_reference, covariance):
    time                = np.array(time).flatten()
    current             = np.array(current).flatten()
    soc                 = np.array(soc).flatten()
    capacity_estimates  = np.array(capacity_estimates).flatten()
    capacity_reference  = np.array(capacity_reference).flatten()
    covariance          = np.array(covariance).flatten()

    # Calculate error and uncertainty
    error = capacity_estimates - capacity_reference
    three_sigma = 3 * np.sqrt(covariance)
    
    # Create figure with 4 subplots
    fig, axes = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot 1: Capacity comparison
    ax = axes[0]
    ax.set_ylim(30, 46)
    ax.plot(time / 3600, capacity_estimates, 'b-', label='Kalman Filter Estimate', linewidth=1.5)
    ax.plot(time / 3600, capacity_reference, 'r--', label='MATLAB Reference', linewidth=1.5)
    ax.fill_between(time / 3600,
                     capacity_estimates - three_sigma,
                     capacity_estimates + three_sigma,
                     alpha=0.2, color='blue', label='3σ Uncertainty')
    ax.set_xlabel('Time (hours)')
    ax.set_ylabel('Capacity (Ah)')
    ax.set_title('Battery Capacity Estimation: Kalman Filter vs MATLAB')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    
    # Plot 2: Input signals
    ax = axes[1]
    ax2 = ax.twinx()
    
    line1 = ax.plot(time / 3600, soc, 'g-', label='SOC', alpha=0.8)
    line2 = ax2.plot(time / 3600, current, 'orange', label='Current', alpha=0.8)
    
    ax.set_xlabel('Time (hours)')
    ax.set_ylabel('SOC', color='g')
    ax2.set_ylabel('Current (A)', color='orange')
    ax.set_title('Input Signals')
    ax.grid(True, alpha=0.3)
    
    # Combine legends
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='best')
    
    plt.tight_layout()
    plt.show()
    
    return fig