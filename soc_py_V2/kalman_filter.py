"""
Extended Kalman Filter for Battery SOC Estimation
"""

import numpy as np
from battery_params import BatteryParameters

class ExtendedKalmanFilter:
    def __init__(self, dt=1.0, initial_soc=0.5):
        self.dt = dt
        
        # Initialize battery parameters
        self.battery_params = BatteryParameters()
        
        # State vector: [SOC, V_RC1, V_RC2]
        self.x = np.array([initial_soc, 0.0, 0.0])
        
        # State covariance matrix P (3x3)
        self.P = np.diag([1e-6, 1e-6, 1.0])
        
        # Process noise covariance Q (3x3)
        self.Q = np.diag([1e-4, 1e-4, 1e-4])
        
        # Measurement noise covariance R (1x1)
        self.R = np.array([[0.7]])
        
        # Initialize matrices
        self.F = np.eye(3)  # State transition matrix
        self.H = np.array([[1.0, -1.0, -1.0]])  # Measurement matrix
        
        # Store history for analysis
        self.kalman_gain_history = []
        
    def predict(self, current, capacity_ah=1.0):
        """
        Prediction step of the Kalman filter
        
        Args:
            current: Battery current in A (positive for discharge)
            capacity_ah: Battery capacity in Ah
        """
        # Get current SOC
        soc = self.x[0]
        
        # Lookup battery parameters
        r1 = self.battery_params.lookup('r1', soc)
        c1 = self.battery_params.lookup('c1', soc)
        r2 = self.battery_params.lookup('r2', soc)
        c2 = self.battery_params.lookup('c2', soc)
        
        # Calculate time constants
        tau1 = r1 * c1
        tau2 = r2 * c2
        
        # Calculate exponential terms
        exp1 = np.exp(-self.dt / tau1)
        exp2 = np.exp(-self.dt / tau2)
        
        # Update state transition matrix F (Jacobian)
        self.F = np.array([
            [1.0, 0.0, 0.0],
            [0.0, exp1, 0.0],
            [0.0, 0.0, exp2]
        ])

        G = np.array([
            -self.dt/ (3600 * capacity_ah),
            r1 * (1 - exp1),
            r2 * (1 - exp2)
            ])
        
        # Update state vector
        self.x = self.F @ self.x.T + G * current
        
        # Predict error covariance: P = F * P * F' + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, voltage_measured, current):
        """
        Update (correction) step of the Kalman filter
        
        Args:
            voltage_measured: Measured terminal voltage in V
            current: Battery current in A (negative for charge)
        """
        # Get current SOC
        soc = self.x[0]
        
        # Lookup battery parameters
        ocv = self.battery_params.lookup('ocv', soc)
        r0 = self.battery_params.lookup('r0', soc)
        dv0_dsoc = self.battery_params.lookup('dv0', soc)
        
        # Update measurement matrix H with current OCV derivative
        self.H[0, 0] = dv0_dsoc
        
        # Calculate predicted terminal voltage
        # V_terminal = OCV + I*R0 - V_RC1 - V_RC2
        v_predicted = ocv - r0 * current - self.x[1] - self.x[2]
        
        # Innovation (measurement residual)
        innovation = voltage_measured - v_predicted
        
        # Innovation covariance: S = H * P * H' + R
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain: K = P * H' * S^(-1)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Store Kalman gain for analysis
        self.kalman_gain_history.append(K.copy())
        
        # Update state estimate: x = x + K * innovation
        self.x = self.x + K.flatten() * innovation
        
        # Update error covariance: P = (I - K * H) * P
        I = np.eye(3)
        self.P = (I - K @ self.H) @ self.P
        
        # Ensure SOC stays within physical bounds
        self.x[0] = np.clip(self.x[0], 0, 1)
        
    def estimate_soc(self, voltage, current, capacity_ah=1.0):
        """
        Main function to estimate SOC using both predict and update steps
        
        Args:
            voltage: Measured terminal voltage in V
            current: Measured current in A (negative for charge)
            capacity_ah: Battery capacity in Ah
            
        Returns:
            dict: Contains SOC and other states
        """
        # Prediction step
        self.predict(current, capacity_ah)
        
        # Update step
        self.update(voltage, current)
        
        return {
            'soc': self.x[0],
            'v_rc1': self.x[1],
            'v_rc2': self.x[2],
            'covariance': np.diag(self.P),
            'kalman_gain': self.kalman_gain_history[-1] if self.kalman_gain_history else None
        }
