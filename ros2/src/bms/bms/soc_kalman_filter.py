import numpy as np
from .battery_params import BatteryParameters

class StateOfChargeFilter:
    """
    Extended Kalman Filter for state of charge estimation of a battery 
    using Thevenin Model (Equivalent Circuit)
    """

    def __init__(self, P, Q, R, H, ocv=None, initial_soc=0.5, dt=1.0):

        self._P = P # state covariance matrix (3x3)
        self._Q = Q # process noise covariance (3x3)
        self._R = R # measurement noise covariance (1x1)
        self._H = H # measurement matrix

        self._dt = dt                            

        # initialize battery parameters         
        self._battery_params = BatteryParameters()

        initial_soc = self.get_initial_soc(ocv) if ocv else initial_soc

        # state vector: [SOC, V_RC1, V_RC2]
        self._x = np.array([initial_soc, 0.0, 0.0])

        
    @property
    def x(self):
        return self._x


    def get_initial_soc(self, voltage):
        ocv_data = self._battery_params.ocv_data
        n = len(ocv_data)
        index = n -1
        ocv = ocv_data[index]

        while(index >= 0 and ocv > voltage):
            index -= 1
            ocv = ocv_data[index]

        # based on the charged OCV, compensate for VR0 and select the next higher OCV value
        index = min(n, index + 1)
        return self._battery_params.soc_bp[index]


    def predict(self, current: float, capacity_ah: float):
        dt  = self._dt  # time step
        x   = self._x   # current state
        P   = self._P   # covariance matrix
        Q   = self._Q   # process noise matrix
        soc = x[0]      # current soc

        # lookup battery parameters for the given soc state
        r1 = self._battery_params.lookup('r1', soc)
        c1 = self._battery_params.lookup('c1', soc)
        r2 = self._battery_params.lookup('r2', soc)
        c2 = self._battery_params.lookup('c2', soc)
        
        # calculate exponential terms
        exp1 = np.exp(-dt / (r1 * c1))
        exp2 = np.exp(-dt / (r2 * c2))

        # state transition matrix
        F = np.array([
            [1.0, 0.0, 0.0],
            [0.0, exp1, 0.0],
            [0.0, 0.0, exp2]
        ])

        # control matrix (input transition matrix)
        G = np.array([
            -dt/ (3600 * capacity_ah),
            r1 * (1 - exp1),
            r2 * (1 - exp2)
        ])

        # Richtig
        # soc_next = soc - (current * dt) / (3600.0 * capacity_ah)
        # v_rc1_next = exp1 * x[1] + r1 * (1 - exp1) * current
        # v_rc2_next = exp2 * x[2] + r2 * (1 - exp2) * current
        # self._x = np.array([soc_next, v_rc1_next, v_rc2_next])
        
        # update state vector
        self._x = F @ x.T + G * current 
        
        # predict error covariance
        self._P = F @ P @ F.T + Q

        
    def update(self, z):
        # measurements
        u_meas, i_meas = z  # measured voltage and current
        soc   = self._x[0]  # current soc
        u_rc1 = self._x[1]  # rc1 voltage
        u_rc2 = self._x[2]  # rc2 voltage

        
        # lookup battery parameters
        ocv      = self._battery_params.lookup('ocv', soc)
        r0       = self._battery_params.lookup('r0',  soc)
        dv0_dsoc = self._battery_params.lookup('dv0', soc)
        
        # Update measurement matrix H with current OCV derivative
        self._H[0, 0] = dv0_dsoc

        H = self._H  # measurement matrix
        P = self._P  # state covariance matrix
        R = self._R  # measurement noise
        
        # voltage loss caused by the battery's internal resistance  
        u_intern = r0 * i_meas

        # predicted terminal voltage
        u_pred = ocv - (u_intern + u_rc1 + u_rc2)
        
        # measurement residual (innovation)
        y = u_meas - u_pred
        
        # innovation covariance
        S = H @ P @ H.T + R
        
        # kalman gain
        K = P @ H.T @ np.linalg.inv(S)
        
        # update state estimate
        self._x = self._x + K.flatten() * y
        
        # update error covariance
        I = np.eye(3)
        self._P = (I - K @ H) @ P
        
        # ensure SOC stays within bounds
        self._x[0] = np.clip(self._x[0], 0, 1)