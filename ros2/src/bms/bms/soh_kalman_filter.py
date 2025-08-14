import numpy as np

class CapacityFilter:
    def __init__(self, dt=1.0, Q=1e-5, R=5e-7, P=1e-8, Q_init=44.0):

        self._dt = dt # time step
        self._P = P   # state covariance matrix
        self._Q = Q   # process noise covariance
        self._R = R   # measurement noise covariance (1x1)
        self._x = Q_init  # state - capacity (Ah)
        self._prev_soc = None  # previous state of charge
        self._Q_init = Q_init

    def step(self, current, soc):
        self.predict()
        return self.update([current, soc])


    def update(self, z):
        current = z[0]
        soc = z[1]

        # first update, no derivation available
        if self._prev_soc is None:
            self._prev_soc = soc
            return self._x

        P = self._P
        x = self._x

        soc_dot = (soc - self._prev_soc) / self._dt
        self._prev_soc = soc

        h = soc_dot + current / (3600.0 * x)
        H = -current / (3600.0 * x**2)

        # compute the kalman gain
        S = H * P * H + self._R
        K = P * H / S

        # update estimate with measurement
        self._x = x + K * (0.0 - h)

        # update the estimate uncertainty
        self._P = (1 - K * H) * P


        self._x = np.clip(self._x, 0, self._Q_init)
        return self._x


    def predict(self):
        # extrapolate the state
        self._x = self._x

        # extrapolate uncertainty
        self._P = self._P + self._Q
