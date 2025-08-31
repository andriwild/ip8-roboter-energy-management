class DualKalmanFilter:
    def __init__(self, soc_filter, soh_filter):
        self._soc_filter = soc_filter
        self._soh_filter = soh_filter
        
    def step(self, current, voltage):
        capacity_estimate = self._soh_filter._x
        
        # SOC update with current capacity
        self._soc_filter.predict(current, capacity_estimate)
        self._soc_filter.update([voltage, current])
        
        # SOH update with new SOC
        soc_estimate = self._soc_filter._x[0]
        self._soh_filter.predict()
        self._soh_filter.update([current, soc_estimate])
        
        return soc_estimate, self._soh_filter._x
