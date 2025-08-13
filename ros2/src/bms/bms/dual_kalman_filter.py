class DualKalmanFilter:
    def __init__(self, soc_filter, soh_filter, coupling_weight=0.1):
        self._soc_filter = soc_filter
        self._soh_filter = soh_filter
        self._coupling_weight = coupling_weight
        
    def step(self, current, voltage):
        # get initial estimates
        capacity_estimate = self._soh_filter._x
        
        # SOC update with current capacity
        self._soc_filter.predict(current, capacity_estimate)
        self._soc_filter.update([voltage, current])
        
        # SOH update with new SOC
        soc_estimate = self._soc_filter.x[0]
        capacity_new = self._soh_filter.step(current, soc_estimate)
        
        # optional: weighted averaging for stability
        self._soh_filter._x = (1 - self._coupling_weight) * capacity_new + \
                              self._coupling_weight * capacity_estimate
        
        return soc_estimate, self._soh_filter._x