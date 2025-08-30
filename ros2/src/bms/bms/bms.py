#!/usr/bin/env python3

import rclpy
import sys
import signal
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import numpy as np
from .soc_kalman_filter import StateOfChargeFilter
from .soh_kalman_filter import CapacityFilter
from .dual_kalman_filter import DualKalmanFilter

SOH_FILE = "soh.txt"

class BmsNode(Node):
    def __init__(self):
        super().__init__('soc_estimator')
        
        self.declare_parameter('dt', 1.0)
        self.declare_parameter('initial_capacity', 44.0)
        self.declare_parameter('eol_capactiy_factor', 0.8)
        
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.initial_capacity = self.get_parameter('initial_capacity').get_parameter_value().double_value
        self.eol_capactiy_factor = self.get_parameter('eol_capactiy_factor').get_parameter_value().double_value
        
        self._filter = None
        self.last_timestamp = None

        self._soh = self.read_soh()
        
        self.battery_subscription = self.create_subscription(
            BatteryState,
            'pzem/battery_state',
            self.battery_callback,
            10
        )
        self._msg_counter = 0       
        self.enhanced_battery_publisher = self.create_publisher(BatteryState, 'bms/state', 10)
        self.get_logger().info(f'BMS Node started: SoH={self._soh}, initial Capacity={self.initial_capacity}')
    
    def battery_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_timestamp is not None:
            # Dynamic dt based on actual time difference
            actual_dt = current_time - self.last_timestamp
            if actual_dt > 0 and actual_dt < 10.0:  # Sanity check: dt between 0 and 10 seconds
                self.dt = actual_dt
        
        self.last_timestamp = current_time
        
        voltage_measured = msg.voltage
        current_measured = msg.current

        # initialize state of charge filter
        if self._filter == None:
            soc_kf = StateOfChargeFilter(
                P=np.diag([1e-6, 1e-6, 1.0]),     # state covariance matrix (3x3)
                Q=np.diag([1e-6, 1e-6, 1e-6]),    # process noise covariance (3x3)
                R=np.array([[2.0]]),              # measurement noise covariance (1x1)
                H=np.array([[1.0, -1.0, -1.0]]),  # measurement matrix (1x3)
                ocv=voltage_measured,
                dt=self.dt,
            )
            q_init=self.initial_capacity * self._soh

            self.get_logger().info(f'Initialize SoC Filter: initial SoC={soc_kf.x[0]}')
            self.get_logger().info(f'Initialize Capacity Filter: initial capacity={q_init}')

            soh_kf = CapacityFilter(Q_init=q_init)
            self._filter = DualKalmanFilter(soc_kf, soh_kf)

        soc, capacity = self._filter.step(current_measured, voltage_measured)

        q_new = msg.design_capacity
        q_eol = q_new * self.eol_capactiy_factor

        self._soh = (capacity - q_eol ) / (q_new - q_eol)

        battery_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        if self._soh <= 0.0: 
            battery_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        
        enhanced_msg = BatteryState()
        enhanced_msg.header = msg.header
        enhanced_msg.voltage = msg.voltage
        enhanced_msg.current = msg.current
        enhanced_msg.capacity = capacity
        enhanced_msg.design_capacity = msg.design_capacity
        enhanced_msg.temperature = msg.temperature
        enhanced_msg.present = msg.present
        enhanced_msg.power_supply_status = msg.power_supply_status
        enhanced_msg.power_supply_health = battery_health
        enhanced_msg.power_supply_technology = msg.power_supply_technology
        enhanced_msg.percentage = soc * 100.0
        enhanced_msg.charge = soc * capacity
        
        self.enhanced_battery_publisher.publish(enhanced_msg)

        
        self._msg_counter += 1
        if self._msg_counter % 10 == 0:
            self.get_logger().info( f'SoC: {soc*100:.1f}%, SoH: {self._soh*100:.1f}%, Capacity: {capacity:.2f},')


    def save_soh(self):
        with open(SOH_FILE, 'w') as f:
            f.write(str(self._soh))


    def read_soh(self):
        try:
            with open(SOH_FILE, 'r') as f:
                soh = f.read()
                return float(soh) if soh else 1.0
        except:
            return 1.0
    

def main(args=None):
    rclpy.init(args=args)
    
    soc_estimator = BmsNode()
    
    try:
        rclpy.spin(soc_estimator)
    except KeyboardInterrupt:
        pass
    finally:
        soc_estimator.save_soh()
        soc_estimator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
