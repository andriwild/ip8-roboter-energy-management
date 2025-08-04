#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import numpy as np
from .kalman_filter import ExtendedKalmanFilter

class SOCEstimatorNode(Node):
    def __init__(self):
        super().__init__('soc_estimator')
        
        self.declare_parameter('dt', 1.0)
        self.declare_parameter('initial_soc', 0.5)
        
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        initial_soc = self.get_parameter('initial_soc').get_parameter_value().double_value
        
        self.capacity_ah = 44.0
        
        # Initialize Extended Kalman Filter with fixed parameters
        try:
            self.kf = ExtendedKalmanFilter(
                P=np.diag([1e-6, 1e-6, 1.0]),     # state covariance matrix (3x3)
                Q=np.diag([1e-4, 1e-4, 1e-4]),    # process noise covariance (3x3)
                R=np.array([[0.4]]),              # measurement noise covariance (1x1)
                H=np.array([[1.0, -1.0, -1.0]]),  # measurement matrix (1x3)
                dt=self.dt,
                initial_soc=initial_soc
            )
            self.get_logger().info('Extended Kalman Filter successfully initialized')
        except Exception as e:
            self.get_logger().error(f'Error initializing Kalman Filter: {e}')
            raise
        
        self.battery_subscription = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_callback,
            10
        )
        
        self.enhanced_battery_publisher = self.create_publisher(BatteryState, 'battery_state_enhanced', 10)
        
        self.last_timestamp = None
        
        self.get_logger().info('SOC Estimator Node started')
        self.get_logger().info(f'Capacity: {self.capacity_ah} Ah, dt: {self.dt} s, Initial SOC: {initial_soc}')
    
    def battery_callback(self, msg):
        try:
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            if self.last_timestamp is not None:
                # Dynamic dt based on actual time difference
                actual_dt = current_time - self.last_timestamp
                if actual_dt > 0 and actual_dt < 10.0:  # Sanity check: dt between 0 and 10 seconds
                    self.dt = actual_dt
            
            self.last_timestamp = current_time
            
            voltage_measured = msg.voltage
            current_measured = msg.current
            
            self.kf.predict(current_measured, self.capacity_ah)
            measurements = [voltage_measured, current_measured]

            self.kf.update(measurements)
            
            estimated_soc = self.kf.x[0]
            estimated_vrc1 = self.kf.x[1]
            estimated_vrc2 = self.kf.x[2]
            
            enhanced_msg = BatteryState()
            
            enhanced_msg.header = msg.header
            enhanced_msg.voltage = msg.voltage
            enhanced_msg.current = msg.current
            enhanced_msg.charge = msg.charge
            enhanced_msg.capacity = msg.capacity
            enhanced_msg.design_capacity = msg.design_capacity
            enhanced_msg.temperature = msg.temperature
            enhanced_msg.present = msg.present
            enhanced_msg.power_supply_status = msg.power_supply_status
            enhanced_msg.power_supply_health = msg.power_supply_health
            enhanced_msg.power_supply_technology = msg.power_supply_technology
            
            # Replace SOC with Kalman Filter estimation
            enhanced_msg.percentage = float(estimated_soc * 100.0)
            
            # Recalculate charge based on estimated SOC
            enhanced_msg.charge = float(estimated_soc * self.capacity_ah * 3600)  # Ah -> Coulomb
            
            self.enhanced_battery_publisher.publish(enhanced_msg)
            
            # Logging (every 10 messages)
            if not hasattr(self, '_msg_counter'):
                self._msg_counter = 0
            
            self._msg_counter += 1
            if self._msg_counter % 10 == 0:
                self.get_logger().info(
                    f'SOC estimated: {estimated_soc:.3f} ({estimated_soc*100:.1f}%), '
                    f'V_RC1: {estimated_vrc1:.3f}V, V_RC2: {estimated_vrc2:.3f}V, '
                    f'U_meas: {voltage_measured:.2f}V, I_meas: {current_measured:.2f}A'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error in battery_callback: {e}')
    

def main(args=None):
    rclpy.init(args=args)
    
    soc_estimator = SOCEstimatorNode()
    
    try:
        rclpy.spin(soc_estimator)
    except KeyboardInterrupt:
        pass
    finally:
        soc_estimator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()