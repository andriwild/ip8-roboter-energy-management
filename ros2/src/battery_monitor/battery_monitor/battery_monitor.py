#!/usr/bin/env python3

import rclpy
import os
import csv
from collections import deque
from statistics import mean
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
import threading
from .gui.dashboard import BatteryDashboard
import sys
from PyQt5.QtWidgets import QApplication
from datetime import datetime

EOL_FACTOR = 0.8
STACK_SIZE = 10

class BatteryMonitorNode(Node):
    def __init__(self, dashboard):
        super().__init__('battery_monitor')
        self.dashboard = dashboard

        self._V_nominal = 12 # nominal voltage of battery (V)
        self._P_avg = 60     # average power consumption for drive (Wh / m)
        self._v_avg = 4      # average speed (m/s)
        self._safety_factor = 0.95 # unpredictable things

        self._time_stack = deque(maxlen=STACK_SIZE)

        self.subscription = self.create_subscription(
            BatteryState,
            '/bms/state',
            self.battery_callback,
            10
        )
        self.get_logger().info('Battery Monitor Node started')


    def estimate_remaining_time(self, current, charge) -> str:
        if current > 0 and charge > 0:
            hours_remaining = charge / current
            self._time_stack.append(hours_remaining)

            if len(self._time_stack) == STACK_SIZE:
                avg_time = mean(self._time_stack)
                hours = int(avg_time)
                minutes = int((avg_time - hours) * 60)
                return f"{hours}:{minutes}"

        return "--:--"
        

    def estimate_remaining_range(self, soc, capacity) -> float:
        energy_remaining = soc * self._V_nominal * capacity
        range = energy_remaining / self._P_avg
        return range * self._safety_factor


    def battery_callback(self, msg):

        # Calculate SoH
        q_new = msg.design_capacity
        q_eol = q_new * EOL_FACTOR
        soh = (msg.capacity - q_eol ) / (q_new - q_eol)

        battery_data = {
            'soc': msg.percentage,
            'voltage': msg.voltage,
            'current': msg.current,
            'power': msg.voltage * msg.current,
            'capacity': msg.capacity,
            'design_capacity': msg.design_capacity,
            'charging': msg.power_supply_status == 4,  # 4 = charging, 2 = not charging
            'health': msg.power_supply_health,
            'range': self.estimate_remaining_range(msg.percentage, msg.capacity),
            'time': self.estimate_remaining_time(msg.current, msg.charge),
            'soh': round(soh * 100)
        }
        
        self.dashboard.update_battery_data(battery_data)



def main(args=None):
    app = QApplication(sys.argv)
    dashboard = BatteryDashboard()
    
    rclpy.init(args=args)
    node = BatteryMonitorNode(dashboard)
    
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    dashboard.show()
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()