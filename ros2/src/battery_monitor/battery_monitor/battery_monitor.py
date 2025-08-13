#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
import threading
from .gui.dashboard import BatteryDashboard
import sys
from PyQt5.QtWidgets import QApplication

class BatteryMonitorNode(Node):
    def __init__(self, dashboard):
        super().__init__('battery_monitor_node')
        self.dashboard = dashboard
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state_enhanced',
            self.battery_callback,
            10
        )
        self.get_logger().info('Battery Monitor Node started')
        
    def battery_callback(self, msg):
        battery_data = {
            'soc': msg.percentage,
            'voltage': msg.voltage,
            'current': msg.current,
            'power': msg.voltage * msg.current if msg.current else 0,
            'capacity': msg.capacity,
            'design_capacity': msg.design_capacity,
            'charging': msg.power_supply_status == 4,  # 4 = charging, 2 = not charging
            'health': msg.power_supply_health
        }
        
        # Calculate SoH
        if msg.design_capacity > 0:
            battery_data['soh'] = (msg.capacity / msg.design_capacity) * 100
        else:
            battery_data['soh'] = 100
            
        # Calculate time remaining with smoothing
        if msg.current > 0 and msg.charge > 0:
            hours_remaining = msg.charge / msg.current
            battery_data['time_remaining'] = hours_remaining
        else:
            battery_data['time_remaining'] = None
            
        self.dashboard.update_battery_data(battery_data)

def main(args=None):
    app = QApplication(sys.argv)
    dashboard = BatteryDashboard()
    
    rclpy.init(args=args)
    node = BatteryMonitorNode(dashboard)
    
    # Run ROS2 in separate thread
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