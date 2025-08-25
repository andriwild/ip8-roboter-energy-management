#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        self.capacity_ah = 44.0
        self.rest_current_a = 4.0
        self.drive_current_a = 8.0
        self.publish_rate_hz = 1.0
        self.movement_linear_eps = 0.01
        self.movement_angular_eps = 0.01
        self.soc_bp = np.array([
            0.0, 0.02941225486521637, 0.058824117602975678, 0.088234411830912074,
            0.11764544129782739, 0.14705916664099683, 0.17647249985671409,
            0.20588441161041271, 0.23529446075872762, 0.26470632349653733,
            0.29411823525027914, 0.32352867652607242, 0.35293995107270271,
            0.38235014726883843, 0.41176029444904005, 0.44117318652141591,
            0.47058406894053362, 0.499995196439295, 0.52940931390989365,
            0.55882098058390717, 0.58823308840130228, 0.61764500015497259,
            0.64705848041845848, 0.676470686267719, 0.70588284310105109,
            0.73529245110594532, 0.76470269631794507, 0.794116372645157,
            0.82352598065004556, 0.85293740224440528, 0.88234784352013151,
            0.91176201000661383, 0.94117691173195439, 0.97059034297941116, 1.0
        ])
        self.ocv_data = np.array([
            11.752391930195678, 11.833322582740049, 11.90371280791882,
            11.965139959216454, 12.028986538595934, 12.084628842590067,
            12.14181741678707, 12.196094314814118, 12.245734791002235, 12.30382544243438,
            12.358777190557541, 12.423025535620173, 12.477014755585477,
            12.533807528951268, 12.581739651537347, 12.641493452761409,
            12.698566399009771, 12.748487707139224, 12.803656292580287,
            12.857705819017156, 12.913385665848981, 12.965942365524132,
            13.006653559166399, 13.065629493277934, 13.110532716204975,
            13.164051379589417, 13.204274071040635, 13.255367871098075,
            13.303253359407229, 13.352845040458794, 13.400354727913175,
            13.449028887411073, 13.499288785741127, 13.549973944152651,
            13.603600034043087
        ])
        self.is_moving = False
        self.last_twist_time = self.get_clock().now()
        self.charge_ah = 32
        self.last_time = self.get_clock().now()
        self.batt_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

    def cmd_cb(self, msg: Twist):
        l = msg.linear
        a = msg.angular
        moving = (
            abs(l.x) > self.movement_linear_eps or
            abs(l.y) > self.movement_linear_eps or
            abs(l.z) > self.movement_linear_eps or
            abs(a.x) > self.movement_angular_eps or
            abs(a.y) > self.movement_angular_eps or
            abs(a.z) > self.movement_angular_eps
        )
        self.is_moving = moving
        self.last_twist_time = self.get_clock().now()

    def ocv_from_soc(self, soc: float) -> float:
        s = float(np.clip(soc, 0.0, 1.0))
        return float(np.interp(s, self.soc_bp, self.ocv_data))

    def on_timer(self):
        now = self.get_clock().now()
        dt = max(0.0, (now - self.last_time).nanoseconds * 1e-9)
        if (now - self.last_twist_time).nanoseconds * 1e-9 > 1.0:
            self.is_moving = False
        i = self.drive_current_a if self.is_moving else self.rest_current_a
        if self.charge_ah <= 0.0:
            i = 0.0
        used_ah = i * (dt / 3600.0)
        self.charge_ah = max(0.0, self.charge_ah - used_ah)
        soc = self.charge_ah / self.capacity_ah if self.capacity_ah > 0.0 else 0.0
        v = self.ocv_from_soc(soc)
        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.voltage = v
        msg.current = i
        msg.charge = float(self.charge_ah)
        msg.capacity = float(self.capacity_ah)
        msg.design_capacity = float(self.capacity_ah)
        msg.percentage = float(soc)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if i > 0.0 else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True
        msg.location = 'base_link'
        msg.serial_number = 'SIM-001'
        self.batt_pub.publish(msg)
        self.voltage_pub.publish(Float32(data=float(v)))
        self.get_logger().info(f"SOC={soc:.3f} Voltage={v:.2f}V Current={i:.2f}A Moving={self.is_moving}")
        if self.charge_ah <= 0.0:
            self.get_logger().warn('Battery depleted')
        self.last_time = now

def main():
    rclpy.init()
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
