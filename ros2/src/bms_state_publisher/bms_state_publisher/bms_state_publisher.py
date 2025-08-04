#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import math
import random

class LeadAcidBatterySimulator(Node):
    def __init__(self):
        super().__init__('lead_acid_battery_simulator')
        
        # Publisher für BatteryState Nachrichten
        self.battery_publisher = self.create_publisher(BatteryState, 'battery_state', 10)
        
        # Timer für 1Hz (jede Sekunde)
        self.timer = self.create_timer(1.0, self.publish_battery_state)
        
        # Batterie-Parameter für Blei-Säure-Batterie
        self.nominal_voltage = 12.0  # V (typisch für 12V Blei-Säure-Batterie)
        self.max_voltage = 14.4      # V (Ladespannung)
        self.min_voltage = 10.8      # V (Entladeschlussspannung)
        self.capacity = 10.0        # Ah (Ampere-Stunden)
        self.current_capacity = 8.0 # Ah (aktuelle Kapazität, 80% geladen)
        
        # Simulationsparameter
        self.charging = False        # Lade-/Entladezustand
        self.charge_current = 10.0   # A (Ladestrom)
        self.discharge_current = 5.0 # A (Entladestrom)
        self.internal_resistance = 0.01  # Ohm
        
        # Zustandswechsel-Timer (alle 30-60 Sekunden)
        self.state_change_counter = 0
        self.state_change_interval = random.randint(30, 60)
        
        self.get_logger().info('Lead Acid Battery Simulator gestartet')
        
    def publish_battery_state(self):
        # BatteryState Nachricht erstellen
        msg = BatteryState()
        
        # Header setzen
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'battery_frame'
        
        # Batterie-Simulation aktualisieren
        self.update_battery_simulation()
        
        # Spannung basierend auf Ladezustand berechnen
        soc = self.current_capacity / self.capacity  # State of Charge (0-1)
        
        if self.charging:
            # Beim Laden: höhere Spannung
            msg.voltage = self.min_voltage + (self.max_voltage - self.min_voltage) * soc
            msg.current = -self.charge_current  # Negativ = Laden
        else:
            # Beim Entladen: Spannung fällt mit Entladung
            msg.voltage = self.min_voltage + (self.nominal_voltage - self.min_voltage) * soc
            msg.current = self.discharge_current  # Positiv = Entladen
        
        # Spannungsabfall durch internen Widerstand
        msg.voltage -= abs(msg.current) * self.internal_resistance
        
        # Weitere Parameter setzen
        msg.charge = self.current_capacity * 3600  # Ah -> Coulomb (As)
        msg.capacity = self.capacity * 3600        # Ah -> Coulomb (As)
        msg.design_capacity = self.capacity * 3600 # Ah -> Coulomb (As)
        msg.percentage = soc * 100.0               # Prozent
        
        # Temperatur simulieren (Blei-Säure erwärmt sich beim Laden/Entladen)
        base_temp = 25.0  # °C
        temp_variation = abs(msg.current) * 0.5  # Erwärmung durch Strom
        msg.temperature = base_temp + temp_variation + random.uniform(-2, 2)
        
        # Power calculation
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.charging else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True
        
        # Nachricht veröffentlichen
        self.battery_publisher.publish(msg)
        
        # Log-Ausgabe
        status = "LADEN" if self.charging else "ENTLADEN"
        self.get_logger().info(
            f'{status}: {msg.voltage:.2f}V, {msg.current:.1f}A, '
            f'{msg.percentage:.1f}%, {msg.temperature:.1f}°C'
        )
    
    def update_battery_simulation(self):
        """Aktualisiert die Batterie-Simulation"""
        
        # Zeitschritt (1 Sekunde)
        dt = 1.0 / 3600.0  # Stunden
        
        if self.charging:
            # Laden: Kapazität erhöhen
            if self.current_capacity < self.capacity:
                self.current_capacity += self.charge_current * dt
                self.current_capacity = min(self.current_capacity, self.capacity)
        else:
            # Entladen: Kapazität verringern
            if self.current_capacity > 0:
                self.current_capacity -= self.discharge_current * dt
                self.current_capacity = max(self.current_capacity, 0)
        
        # Zustandswechsel simulation (Laden/Entladen)
        self.state_change_counter += 1
        if self.state_change_counter >= self.state_change_interval:
            self.state_change_counter = 0
            self.state_change_interval = random.randint(30, 60)
            
            # Zustand wechseln basierend auf Ladezustand
            soc = self.current_capacity / self.capacity
            
            if soc >= 0.95:  # Bei 95% auf Entladen wechseln
                self.charging = False
            elif soc <= 0.20:  # Bei 20% auf Laden wechseln
                self.charging = True
            else:
                # Zufälliger Wechsel
                self.charging = not self.charging
            
            status = "LADEN" if self.charging else "ENTLADEN"
            self.get_logger().info(f'Zustandswechsel: {status}')

def main(args=None):
    rclpy.init(args=args)
    
    battery_simulator = LeadAcidBatterySimulator()
    
    try:
        rclpy.spin(battery_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        battery_simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()