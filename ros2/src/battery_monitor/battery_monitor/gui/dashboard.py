#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QPalette, QColor, QFont
from .widgets import BatteryWidget, MetricWidget, TimeRemainingWidget, ChargingIndicator

class BatteryDashboard(QWidget):
    data_received = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        self.time_remaining_buffer = []  # For smoothing
        self.init_ui()
        self.data_received.connect(self.update_display)
        
    def init_ui(self):
        self.setWindowTitle('Battery Monitor')
        self.setStyleSheet("""
            QWidget {
                background-color: #1a1a1a;
                color: #ffffff;
            }
        """)
        
        # Main layout
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # Title
        title = QLabel('BATTERY MONITOR')
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            font-size: 24px;
            font-weight: bold;
            color: #00ff88;
            padding: 10px;
            letter-spacing: 2px;
        """)
        main_layout.addWidget(title)
        
        # Top section - Battery and charging
        top_layout = QHBoxLayout()
        top_layout.setSpacing(30)
        
        self.charging_indicator = ChargingIndicator()
        self.battery_widget = BatteryWidget()
        
        top_layout.addStretch()
        top_layout.addWidget(self.charging_indicator)
        top_layout.addWidget(self.battery_widget)
        top_layout.addStretch()
        
        main_layout.addLayout(top_layout)
        
        # Time remaining
        self.time_widget = TimeRemainingWidget()
        main_layout.addWidget(self.time_widget)
        
        # Metrics grid
        metrics_layout = QGridLayout()
        metrics_layout.setSpacing(20)
        
        self.voltage_widget = MetricWidget('Voltage', 'V', '#00aaff')
        self.current_widget = MetricWidget('Current', 'A', '#ffaa00')
        self.power_widget = MetricWidget('Power', 'Wh', '#ff00aa')
        self.soh_widget = MetricWidget('SoH', '%', '#00ff88')
        
        metrics_layout.addWidget(self.voltage_widget, 0, 0)
        metrics_layout.addWidget(self.current_widget, 0, 1)
        metrics_layout.addWidget(self.power_widget, 1, 0)
        metrics_layout.addWidget(self.soh_widget, 1, 1)
        
        main_layout.addLayout(metrics_layout)
        main_layout.addSpacing(20)
        main_layout.addStretch()
        
        self.setLayout(main_layout)
        self.setFixedSize(400, 550)
        
    def update_battery_data(self, data):
        self.data_received.emit(data)
        
    @pyqtSlot(dict)
    def update_display(self, data):
        self.battery_widget.set_level(data['soc'])
        self.voltage_widget.set_value(f"{data['voltage']:.1f}")
        self.power_widget.set_value(f"{(data['power']):.1f}")
        self.soh_widget.set_value(f"{int(data['soh'])}")
        self.charging_indicator.set_charging(data['charging'])


        current = data['current']
        if data['charging']:
            current = -current
        self.current_widget.set_value(f"{current:.1f}")
            
        if 'time_remaining' in data:
            if data['time_remaining'] and not data['charging']:
                # Smooth time remaining with moving average
                self.time_remaining_buffer.append(data['time_remaining'])
                if len(self.time_remaining_buffer) > 10:  # Keep last 10 values
                    self.time_remaining_buffer.pop(0)
                
                avg_time = sum(self.time_remaining_buffer) / len(self.time_remaining_buffer)
                hours = int(avg_time)
                minutes = int((avg_time - hours) * 60)
                self.time_widget.set_time(hours, minutes)
            else:
                self.time_widget.set_time(None, None)
                self.time_remaining_buffer.clear()
                