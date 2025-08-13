#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import Qt, QRect, QTimer, QPropertyAnimation, QEasingCurve, pyqtProperty
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen, QLinearGradient, QFont

class BatteryWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.level = 0
        self.setFixedSize(120, 60)
        
    def set_level(self, level):
        self.level = round(max(0, min(100, level)))
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Battery body
        body_rect = QRect(10, 10, 90, 40)
        painter.setPen(QPen(QColor('#444'), 2))
        painter.setBrush(QBrush(QColor('#222')))
        painter.drawRoundedRect(body_rect, 5, 5)
        
        # Battery terminal
        terminal_rect = QRect(100, 22, 8, 16)
        painter.drawRoundedRect(terminal_rect, 2, 2)
        
        # Fill level
        if self.level > 0:
            fill_width = int((body_rect.width() - 6) * self.level / 100)
            fill_rect = QRect(13, 13, fill_width, 34)
            
            # Color based on level
            if self.level > 60:
                color = QColor('#00ff88')
            elif self.level > 30:
                color = QColor('#ffaa00')
            else:
                color = QColor('#ff4444')
                
            gradient = QLinearGradient(fill_rect.topLeft(), fill_rect.bottomLeft())
            gradient.setColorAt(0, color.lighter(120))
            gradient.setColorAt(1, color)
            
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(gradient))
            painter.drawRoundedRect(fill_rect, 3, 3)
        
        # Percentage text
        painter.setPen(QPen(QColor('#fff'), 1))
        painter.setFont(QFont('Arial', 12, QFont.Bold))
        painter.drawText(body_rect, Qt.AlignCenter, f"{self.level}%")

class MetricWidget(QWidget):
    def __init__(self, label, unit, color):
        super().__init__()
        self.color = color
        self.init_ui(label, unit)
        
    def init_ui(self, label, unit):
        layout = QVBoxLayout()
        layout.setSpacing(5)
        layout.setContentsMargins(15, 15, 15, 15)
        
        # Label
        label_widget = QLabel(label + f" ({unit})")
        label_widget.setAlignment(Qt.AlignCenter)
        label_widget.setStyleSheet(f"""
            font-size: 14px;
            color: #bbb;
            font-weight: normal;
        """)
        
        # Value
        self.value_label = QLabel('--')
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet(f"""
            font-size: 26px;
            color: {self.color};
            font-weight: bold;
            font-family: 'Arial', sans-serif;
            padding: 5px;
        """)
        
        layout.addWidget(label_widget)
        layout.addWidget(self.value_label)
        
        self.setLayout(layout)
        self.setStyleSheet("""
            QWidget {
                background-color: #2a2a2a;
                border-radius: 2px;
            }
        """)
        self.setMinimumSize(180, 100)
        self.setMaximumHeight(120)
        
    def set_value(self, value):
        self.value_label.setText(str(value))

class TimeRemainingWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        
    def init_ui(self):
        layout = QVBoxLayout()
        
        label = QLabel('Time Remaining')
        label.setAlignment(Qt.AlignCenter)
        label.setStyleSheet("""
            font-size: 16px;
            color: #888;
            font-weight: bold;
        """)
        
        self.time_label = QLabel('--:--')
        self.time_label.setAlignment(Qt.AlignCenter)
        self.time_label.setStyleSheet("""
            font-size: 48px;
            color: #00aaff;
            font-weight: bold;
            font-family: 'Courier New', monospace;
        """)
        
        layout.addWidget(label)
        layout.addWidget(self.time_label)
        
        self.setLayout(layout)
        self.setStyleSheet("""
            QWidget {
                background-color: #2a2a2a;
                border-radius: 10px;
                padding: 15px;
            }
        """)
        
    def set_time(self, hours, minutes):
        if hours is not None and minutes is not None:
            self.time_label.setText(f"{hours:02d}:{minutes:02d}")
        else:
            self.time_label.setText("--:--")

class ChargingIndicator(QWidget):
    def __init__(self):
        super().__init__()
        self._animation_value = 0
        self.charging = False
        self.setFixedSize(60, 60)
        
        self.animation = QPropertyAnimation(self, b"animation_value")
        self.animation.setDuration(1500)
        self.animation.setStartValue(0)
        self.animation.setEndValue(100)
        self.animation.setLoopCount(-1)
        self.animation.setEasingCurve(QEasingCurve.InOutSine)
        
    @pyqtProperty(int)
    def animation_value(self):
        return self._animation_value
        
    @animation_value.setter
    def animation_value(self, value):
        self._animation_value = value
        self.update()
        
    def set_charging(self, charging):
        self.charging = charging
        if charging:
            self.animation.start()
        else:
            self.animation.stop()
            self._animation_value = 0
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        center = self.rect().center()
        
        if self.charging:
            # Animated glow effect when charging
            opacity = 100 + int(155 * self._animation_value / 100)
            color = QColor(0, 255, 100, opacity)
            
            # Outer glow
            painter.setPen(Qt.NoPen)
            painter.setBrush(QBrush(color))
            painter.drawEllipse(center, 25, 25)
            
            # Lightning bolt - white when charging
            painter.setPen(QPen(QColor('#fff'), 3))
            painter.drawLine(center.x() - 5, center.y() - 10, center.x() + 2, center.y())
            painter.drawLine(center.x() + 2, center.y(), center.x() - 2, center.y())
            painter.drawLine(center.x() - 2, center.y(), center.x() + 5, center.y() + 10)
        else:
            # Not charging - gray circle with gray lightning
            painter.setPen(QPen(QColor('#333'), 2))
            painter.setBrush(QBrush(QColor('#1a1a1a')))
            painter.drawEllipse(center, 22, 22)
            
            # Gray lightning bolt when not charging
            painter.setPen(QPen(QColor('#555'), 2))
            painter.drawLine(center.x() - 5, center.y() - 10, center.x() + 2, center.y())
            painter.drawLine(center.x() + 2, center.y(), center.x() - 2, center.y())
            painter.drawLine(center.x() - 2, center.y(), center.x() + 5, center.y() + 10)