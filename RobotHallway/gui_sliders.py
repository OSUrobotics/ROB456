
# Get the windowing/drawing packages
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QGroupBox, QSlider, QLabel, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt5.QtCore import Qt, QSize, QPoint
from PyQt5.QtGui import QPainter, QBrush, QPen, QFont, QColor

import numpy as np

# A helper class that implements a slider with given start and end float value; displays values
class SliderFloatDisplay(QWidget):
    gui = None

    def __init__(self, name, low, high, initial_value, ticks=100):
        """
        Give me a name, the low and high values, and an initial value to set
        :param name: Name displayed on slider
        :param low: Minimum value slider returns
        :param high: Maximum value slider returns
        :param initial_value: Should be a value between low and high
        :param ticks: Resolution of slider - all sliders are integer/fixed number of ticks
        """
        # Save input values
        self.name = name
        self.low = low
        self.range = high - low
        self.ticks = ticks

        # I'm a widget with a text value next to a slider
        QWidget.__init__(self)
        layout = QHBoxLayout()
        self.setLayout(layout)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(ticks)
        # call back - call change_value when slider changed
        self.slider.valueChanged.connect(self.change_value)

        self.display = QLabel()
        self.set_value(initial_value)
        self.change_value()

        layout.addWidget(self.display)
        layout.addWidget(self.slider)

    # Use this to get the value between low/high
    def value(self):
        """Return the current value of the slider"""
        val = (self.slider.value() / self.ticks) * self.range + self.low
        # print(f"Get value: name {self.name} value {val} text {self.display.text()}")
        return val

    # Called when the value changes - resets text
    def change_value(self):
        if SliderFloatDisplay.gui is not None:
            SliderFloatDisplay.gui.update_simulation_parameters()
        self.display.setText('{0}: {1:.3f}'.format(self.name, self.value()))
        try:
            self.gui.repaint()
        except AttributeError:
            pass

    # Use this to change the value (does clamping)
    def set_value(self, value_f):
        value_tick = self.ticks * (value_f - self.low) / self.range
        value_tick = min(max(0, value_tick), self.ticks)
        self.slider.setValue(int(value_tick))
        self.display.setText('{0}: {1:.3f}'.format(self.name, self.value()))

        # print(f"Set value: name {self.name} value {self.slider.value()} text {self.display.text()}")


# A helper class that implements a slider with given start and end value; displays values
class SliderIntDisplay(QWidget):
    gui = None

    def __init__(self, name, min_ticks, max_ticks, initial_value):
        """
        Give me a name, the low and high values, and an initial value to set
        :param name: Name displayed on slider
        :param min_ticks: Minimum value slider returns
        :param max_ticks: Maximum value slider returns
        :param initial_value: Should be a value between low and high
        """
        # Save input values
        self.name = name
        self.min_ticks = min_ticks
        self.max_ticks = max_ticks

        # I'm a widget with a text value next to a slider
        QWidget.__init__(self)
        layout = QHBoxLayout()
        self.setLayout(layout)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(min_ticks)
        self.slider.setMaximum(max_ticks)
        # call back - call change_value when slider changed
        self.slider.valueChanged.connect(self.change_value)

        self.display = QLabel()
        self.set_value(initial_value)
        self.change_value()

        layout.addWidget(self.display)
        layout.addWidget(self.slider)

    # Use this to get the value between low/high
    def value(self):
        """Return the current value of the slider"""
        return self.slider.value()

    # Called when the value changes - resets text
    def change_value(self):
        if SliderIntDisplay.gui is not None:
            SliderIntDisplay.gui.repaint()
        self.display.setText('{}: {}'.format(self.name, self.value()))

    # Use this to change the value (does clamping)
    def set_value(self, value_tick):
        value_tick = min(max(self.min_ticks, value_tick), self.max_ticks)
        self.slider.setValue(int(value_tick))
        self.change_value()
