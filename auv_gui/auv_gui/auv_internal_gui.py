from PyQt6.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import time
from queue import Queue
from pglive.kwargs import Axis
from pglive.sources.data_connector import DataConnector
from pglive.sources.live_axis import LiveAxis
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget


class AnalogWidget:
    def __init__(self, name: str, unit: str, color: str):
        self.name, self.unit, self.color = name, unit, color
        self.value = 0
        self.plot = LiveLinePlot(pen=color)
        self.data_connector = DataConnector(self.plot, max_points=300, update_rate=10)
        self.bottom_axis = LiveAxis("bottom", **{Axis.TICK_FORMAT: Axis.TIME})
        self.chart_view = LivePlotWidget(
            title=f"{name} Plot - Time series @ 10Hz",
            axisItems={"bottom": self.bottom_axis},
        )
        self.chart_view.showGrid(x=True, y=True, alpha=0.3)
        self.chart_view.setLabel("bottom", "Time", units="s")
        self.chart_view.setLabel("left", unit)

        self.chart_view.addItem(self.plot)

        self.label = QLabel(f"{name}: {self.value:.2f}")

    def update(self, queue):
        while not queue.empty():
            datapoint = queue.get()
            self.data_connector.cb_append_data_point(*datapoint)
            self.value = datapoint[0]
        self.label.setText(f"{self.name}: {self.value:.2f}")

    def return_widgets(self):
        return self.label, self.chart_view


class InternalStatusWidget:
    def __init__(self):
        self.internal_widget = QWidget()
        self.internal_layout = QVBoxLayout(self.internal_widget)

        self.internal_status_label = QLabel(parent=self.internal_widget)
        self.internal_layout.addWidget(self.internal_status_label)

        self.current = AnalogWidget("Current", "A", "red")
        self.voltage = AnalogWidget("Voltage", "V", "green")
        self.temperature = AnalogWidget("Temperature", "C", "blue")
        self.pressure = AnalogWidget("Pressure", "Pa", "yellow")

        for analog in [self.current, self.voltage, self.temperature, self.pressure]:
            label, chart = analog.return_widgets()
            self.internal_layout.addWidget(label)
            self.internal_layout.addWidget(chart)

    def get_widget(self):
        return self.internal_widget

    def update(
        self, current: Queue, voltage: Queue, temperature: Queue, pressure: Queue
    ):
        # read all values from the queues
        for analog, queue in zip(
            [self.current, self.voltage, self.temperature, self.pressure],
            [current, voltage, temperature, pressure],
        ):
            analog.update(queue)
