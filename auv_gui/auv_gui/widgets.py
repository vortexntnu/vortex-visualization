from queue import Queue

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pglive.kwargs import Axis
from pglive.sources.data_connector import DataConnector
from pglive.sources.live_axis import LiveAxis
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget
from PyQt6.QtGui import QOpenGLContext, QVector3D
from PyQt6.QtWidgets import (
    QLabel,
    QVBoxLayout,
    QWidget,
)


class OpenGLPlotWidget(QWidget):
    def __init__(self, gui_node, parent=None):
        """Initialize the OpenGL 3D plot."""
        super().__init__(parent)
        self.opengl_context = QOpenGLContext.currentContext()
        if not self.opengl_context:
            print("Warning: No OpenGL context available!")

        self.gui_node = gui_node

        self.follow_mode = False

        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=10)
        self.view.setBackgroundColor("gray")

        self.grid = gl.GLGridItem()
        self.grid.setSize(20, 20, 1)
        self.view.addItem(self.grid)

        self.trajectory = gl.GLLinePlotItem(
            color=(0, 0, 255, 255), width=2
        )  # Blue line

        self.line = gl.GLLinePlotItem(color=(200, 0, 0, 255), width=2)  # Red line

        self.current_position_dot = gl.GLScatterPlotItem(
            pos=np.array([[0, 0, 0]]),
            color=(255, 0, 0, 255),
            size=5,  # Red dot
        )

        self.view.addItem(self.trajectory)
        self.view.addItem(self.current_position_dot)
        self.view.addItem(self.line)

        follow_button = pg.QtWidgets.QPushButton("Toggle Follow Mode")
        follow_button.clicked.connect(self.toggle_follow_mode)

        layout = pg.QtWidgets.QVBoxLayout()
        layout.addWidget(follow_button)
        layout.addWidget(self.view)
        self.setLayout(layout)

    def update_plot(self, x_data, y_data, z_data):
        """Update the 3D plot with new trajectory data."""
        if len(x_data) > 1:
            points = np.vstack((x_data, y_data, z_data)).T
            self.trajectory.setData(pos=points)

            self.current_position_dot.setData(pos=points[-1:])

            if self.follow_mode:
                vec = QVector3D()
                vec.setX(points[-1][0])
                vec.setY(points[-1][1])
                vec.setZ(points[-1][2])
                self.view.setCameraPosition(pos=vec)

    def clear_plot(self):
        """Clear the 3D plot by resetting the trajectory and position dot."""
        self.trajectory.setData(pos=np.empty((0, 3), dtype=np.float32))
        self.gui_node.xpos_data.clear()
        self.gui_node.ypos_data.clear()
        self.gui_node.zpos_data.clear()

        self.line.setData(pos=np.empty((0, 3), dtype=np.float32))

    def toggle_follow_mode(self):
        """Toggle follow mode on or off."""
        self.follow_mode = not self.follow_mode

    def plot_points_and_line(self, points):
        """Plots two points and a line connecting them.

        Parameters
        ----------
            points (list): A list containing two points, where each point is a list [x, y, z].

        """
        if len(points) != 2:
            raise ValueError("Input list must contain exactly two points.")

        # Extract the two points
        point1 = points[0]
        point2 = points[1]
        point2[2] *= -1  # Invert the z-axis

        # Create the line between the two points
        line_points = np.array([point1, point2])

        # Plot the two points and the line
        self.line.setData(pos=line_points)


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
        self.pressure = AnalogWidget("Pressure", "hPa", "yellow")

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
