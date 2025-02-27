#!/usr/bin/env python3

import sys
from threading import Thread

from ament_index_python.packages import get_package_share_directory
from data_manager import DataManager
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QDoubleValidator, QIcon
from PyQt6.QtWidgets import (
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QMainWindow,
    QPushButton,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from vortex_utils.python_utils import H264Decoder
from widgets import InternalStatusWidget, OpenGLPlotWidget


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.data_manager = DataManager()
        self.setWindowTitle("Vortex GUI")
        self.setGeometry(100, 100, 600, 400)
        self.setWindowIcon(
            QIcon(get_package_share_directory("auv_gui") + "/resources/vortex_logo.png")
        )

        self.init_ui()

        # Initialize OpenGL plot and video decoder
        self.decoder = H264Decoder()
        self.decoder_thread = Thread(target=self.decoder.start, daemon=True)
        self.decoder_thread.start()
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # Timer to update GUI
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # Update every 100ms

    def init_ui(self):
        """Initialize the GUI layout and widgets."""
        tabs = QTabWidget()
        tabs.setTabPosition(QTabWidget.TabPosition.North)
        tabs.setMovable(True)

        # --- Mission Tab ---
        mission_widget = QWidget()
        mission_layout = QGridLayout(mission_widget)
        inputs_layout = QHBoxLayout()
        buttons_layout = QHBoxLayout()

        # --- Position Section ---
        self.plot_canvas = OpenGLPlotWidget(self.data_manager)
        mission_layout.addWidget(self.plot_canvas, 0, 0, 1, 3)
        mission_layout.setRowStretch(0, 5)
        mission_layout.setColumnStretch(0, 5)

        # Create the labels for current position and internal status
        self.current_pos_label = QLabel("<b>Current Position:</b> Not Available")
        self.current_pos_label.setStyleSheet("font-size: 20px;")
        self.internal_status_label = QLabel("<b>Internal Status:</b> Not Available")
        self.internal_status_label.setStyleSheet("font-size: 20px;")

        status_layout = QVBoxLayout()
        status_layout.addWidget(self.current_pos_label)
        status_layout.addWidget(self.internal_status_label)
        mission_layout.addLayout(status_layout, 0, 4)

        # Inputs for X, Y, Z
        self.x_input = QLineEdit()
        self.x_input.setPlaceholderText("X")
        self.x_input.setValidator(QDoubleValidator())

        self.y_input = QLineEdit()
        self.y_input.setPlaceholderText("Y")
        self.y_input.setValidator(QDoubleValidator())

        self.z_input = QLineEdit()
        self.z_input.setPlaceholderText("Z")
        self.z_input.setValidator(QDoubleValidator())

        # Inputs for roll, pitch, yaw
        self.roll_input = QLineEdit()
        self.roll_input.setPlaceholderText("Roll")
        self.roll_input.setValidator(QDoubleValidator())

        self.pitch_input = QLineEdit()
        self.pitch_input.setPlaceholderText("Pitch")
        self.pitch_input.setValidator(QDoubleValidator())

        self.yaw_input = QLineEdit()
        self.yaw_input.setPlaceholderText("Yaw")
        self.yaw_input.setValidator(QDoubleValidator())

        # Add input widgets
        inputs_layout.addWidget(QLabel("Waypoint:"))
        inputs_layout.addWidget(self.x_input)
        inputs_layout.addWidget(self.y_input)
        inputs_layout.addWidget(self.z_input)

        inputs_layout.addWidget(QLabel("Axes:"))
        inputs_layout.addWidget(self.roll_input)
        inputs_layout.addWidget(self.pitch_input)
        inputs_layout.addWidget(self.yaw_input)

        # Buttons
        self.add_button = QPushButton("Add Waypoint")

        self.send_button_ref = QPushButton("Send Mission (ReferenceFilter)")
        self.send_button_ref.setEnabled(False)

        self.send_button_los = QPushButton("Send Mission (LOS)")
        self.send_button_los.setEnabled(False)

        self.clear_button = QPushButton("Clear Waypoints")
        self.clear_button.clicked.connect(self.clear_waypoints)

        self.cancel_button = QPushButton("Cancel Mission")

        self.clear_plot_button = QPushButton("Clear Plot")

        # Add buttons to layout
        buttons_layout.addWidget(self.add_button)
        buttons_layout.addWidget(self.send_button_ref)
        buttons_layout.addWidget(self.send_button_los)
        buttons_layout.addWidget(self.clear_button)
        buttons_layout.addWidget(self.cancel_button)
        buttons_layout.addWidget(self.clear_plot_button)

        # List widget to display waypoints
        self.waypoint_list = QListWidget()
        self.waypoint_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        self.waypoint_list.itemSelectionChanged.connect(self.update_button_states)

        # Add layouts to mission section
        mission_layout.addLayout(inputs_layout, 1, 0, 1, 4)
        mission_layout.addLayout(buttons_layout, 2, 0, 1, 4)
        mission_layout.addWidget(self.waypoint_list, 3, 0, 1, 4)

        tabs.addTab(mission_widget, "Mission")

        # --- Internal Status Tab ---
        internal_status_tab = InternalStatusWidget(self.data_manager)
        tabs.addTab(internal_status_tab.get_widget(), "Internal")

        # --- Image Tab ---
        image_tab_widget = QWidget()
        image_layout = QVBoxLayout(image_tab_widget)
        image_layout.addWidget(self.video_label)
        tabs.addTab(image_tab_widget, "Camera Feed")

        self.setCentralWidget(tabs)

    def update_gui(self):
        """Periodically updates GUI elements."""
        data = self.data_manager.get_latest("pose")
        if data:
            position_text = (
                f"<b>Current Position:</b><br>X: {data['x']:.2f}<br>"
                f"Y: {data['y']:.2f}<br>Z: {data['z']:.2f}"
            )
            self.current_pos_label.setText(position_text)

        internal_status = self.data_manager.get_latest("internal_status")
        if internal_status:
            status_text = (
                f"<b>Internal Status:</b><br>"
                f"Current: {internal_status['current']:.2f} A<br>"
                f"Voltage: {internal_status['voltage']:.2f} V<br>"
                f"Temperature: {internal_status['temperature']:.2f} °C<br>"
                f"Pressure: {internal_status['pressure']:.2f} hPa"
            )
            self.internal_status_label.setText(status_text)

    def clear_waypoints(self):
        """Clear the waypoint list."""
        self.waypoint_list.clear()
        self.send_button_ref.setEnabled(False)
        self.send_button_los.setEnabled(False)

    def update_button_states(self):
        """Enable/Disable buttons based on waypoint selection."""
        selected_count = len(self.waypoint_list.selectedItems())
        self.send_button_ref.setEnabled(selected_count == 1)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.showMaximized()
    sys.exit(app.exec())
