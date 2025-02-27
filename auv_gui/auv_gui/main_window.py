#!/usr/bin/env python3

import sys
from ament_index_python.packages import get_package_share_directory
from playsound import playsound
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QDoubleValidator, QIcon, QPalette
from PyQt6.QtWidgets import (
    QApplication, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
    QListWidget, QListWidgetItem, QMainWindow, QPushButton,
    QTabWidget, QVBoxLayout, QWidget
)
from geometry_msgs.msg import PoseStamped
from vortex_utils.python_utils import euler_to_quat, H264Decoder
from threading import Thread
from PyQt6.QtGui import QImage, QPixmap
import numpy as np
from widgets import OpenGLPlotWidget, InternalStatusWidget
from data_manager import DataManager


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.data_manager = DataManager()
        self.setWindowTitle("Vortex GUI")
        self.setGeometry(100, 100, 600, 400)
        self.setWindowIcon(QIcon(get_package_share_directory("auv_gui") + "/resources/vortex_logo.png"))

        self.init_ui()

        self.decoder = H264Decoder()
        self.decoded_frames = None

        self.decoder_thread = Thread(target=self.decoder.start, daemon=True)
        self.decoder_thread.start()

    def init_ui(self):
        """Initialize the GUI layout and widgets."""
        tabs = QTabWidget()
        tabs.setTabPosition(QTabWidget.TabPosition.North)
        tabs.setMovable(True)

        # Mission Tab
        mission_widget = QWidget()
        mission_layout = QGridLayout(mission_widget)
        inputs_layout = QHBoxLayout()
        buttons_layout = QHBoxLayout()

        # --- Position Section ---
        plot_canvas = OpenGLPlotWidget(self.data_manager)
        mission_layout.addWidget(plot_canvas, 0, 0, 1, 3)
        mission_layout.setRowStretch(0, 5)
        mission_layout.setColumnStretch(0, 5)

        # Create the labels for current position and internal status
        current_pos = QLabel("<b>Current Position:</b> Not Available")
        current_pos.setStyleSheet("font-size: 30px;")
        internal_status_label = QLabel("<b>Internal Status:</b> Not Available")
        internal_status_label.setStyleSheet("font-size: 30px;")

        # Group them in a horizontal layout
        status_layout = QVBoxLayout()
        status_layout.addWidget(current_pos)
        status_layout.addWidget(internal_status_label)
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
        self.add_button.clicked.connect(self.add_waypoint)

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

    def display_frame(self, frame: np.ndarray) -> None:
        """Display a frame in the GUI."""
        height, width, channel = frame.shape
        bytes_per_line = 3 * width

        q_image = QImage(
            frame.data, width, height, bytes_per_line, QImage.Format.Format_BGR888
        )

        pixmap = QPixmap.fromImage(q_image)
        self.video_label.setPixmap(pixmap)

    def add_waypoint(self):
        """Add a waypoint to the list widget."""
        x, y, z = self.x_input.text().strip(), self.y_input.text().strip(), self.z_input.text().strip()
        roll, pitch, yaw = self.roll_input.text().strip() or 0, self.pitch_input.text().strip() or 0, self.yaw_input.text().strip() or 0

        if x and y and z:
            list_entry = f"X: {x}, Y: {y}, Z: {z}, Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}"
            self.waypoint_list.addItem(QListWidgetItem(list_entry))

            self.send_button_ref.setEnabled(True)
            self.send_button_los.setEnabled(True)

        # Clear inputs
        self.x_input.clear()
        self.y_input.clear()
        self.z_input.clear()
        self.roll_input.clear()
        self.pitch_input.clear()
        self.yaw_input.clear()

    def clear_waypoints(self):
        """Clear the waypoint list."""
        self.waypoint_list.clear()
        self.send_button_ref.setEnabled(False)
        self.send_button_los.setEnabled(False)

    def update_button_states(self):
        """Enable/Disable buttons based on waypoint selection."""
        selected_count = len(self.waypoint_list.selectedItems())
        self.send_button_ref.setEnabled(selected_count == 1)


def main():
    """Initialize the application and GUI."""
    app = QApplication(sys.argv)

    package_share_directory = get_package_share_directory("auv_gui")
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.ColorRole.WindowText, Qt.GlobalColor.darkRed)
    palette.setColor(QPalette.ColorRole.ButtonText, Qt.GlobalColor.darkRed)
    palette.setColor(QPalette.ColorRole.Text, Qt.GlobalColor.black)
    palette.setColor(QPalette.ColorRole.BrightText, Qt.GlobalColor.red)
    palette.setColor(QPalette.ColorRole.Link, Qt.GlobalColor.red)
    palette.setColor(QPalette.ColorRole.Base, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Window, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.Shadow, Qt.GlobalColor.darkGray)
    palette.setColor(QPalette.ColorRole.Button, Qt.GlobalColor.white)
    palette.setColor(QPalette.ColorRole.ToolTipBase, Qt.GlobalColor.white)
    app.setPalette(palette)

    main_window = MainWindow()
    main_window.showMaximized()
    playsound(package_share_directory + "/resources/GUI_startup_sound.mp3")

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
