#!/usr/bin/env python3

import sys

from ament_index_python.packages import get_package_share_directory
from playsound import playsound
from PyQt6.QtGui import QIcon
from PyQt6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)
from widgets import OpenGLPlotWidget

package_share_directory = get_package_share_directory("auv_gui")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.init_ui()

    def init_ui(self):
        self.setWindowIcon(
            QIcon(package_share_directory + "/resources/vortex_logo.png")
        )
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        plot_window = OpenGLPlotWidget(None)
        display_data = QLabel("Display Data Here")
        display_data.setStyleSheet("background-color: red;")

        mission_label = QLabel("Mission Control")
        mission_label.setStyleSheet("background-color: yellow;")

        main_layout = QVBoxLayout()

        display_layout = QHBoxLayout()  # Plot window and data display
        display_layout.addWidget(plot_window)
        display_layout.addWidget(display_data)
        display_layout.setStretch(0, 8)
        display_layout.setStretch(1, 2)

        mission_layout = QHBoxLayout()  # Mission control buttons
        mission_layout.addWidget(mission_label)

        main_layout.addLayout(display_layout)
        main_layout.addLayout(mission_layout)
        main_layout.setStretch(0, 8)
        main_layout.setStretch(1, 2)

        central_widget.setLayout(main_layout)


def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.showMaximized()
    playsound(package_share_directory + "/resources/GUI_startup_sound.mp3")
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
