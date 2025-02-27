#!/usr/bin/env python3

import signal
import sys
from threading import Thread
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QDoubleValidator, QIcon, QImage, QPalette, QPixmap
from PyQt6.QtWidgets import (
    QAbstractItemView,
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
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from vortex_msgs.action import LOSGuidance, ReferenceFilterWaypoint
from vortex_utils.python_utils import H264Decoder, euler_to_quat, quat_to_euler

from auv_gui.widgets import InternalStatusWidget, OpenGLPlotWidget
from auv_gui.data_manager import DataManager

best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

import random
import time
from queue import Queue

# --- GUI Node ---


class GuiNode(Node):
    """ROS2 Node that subscribes to odometry data and stores x, y positions."""

    def __init__(self) -> None:
        """Initialize the GuiNode and set up the odometry subscriber."""
        super().__init__("auv_gui_node")

        self._reference_filter_client = ActionClient(
            self, ReferenceFilterWaypoint, "reference_filter"
        )
        self._navigate_waypoints_client = ActionClient(
            self, LOSGuidance, "los_guidance"
        )

        topic_params = [
            "pose",
            "twist",
            "current",
            "voltage",
            "temperature",
            "pressure",
        ]

        for param in topic_params:
            self.declare_parameter(f"topics.{param}", "_")
            setattr(
                self,
                param + "_topic",
                self.get_parameter(f"topics.{param}").value,
            )

        self.declare_parameter("mock_data", False)

        self.mock_data = self.get_parameter("mock_data").value

        # Subscriber to the odometry topics
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            qos_profile=best_effort_qos,
        )

        self.twist_subscription = self.create_subscription(
            TwistWithCovarianceStamped,
            self.twist_topic,
            self.twist_callback,
            qos_profile=best_effort_qos,
        )

        self.image_subscription = self.create_subscription(
            CompressedImage,
            "/image_compressed",
            self.image_callback,
            qos_profile=best_effort_qos,
        )

        self.decoder = H264Decoder()
        self.decoded_frames = None

        self.decoder_thread = Thread(target=self.decoder.start, daemon=True)
        self.decoder_thread.start()

        self.waypoints = []
        self.los_points = []

        self.data_manager = DataManager()

        # Subscribe to internal status topics
        self.current_subscriber = self.create_subscription(
            Float32, self.current_topic, self.current_callback, 5
        )
        self.voltage_subscriber = self.create_subscription(
            Float32, self.voltage_topic, self.voltage_callback, 5
        )
        self.temperature_subscriber = self.create_subscription(
            Float32, self.temperature_topic, self.temperature_callback, 5
        )
        self.pressure_subscriber = self.create_subscription(
            Float32, self.pressure_topic, self.pressure_callback, 5
        )


    def display_frame(self, frame: np.ndarray) -> None:
        """Display a frame in the GUI."""
        height, width, channel = frame.shape
        bytes_per_line = 3 * width

        q_image = QImage(
            frame.data, width, height, bytes_per_line, QImage.Format.Format_BGR888
        )

        pixmap = QPixmap.fromImage(q_image)
        self.video_label.setPixmap(pixmap)


    def send_goal_reference_filter(self):
        """Send a single waypoint to the ReferenceFilter action."""
        waypoints = self.get_waypoints()
        if not waypoints or len(waypoints) != 1:
            self.get_logger().error("This action requires exactly one waypoint.")
            return

        goal_msg = ReferenceFilterWaypoint.Goal()
        goal_msg.goal = waypoints[0]

        # Send the goal asynchronously
        if not self._reference_filter_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("ReferenceFilter action server not available.")
            return
        self.get_logger().info("Sending goal (ReferenceFilter)...")
        self._send_goal_future = self._reference_filter_client.send_goal_async(
            goal_msg, feedback_callback=None
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goal_los(self):
        """Send a single waypoint to the LOSGuidance action."""
        waypoints = self.get_waypoints()
        if not waypoints or len(waypoints) != 1:
            self.get_logger().error("This action requires exactly one waypoint.")
            return

        goal_msg = LOSGuidance.Goal()
        goal_msg.goal.point = waypoints[0].pose.position

        # Send the goal asynchronously
        if not self._navigate_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("LOSGuidance action server not available.")
            return
        self.get_logger().info("Sending goal (LOS)...")
        self._send_goal_future = self._navigate_waypoints_client.send_goal_async(
            goal_msg, feedback_callback=None
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        current_pos = [self.xpos_data[-1], self.ypos_data[-1], self.zpos_data[-1]]
        point = waypoints[0].pose.position
        goal = [point.x, point.y, point.z]
        self.los_points = [current_pos, goal]

    def cancel_goal(self) -> None:
        """Cancel the currently active goal."""
        self.get_logger().info("Canceling goal...")

        if hasattr(self, "_send_goal_future") and self._send_goal_future:
            goal_handle = self._send_goal_future.result()

            if goal_handle:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_result_callback)
            else:
                self.get_logger().warn("No active goal to cancel.")

    # --- Callback functions ---
    def image_callback(self, msg: CompressedImage) -> None:
        """Callback function that is triggered when a camera message is received."""
        self.decoder.push_data(msg.data)

        if self.decoder.decoded_frames:
            frame = self.decoder.decoded_frames[-1]
            self.display_frame(frame)

    def cancel_result_callback(self, future):
        """Callback when the goal cancel request has been completed."""
        try:
            result = future.result()
            if result:
                self.get_logger().info("Goal has been successfully canceled.")
            else:
                self.get_logger().warn("Goal cancel request failed.")
        except Exception as e:
            self.get_logger().error(f"Error during goal cancel: {e}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback

    def get_result_callback(self, future):
        result = future.result().result.success
        self.get_logger().info(f"Mission completed successfully: {result}")

    def pose_callback(self, msg):
        timestamp = time.time()
        data = {
            "timestamp": timestamp,
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "roll": msg.pose.pose.orientation.x,
            "pitch": msg.pose.pose.orientation.y,
            "yaw": msg.pose.pose.orientation.z,
        }
        self.data_manager.add_data("pose", data)

    def twist_callback(self, msg):
        timestamp = time.time()
        data = {"timestamp": timestamp, "vx": msg.twist.twist.linear.x, "vy": msg.twist.twist.linear.y, "vz": msg.twist.twist.linear.z}
        self.data_manager.add_data("twist", data)

    def current_callback(self, msg):
        self.data_manager.add_data("current", {"timestamp": time.time(), "value": msg.data})

    def voltage_callback(self, msg):
        self.data_manager.add_data("voltage", {"timestamp": time.time(), "value": msg.data})

    def temperature_callback(self, msg):
        self.data_manager.add_data("temperature", {"timestamp": time.time(), "value": msg.data})

    def pressure_callback(self, msg):
        self.data_manager.add_data("pressure", {"timestamp": time.time(), "value": msg.data})


def run_ros_node(ros_node: GuiNode, executor: MultiThreadedExecutor) -> None:
    """Run the ROS2 node in a separate thread using a MultiThreadedExecutor."""
    rclpy.spin(ros_node, executor)


def main(args: Optional[list[str]] = None) -> None:
    """The main function to initialize ROS2 and the GUI application."""
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

    rclpy.init(args=args)
    ros_node = GuiNode()
    executor = MultiThreadedExecutor()

    ros_thread = Thread(target=run_ros_node, args=(ros_node, executor), daemon=True)
    ros_thread.start()

    gui = QMainWindow()
    gui.setWindowTitle("Vortex GUI")
    gui.setGeometry(100, 100, 600, 400)
    gui.autoFillBackground()
    gui.setWindowIcon(QIcon(package_share_directory + "/resources/vortex_logo.png"))
    tabs = QTabWidget()
    tabs.setTabPosition(QTabWidget.TabPosition.North)
    tabs.setMovable(True)

    # --- Image Tab ---
    image_tab_widget = QWidget()
    image_layout = QVBoxLayout(image_tab_widget)
    image_layout.addWidget(ros_node.video_label)
    tabs.addTab(image_tab_widget, "Camera Feed")

    # Use a QTimer to update plot, current position, and internal status in the main thread
    def update_gui() -> None:
        # Update mock data first if enabled
        if ros_node.mock_data:
            ros_node.current.put((1.0 + (random.random() * 0.06), time.time()))
            ros_node.voltage.put((12.0 + (random.random() * 0.07), time.time()))
            ros_node.temperature.put((25.0 + (random.random() * 0.15), time.time()))
            ros_node.pressure.put((1013.25 + (random.random()), time.time()))

        plot_canvas.update_plot(
            ros_node.xpos_data, ros_node.ypos_data, ros_node.zpos_data
        )

        if len(ros_node.xpos_data) > 0 and ros_node.roll is not None:
            position_text = (
                f"<b>Current Position:</b><br>X: {ros_node.xpos_data[-1]:.2f}<br>"
                f"Y: {ros_node.ypos_data[-1]:.2f}<br>"
                f"Z: {ros_node.zpos_data[-1]:.2f}"
            )
            orientation_text = (
                f"<b>Current Orientation:</b><br>Roll: {ros_node.roll:.2f}<br>"
                f"Pitch: {ros_node.pitch:.2f}<br>"
                f"Yaw: {ros_node.yaw:.2f}"
            )
            current_pos.setText(position_text + "<br><br>" + orientation_text + "<br>")

        if len(ros_node.los_points) > 0:
            plot_canvas.plot_points_and_line(ros_node.los_points)
            ros_node.los_points = []

        try:
            current_val = ros_node.current.queue[-1][0]
            voltage_val = ros_node.voltage.queue[-1][0]
            temperature_val = ros_node.temperature.queue[-1][0]
            pressure_val = ros_node.pressure.queue[-1][0]
            status_text = (
                f"<b>Internal Status:</b><br>"
                f"Current: {current_val:.2f} A<br>"
                f"Voltage: {voltage_val:.2f} V<br>"
                f"Temperature: {temperature_val:.2f} °C<br>"
                f"Pressure: {pressure_val:.2f} hPa"
            )
        except IndexError:
            status_text = "<b>Internal Status:</b> Not Available"
        internal_status_label.setText(status_text)

        internal_status.update(
            ros_node.current, ros_node.voltage, ros_node.temperature, ros_node.pressure
        )

    timer = QTimer()
    timer.timeout.connect(update_gui)
    timer.start(100)

    def signal_handler(sig, frame):
        print("\n[INFO] Shutting down gracefully...")
        ros_node.destroy_node()
        rclpy.shutdown()
        app.quit()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        app.exec()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
