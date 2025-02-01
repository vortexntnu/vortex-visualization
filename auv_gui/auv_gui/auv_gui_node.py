#!/usr/bin/env python3

import sys
import signal
from threading import Thread
from typing import Optional
from auv_gui.widgets import OpenGLPlotWidget, InternalStatusWidget
from ament_index_python.packages import get_package_share_directory

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QDoubleValidator, QAction, QPalette, QIcon
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QGridLayout,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMenu,
    QPushButton,
    QTabWidget,
    QWidget,
)
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from vortex_msgs.action import NavigateWaypoints, ReferenceFilterWaypoint
from vortex_utils.python_utils import euler_to_quat, quat_to_euler

best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

import random
import time
from queue import Queue

MOCK_INTERNAL_DATA = False

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
            self, NavigateWaypoints, "navigate_waypoints"
        )

        # ROS2 parameters
        self.declare_parameter("pose_topic", "/dvl/pose")
        self.declare_parameter("twist_topic", "/dvl/twist")
        self.declare_parameter("current_topic", "/auv/power_sense_module/current")
        self.declare_parameter("voltage_topic", "/auv/power_sense_module/voltage")
        self.declare_parameter("temperature_topic", "/auv/temperature")
        self.declare_parameter("pressure_topic", "/auv/pressure")
        self.declare_parameter("history_length", 30)
        self.declare_parameter("mock_data", False)

        pose_topic = self.get_parameter("pose_topic").value
        twist_topic = self.get_parameter("twist_topic").value
        current_topic = self.get_parameter("current_topic").value
        voltage_topic = self.get_parameter("voltage_topic").value
        temperature_topic = self.get_parameter("temperature_topic").value
        pressure_topic = self.get_parameter("pressure_topic").value

        self.mock_data = self.get_parameter("mock_data").value

        # Subscriber to the odometry topics
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self.twist_callback, qos_profile=best_effort_qos
        )

        self.twist_subscription = self.create_subscription(
            TwistWithCovarianceStamped, twist_topic, self.twist_callback, qos_profile=best_effort_qos
        )

        self.waypoints = []

        # Variables to store odometry data
        self.xpos_data: list[float] = []  # x position
        self.ypos_data: list[float] = []  # y position
        self.zpos_data: list[float] = []  # z position

        self.w_data: list[float] = []  # w component of the quaternion
        self.x_data: list[float] = []  # x component of the quaternion
        self.y_data: list[float] = []  # y component of the quaternion
        self.z_data: list[float] = []  # z component of the quaternion

        self.roll: float = None
        self.pitch: float = None
        self.yaw: float = None

        # Subscribe to internal status topics
        self.current_subscriber = self.create_subscription(
            Float32, current_topic, self.current_callback, 5
        )
        self.voltage_subscriber = self.create_subscription(
            Float32, voltage_topic, self.voltage_callback, 5
        )
        self.temperature_subscriber = self.create_subscription(
            Float32, temperature_topic, self.temperature_callback, 5
        )
        self.pressure_subscriber = self.create_subscription(
            Float32, pressure_topic, self.pressure_callback, 5
        )

        # Variables for internal status
        self.current = Queue(maxsize=10)
        self.voltage = Queue(maxsize=10)
        self.temperature = Queue(maxsize=10)
        self.pressure = Queue(maxsize=10)

        # --- Waypoint stuff ---

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

        # Buttons
        self.add_button = QPushButton("Add Waypoint")
        self.add_button.clicked.connect(self.add_waypoint)

        self.send_button_ref = QPushButton("Send Mission (ReferenceFilter)")
        self.send_button_ref.clicked.connect(self.send_goal_reference_filter)
        self.send_button_ref.setEnabled(False)

        self.send_button_nav = QPushButton("Send Mission (NavigateWaypoints)")
        self.send_button_nav.clicked.connect(self.send_goal_navigate_waypoints)
        self.send_button_nav.setEnabled(False)

        self.clear_button = QPushButton("Clear Waypoints")
        self.clear_button.clicked.connect(self.clear_waypoints)

        self.cancel_button = QPushButton("Cancel Mission")
        self.cancel_button.clicked.connect(self.cancel_goal)

    def add_waypoint(self):
        """Add a waypoint to the list widget."""
        x = self.x_input.text().strip()
        y = self.y_input.text().strip()
        z = self.z_input.text().strip()
        roll = self.roll_input.text().strip() or 0
        pitch = self.pitch_input.text().strip() or 0
        yaw = self.yaw_input.text().strip() or 0

        if x and y and z:
            list_entry = f"X: {x}, Y: {y}, Z: {z}"
            list_entry += f", Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}"
            self.waypoint_list.addItem(QListWidgetItem(list_entry))
        else:
            self.get_logger().warn("Invalid waypoint.")

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
        self.ordered_list.clear()
        self.send_button_nav.setEnabled(False)
        self.send_button_ref.setEnabled(False)

    def update_ordered_waypoints(self):
        """Update internal order of waypoints based on the reordered list."""
        ordered_items = [
            self.ordered_list.item(i).text() for i in range(self.ordered_list.count())
        ]

    def get_waypoints(self):
        """Retrieve waypoints from the selected items in the list."""
        selected_items = self.waypoint_list.selectedItems()

        if not selected_items:
            self.get_logger().error("No waypoints selected.")
            return None

        waypoints = []
        for item in selected_items:
            text = item.text()
            parts = text.replace(" ", "").split(",")
            x_val = float(parts[0].split(":")[1])
            y_val = float(parts[1].split(":")[1])
            z_val = float(parts[2].split(":")[1])
            roll_val = float(parts[3].split(":")[1])
            pitch_val = float(parts[4].split(":")[1])
            yaw_val = float(parts[5].split(":")[1])

            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x_val
            pose_stamped.pose.position.y = y_val
            pose_stamped.pose.position.z = z_val

            quat = euler_to_quat(roll_val, pitch_val, yaw_val)
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            waypoints.append(pose_stamped)

        return waypoints

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

    def send_goal_navigate_waypoints(self):
        """Send reordered waypoints to the NavigateWaypoints action."""
        if self.ordered_list.count() < 2:
            self.get_logger().error("NavigateWaypoints requires multiple waypoints.")
            return

        # Add first waypoint for plotting
        current_pos = PoseStamped()
        current_pos.pose.position.x = self.xpos_data[-1]
        current_pos.pose.position.y = self.ypos_data[-1]
        current_pos.pose.position.z = -self.zpos_data[-1]
        self.waypoints = [current_pos]
        for i in range(self.ordered_list.count()):
            text = self.ordered_list.item(i).text()
            parts = text.replace(" ", "").split(",")
            x_val = float(parts[0].split(":")[1])
            y_val = float(parts[1].split(":")[1])
            z_val = float(parts[2].split(":")[1])
            roll_val = float(parts[3].split(":")[1])
            pitch_val = float(parts[4].split(":")[1])
            yaw_val = float(parts[5].split(":")[1])

            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x_val
            pose_stamped.pose.position.y = y_val
            pose_stamped.pose.position.z = z_val

            quat = euler_to_quat(roll_val, pitch_val, yaw_val)
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            self.waypoints.append(pose_stamped)

        goal_msg = NavigateWaypoints.Goal()
        goal_msg.waypoints = self.waypoints[1:]

        # Send the goal asynchronously
        if not self._navigate_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("NavigateWaypoints action server not available.")
            return
        self.get_logger().info("Sending waypoints to NavigateWaypoints...")
        self._send_goal_future = self._navigate_waypoints_client.send_goal_async(
            goal_msg, feedback_callback=None
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def update_button_states(self):
        """Enable/Disable buttons based on waypoint selection."""
        selected_count = len(self.waypoint_list.selectedItems())
        self.send_button_ref.setEnabled(selected_count == 1)

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

    def show_waypoint_context_menu(self, pos):
        """Display the context menu when a waypoint is right-clicked."""
        menu = QMenu(self.waypoint_list)
        transfer_action = QAction("Transfer to Ordered List", self.waypoint_list)
        transfer_action.triggered.connect(self.transfer_selected_waypoint)
        menu.addAction(transfer_action)
        menu.exec(self.waypoint_list.viewport().mapToGlobal(pos))

    def transfer_selected_waypoint(self):
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            self.get_logger().warn("No waypoint selected for transfer.")
            return

        for item in selected_items:
            self.ordered_list.addItem(item.text())
            self.waypoint_list.takeItem(self.waypoint_list.row(item))

        self.send_button_nav.setEnabled(self.ordered_list.count() > 1)

    # --- Callback functions ---
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

    def pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """Callback function that is triggered when an odometry message is received."""
        # Extract x, y, z positions from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = -(msg.pose.pose.position.z)
        self.xpos_data.append(x)
        self.ypos_data.append(y)
        self.zpos_data.append(z)

        # Extract the quaternion components from the odometry message
        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        self.roll, self.pitch, self.yaw = quat_to_euler(x, y, z, w)

        # Limit the stored data for real-time plotting (avoid memory overflow)
        max_data_points = (
            self.get_parameter("history_length").get_parameter_value().integer_value
        )
        if len(self.xpos_data) > max_data_points:
            self.xpos_data.pop(0)
            self.ypos_data.pop(0)
            self.zpos_data.pop(0)

    def twist_callback(self, msg: TwistWithCovarianceStamped) -> None:
        pass

    def current_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a current message is received."""
        temp_timestamp = time.time()
        self.current.put((msg.data, temp_timestamp))

    def voltage_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a voltage message is received."""
        temp_timestamp = time.time()
        self.voltage.put((msg.data, temp_timestamp))

    def temperature_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a temperature message is received."""
        temp_timestamp = time.time()
        self.temperature.put((msg.data, temp_timestamp))

    def pressure_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a pressure message is received."""
        temp_timestamp = time.time()
        self.pressure.put((msg.data, temp_timestamp))

def run_ros_node(ros_node: GuiNode, executor: MultiThreadedExecutor) -> None:
    """Run the ROS2 node in a separate thread using a MultiThreadedExecutor."""
    rclpy.spin(ros_node, executor)


def main(args: Optional[list[str]] = None) -> None:
    """The main function to initialize ROS2 and the GUI application."""
    # Initialize QApplication before creating any widgets
    app = QApplication(sys.argv)
    package_share_directory = get_package_share_directory("auv_gui")
    app.setWindowIcon(QIcon(package_share_directory + "/resources/vortex_logo.png"))
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
    tabs = QTabWidget()
    tabs.setTabPosition(QTabWidget.TabPosition.North)
    tabs.setMovable(True)

    # --- Mission and Position Tab ---
    mission_position_widget = QWidget()
    mission_position_layout = QGridLayout(mission_position_widget)

    # --- Position Section ---
    plot_canvas = OpenGLPlotWidget(ros_node, mission_position_widget)
    mission_position_layout.addWidget(plot_canvas, 0, 0, 1, 3)
    mission_position_layout.setRowStretch(0, 5)
    mission_position_layout.setColumnStretch(0, 5)

    # Create the labels for current position and internal status
    current_pos = QLabel("Current Position: Not Available")
    internal_status_label = QLabel("Internal Status: Not Available")

    # Group them in a horizontal layout
    status_layout = QVBoxLayout()
    status_layout.addWidget(current_pos)
    status_layout.addWidget(internal_status_label)
    mission_position_layout.addLayout(status_layout, 0, 4)

    # --- Mission Section ---
    inputs_layout = QHBoxLayout()
    buttons_layout = QHBoxLayout()

    # Inputs for X, Y, Z
    ros_node.x_input = QLineEdit()
    ros_node.x_input.setPlaceholderText("X")
    ros_node.x_input.setValidator(QDoubleValidator())

    ros_node.y_input = QLineEdit()
    ros_node.y_input.setPlaceholderText("Y")
    ros_node.y_input.setValidator(QDoubleValidator())

    ros_node.z_input = QLineEdit()
    ros_node.z_input.setPlaceholderText("Z")
    ros_node.z_input.setValidator(QDoubleValidator())

    # Inputs for roll, pitch, yaw
    ros_node.roll_input = QLineEdit()
    ros_node.roll_input.setPlaceholderText("Roll")
    ros_node.roll_input.setValidator(QDoubleValidator())

    ros_node.pitch_input = QLineEdit()
    ros_node.pitch_input.setPlaceholderText("Pitch")
    ros_node.pitch_input.setValidator(QDoubleValidator())

    ros_node.yaw_input = QLineEdit()
    ros_node.yaw_input.setPlaceholderText("Yaw")
    ros_node.yaw_input.setValidator(QDoubleValidator())

    inputs_layout.addWidget(QLabel("Waypoint:"))
    inputs_layout.addWidget(ros_node.x_input)
    inputs_layout.addWidget(ros_node.y_input)
    inputs_layout.addWidget(ros_node.z_input)

    inputs_layout.addWidget(QLabel("Axes:"))
    inputs_layout.addWidget(ros_node.roll_input)
    inputs_layout.addWidget(ros_node.pitch_input)
    inputs_layout.addWidget(ros_node.yaw_input)

    # Buttons for mission interface
    ros_node.add_button = QPushButton("Add Waypoint")
    ros_node.add_button.clicked.connect(ros_node.add_waypoint)

    ros_node.send_button_ref = QPushButton("Send Mission (ReferenceFilter)")
    ros_node.send_button_ref.clicked.connect(ros_node.send_goal_reference_filter)
    ros_node.send_button_ref.setEnabled(False)

    ros_node.send_button_nav = QPushButton("Send Mission (NavigateWaypoints)")
    ros_node.send_button_nav.clicked.connect(ros_node.send_goal_navigate_waypoints)
    ros_node.send_button_nav.setEnabled(False)

    ros_node.clear_button = QPushButton("Clear Waypoints")
    ros_node.clear_button.clicked.connect(ros_node.clear_waypoints)

    ros_node.cancel_button = QPushButton("Cancel Mission")
    ros_node.cancel_button.clicked.connect(ros_node.cancel_goal)

    ros_node.clear_plot_button = QPushButton("Clear Plot")
    ros_node.clear_plot_button.clicked.connect(plot_canvas.clear_plot)

    buttons_layout.addWidget(ros_node.add_button)
    buttons_layout.addWidget(ros_node.send_button_ref)
    buttons_layout.addWidget(ros_node.clear_button)
    buttons_layout.addWidget(ros_node.cancel_button)
    buttons_layout.addWidget(ros_node.clear_plot_button)

    # List widget to display waypoints
    ros_node.waypoint_list = QListWidget()
    ros_node.waypoint_list.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
    ros_node.waypoint_list.customContextMenuRequested.connect(
        ros_node.show_waypoint_context_menu
    )
    ros_node.waypoint_list.setSelectionMode(
        QAbstractItemView.SelectionMode.ExtendedSelection
    )
    ros_node.waypoint_list.itemSelectionChanged.connect(ros_node.update_button_states)

    # Re-orderable list for NavigateWaypoints
    ros_node.ordered_list = QListWidget()
    ros_node.ordered_list.setSelectionMode(
        QAbstractItemView.SelectionMode.ExtendedSelection
    )
    ros_node.ordered_list.setDragDropMode(QAbstractItemView.DragDropMode.InternalMove)
    ros_node.ordered_list.model().rowsMoved.connect(ros_node.update_ordered_waypoints)

    # Add layouts to the mission position layout
    mission_position_layout.addLayout(inputs_layout, 1, 0, 1, 4)
    mission_position_layout.addLayout(buttons_layout, 2, 0, 1, 4)
    mission_position_layout.addWidget(ros_node.waypoint_list, 3, 0, 1, 4)
    mission_position_layout.addWidget(ros_node.ordered_list, 3, 4)
    mission_position_layout.addWidget(ros_node.send_button_nav, 2, 4)
    tabs.addTab(mission_position_widget, "Mission")

    # --- Internal Status Tab ---
    internal_status = InternalStatusWidget()
    tabs.addTab(internal_status.get_widget(), "Internal")

    # Start in fullscreen
    gui.setCentralWidget(tabs)
    gui.showMaximized()

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
                f"Current Position:\nX: {ros_node.xpos_data[-1]:.2f}\n"
                f"Y: {ros_node.ypos_data[-1]:.2f}\n"
                f"Z: {ros_node.zpos_data[-1]:.2f}"
            )
            orientation_text = (
                f"Current Orientation:\nRoll: {ros_node.roll:.2f}\n"
                f"Pitch: {ros_node.pitch:.2f}\n"
                f"Yaw: {ros_node.yaw:.2f}"
            )
            current_pos.setText(position_text + "\n\n" + orientation_text)

        try:
            current_val = ros_node.current.queue[-1][0]
            voltage_val = ros_node.voltage.queue[-1][0]
            temperature_val = ros_node.temperature.queue[-1][0]
            pressure_val = ros_node.pressure.queue[-1][0]
            status_text = (
                f"Internal Status:\n"
                f"Current: {current_val:.2f} A\n"
                f"Voltage: {voltage_val:.2f} V\n"
                f"Temperature: {temperature_val:.2f} °C\n"
                f"Pressure: {pressure_val:.2f} Pa"
            )
        except IndexError:
            status_text = "Internal Status: Not Available"
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
