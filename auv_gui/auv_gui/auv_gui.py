import math
import sys
from threading import Thread
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from nav_msgs.msg import Odometry
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QDoubleValidator
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
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32
from vortex_msgs.action import ReferenceFilterWaypoint

# --- Quaternion to Euler angles ---


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> list[float]:
    """Convert a quaternion to Euler angles (roll, pitch, yaw).

    Args:
        x (float): The x component of the quaternion.
        y (float): The y component of the quaternion.
        z (float): The z component of the quaternion.
        w (float): The w component of the quaternion.

    Returns:
        List[float]: A list of Euler angles [roll, pitch, yaw].

    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.degrees(math.atan2(sinr_cosp, cosr_cosp))

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.degrees(math.asin(sinp))

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.degrees(math.atan2(siny_cosp, cosy_cosp))

    return [roll, pitch, yaw]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    """Convert Euler angles to a quaternion.

    Args:
        roll (float): The roll angle in degrees.
        pitch (float): The pitch angle in degrees.
        yaw (float): The yaw angle in degrees.

    Returns:
        List[float]: A list of quaternion components [x, y, z, w].

    """
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cy * cp * cr + sy * sp * sr
    x = cy * cp * sr - sy * sp * cr
    y = sy * cp * sr + cy * sp * cr
    z = sy * cp * cr - cy * sp * sr

    return [x, y, z, w]


# --- GUI Node ---


class GuiNode(Node):
    """ROS2 Node that subscribes to odometry data and stores x, y positions."""

    def __init__(self) -> None:
        """Initialize the GuiNode and set up the odometry subscriber."""
        super().__init__("auv_gui_node")

        self._action_client = ActionClient(
            self, ReferenceFilterWaypoint, "reference_filter"
        )

        # ROS2 parameters
        self.declare_parameter("odom_topic", "/nucleus/odom")
        self.declare_parameter("current_topic", "/auv/power_sense_module/current")
        self.declare_parameter("voltage_topic", "/auv/power_sense_module/voltage")
        self.declare_parameter("temperature_topic", "/auv/temperature")
        self.declare_parameter("pressure_topic", "/auv/pressure")
        self.declare_parameter("history_length", 30)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        current_topic = (
            self.get_parameter("current_topic").get_parameter_value().string_value
        )
        voltage_topic = (
            self.get_parameter("voltage_topic").get_parameter_value().string_value
        )
        temperature_topic = (
            self.get_parameter("temperature_topic").get_parameter_value().string_value
        )
        pressure_topic = (
            self.get_parameter("pressure_topic").get_parameter_value().string_value
        )

        # Subscriber to the /nucleus/odom topic
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        # Variables to store odometry data
        self.xpos_data: list[float] = []  # x position
        self.ypos_data: list[float] = []  # y position
        self.zpos_data: list[float] = []  # z position

        self.w_data: list[float] = []  # w component of the quaternion
        self.x_data: list[float] = []  # x component of the quaternion
        self.y_data: list[float] = []  # y component of the quaternion
        self.z_data: list[float] = []  # z component of the quaternion

        self.roll: Optional[float] = None
        self.pitch: Optional[float] = None
        self.yaw: Optional[float] = None

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
        self.current = 0.0
        self.voltage = 0.0
        self.temperature = 0.0
        self.pressure = 0.0

        # --- Waypoint stuff ---
        # Create a publisher for the custom Waypoints message
        self.publisher_ = self.create_publisher(Pose, "waypoints", 10)

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

        self.send_button = QPushButton("Send Mission")
        self.send_button.clicked.connect(self.send_goal)
        self.send_button.setEnabled(False)  # Disabled by default

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

        # Enable the send button if there's at least one waypoint
        self.send_button.setEnabled(True)

    # def send_mission(self):
    #     """Publish an entire list of waypoints as a single Waypoints message."""
    #     waypoints_msg = Pose()
    #     waypoints_list = []

    #     for i in range(self.waypoint_list.count()):
    #         text = self.waypoint_list.item(i).text()
    #         # Simple parse: "X: xval, Y: yval, Z: zval"
    #         parts = text.replace(" ", "").split(",")
    #         x_val = float(parts[0].split(":")[1])
    #         y_val = float(parts[1].split(":")[1])
    #         z_val = float(parts[2].split(":")[1])

    #         pt = Point()
    #         pt.x = x_val
    #         pt.y = y_val
    #         pt.z = z_val
    #         waypoints_list.append(pt)

    #     waypoints_msg.points = waypoints_list

    #     self.publisher_.publish(waypoints_msg)
    #     self.get_logger().info("Mission waypoints have been published.")

    #     # Clear the list after sending
    #     self.waypoint_list.clear()
    #     self.send_button.setEnabled(False)  # Disable until a new waypoint is added

    def clear_waypoints(self):
        """Clear the waypoint list."""
        self.waypoint_list.clear()
        self.send_button.setEnabled(False)

    def send_goal(self):
        # Create the goal message
        goal_msg = ReferenceFilterWaypoint.Goal()

        # Create a PoseStamped message for the goal
        pose_stamped = PoseStamped()

        text = self.waypoint_list.selectedItems()[0].text()
        parts = text.replace(" ", "").split(",")
        x_val = float(parts[0].split(":")[1])
        y_val = float(parts[1].split(":")[1])
        z_val = float(parts[2].split(":")[1])
        roll_val = float(parts[3].split(":")[1])
        pitch_val = float(parts[4].split(":")[1])
        yaw_val = float(parts[5].split(":")[1])

        # Set the PoseStamped position
        pose_stamped.pose.position.x = x_val
        pose_stamped.pose.position.y = y_val
        pose_stamped.pose.position.z = z_val

        quat = euler_to_quaternion(roll_val, pitch_val, yaw_val)

        # Set orientation (identity quaternion if not specified)
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]

        goal_msg.goal = pose_stamped

        # Send the goal asynchronously
        self._action_client.wait_for_server()
        self.get_logger().info("Sending goal...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_goal(self) -> None:
        """Cancel the currently active goal."""
        self.get_logger().info("Canceling goal...")

        # Check if a goal has been sent
        if hasattr(self, "_send_goal_future") and self._send_goal_future:
            goal_handle = self._send_goal_future.result()

            # Cancel the goal
            if goal_handle:
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_result_callback)
            else:
                self.get_logger().warn("No active goal to cancel.")

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

    # --- Callback functions ---
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.feedback
        # self.get_logger().info(f'Received feedback: x={feedback.x}, y={feedback.y}, z={feedback.z}')

    def get_result_callback(self, future):
        result = future.result().result.result
        self.get_logger().info(f"Goal result: x={result.x}, y={result.y}, z={result.z}")

    def odom_callback(self, msg: Odometry) -> None:
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
        self.roll, self.pitch, self.yaw = quaternion_to_euler(x, y, z, w)

        # Limit the stored data for real-time plotting (avoid memory overflow)
        max_data_points = (
            self.get_parameter("history_length").get_parameter_value().integer_value
        )
        if len(self.x_data) > max_data_points:
            self.xpos_data.pop(0)
            self.ypos_data.pop(0)
            self.zpos_data.pop(0)

    def current_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a current message is received."""
        self.current = msg.data

    def voltage_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a voltage message is received."""
        self.voltage = msg.data

    def temperature_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a temperature message is received."""
        self.temperature = msg.data

    def pressure_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a pressure message is received."""
        self.pressure = msg.data


# --- Plotting ---


class PlotCanvas(FigureCanvas):
    """A canvas widget for plotting odometry data using matplotlib."""

    def __init__(self, gui_node: GuiNode, parent: Optional[QWidget] = None) -> None:
        """Initialize the PlotCanvas."""
        # Set up the 3D plot
        self.gui_node = gui_node
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")

        # Initialize a red dot for the current position
        (self.current_position_dot,) = self.ax.plot([], [], [], "ro")

        super().__init__(self.fig)
        self.setParent(parent)

        # Set labels and title for the 3D plot
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("Position")

        # Initialize data lists for 3D plot
        self.x_data: list[float] = []
        self.y_data: list[float] = []
        self.z_data: list[float] = []
        (self.line,) = self.ax.plot([], [], [], "b-")

    def update_plot(
        self, x_data: list[float], y_data: list[float], z_data: list[float]
    ) -> None:
        """Update the 3D plot with the latest odometry data."""
        # Convert lists to numpy arrays to ensure compatibility with the plot functions
        x_data = np.array(x_data, dtype=float)
        y_data = np.array(y_data, dtype=float)
        z_data = np.array(z_data, dtype=float)

        # Check if the arrays are non-empty before updating the plot
        if len(x_data) > 0 and len(y_data) > 0 and len(z_data) > 0:
            self.line.set_data(x_data, y_data)
            self.line.set_3d_properties(z_data)

            # Update the current position dot
            self.current_position_dot.set_data(x_data[-1:], y_data[-1:])
            self.current_position_dot.set_3d_properties(z_data[-1:])

            # Update the limits for the 3D plot around the latest data point
            x_latest = x_data[-1]
            y_latest = y_data[-1]
            z_latest = z_data[-1]
            margin = 2.5  # Define a margin around the latest point

            self.ax.set_xlim(x_latest - margin, x_latest + margin)
            self.ax.set_ylim(y_latest - margin, y_latest + margin)
            self.ax.set_zlim(z_latest - margin, z_latest + margin)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()


def run_ros_node(ros_node: GuiNode, executor: MultiThreadedExecutor) -> None:
    """Run the ROS2 node in a separate thread using a MultiThreadedExecutor."""
    rclpy.spin(ros_node, executor)


def main(args: Optional[list[str]] = None) -> None:
    """The main function to initialize ROS2 and the GUI application."""
    # Initialize QApplication before creating any widgets
    app = QApplication(sys.argv)

    # Initialize ROS2 after QApplication
    rclpy.init(args=args)
    ros_node = GuiNode()
    executor = MultiThreadedExecutor()

    # Run ROS in a separate thread
    ros_thread = Thread(target=run_ros_node, args=(ros_node, executor), daemon=True)
    ros_thread.start()

    # Setup the PyQt5 application and window
    gui = QMainWindow()
    gui.setWindowTitle("Vortex GUI")
    gui.setGeometry(100, 100, 600, 400)

    # Create the tab widget
    tabs = QTabWidget()
    tabs.setTabPosition(QTabWidget.TabPosition.North)
    tabs.setMovable(True)

    # --- Position Tab ---
    position_widget = QWidget()
    layout = QGridLayout(position_widget)  # grid layout

    plot_canvas = PlotCanvas(ros_node, position_widget)
    layout.addWidget(plot_canvas, 0, 0)

    current_pos = QLabel(parent=position_widget)
    layout.addWidget(current_pos, 0, 1)

    tabs.addTab(position_widget, "Position")

    # --- Internal Status Tab ---
    internal_widget = QWidget()
    internal_layout = QVBoxLayout(internal_widget)

    internal_status_label = QLabel(parent=internal_widget)
    internal_layout.addWidget(internal_status_label)
    tabs.addTab(internal_widget, "Internal")

    gui.setCentralWidget(tabs)
    gui.showMaximized()

    # --- Mission Interface Tab ---
    mission_widget = QWidget()
    mission_layout = QVBoxLayout(mission_widget)

    # Layouts for mission interface
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

    ros_node.send_button = QPushButton("Send Mission")
    ros_node.send_button.clicked.connect(ros_node.send_goal)
    ros_node.send_button.setEnabled(False)  # Disabled by default

    ros_node.clear_button = QPushButton("Clear Waypoints")
    ros_node.clear_button.clicked.connect(ros_node.clear_waypoints)

    ros_node.cancel_button = QPushButton("Cancel Mission")
    ros_node.cancel_button.clicked.connect(ros_node.cancel_goal)

    buttons_layout.addWidget(ros_node.add_button)
    buttons_layout.addWidget(ros_node.send_button)
    buttons_layout.addWidget(ros_node.clear_button)
    buttons_layout.addWidget(ros_node.cancel_button)

    # List widget to display waypoints
    ros_node.waypoint_list = QListWidget()

    # Add layouts to the main layout
    mission_layout.addLayout(inputs_layout)
    mission_layout.addLayout(buttons_layout)
    mission_layout.addWidget(ros_node.waypoint_list)

    tabs.addTab(mission_widget, "Mission")

    # Use a QTimer to update plot, current position, and internal status in the main thread
    def update_gui() -> None:
        plot_canvas.update_plot(
            ros_node.xpos_data, ros_node.ypos_data, ros_node.zpos_data
        )
        if len(ros_node.xpos_data) > 0 and ros_node.roll is not None:
            position_text = f"Current Position:\nX: {ros_node.xpos_data[-1]:.2f}\nY: {ros_node.ypos_data[-1]:.2f}\nZ: {ros_node.zpos_data[-1]:.2f}"
            orientation_text = f"Current Orientation:\nRoll: {ros_node.roll:.2f}\nPitch: {ros_node.pitch:.2f}\nYaw: {ros_node.yaw:.2f}"
            current_pos.setText(position_text + "\n\n\n" + orientation_text)

        # Update internal status
        internal_status_label.setText(
            f"Internal Status:\n"
            f"Current: {ros_node.current:.2f}\n"
            f"Voltage: {ros_node.voltage:.2f}\n"
            f"Temperature: {ros_node.temperature:.2f}\n"
            f"Pressure: {ros_node.pressure:.2f}"
        )

    # Set up the timer to call update_gui every 100ms
    timer = QTimer()
    timer.timeout.connect(update_gui)
    timer.start(100)

    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit()


if __name__ == "__main__":
    main()
