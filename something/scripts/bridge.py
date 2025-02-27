#!/usr/bin/env python3

from gevent import monkey
monkey.patch_all()

from flask import Flask, render_template
from flask_socketio import SocketIO
import rclpy
from rclpy.node import Node, Subscription
from rosidl_runtime_py.utilities import get_message
import threading
import yaml
import atexit
import time
from custom_msgs.msg import NorthEastHeading, ActuatorSetpoints, MotorState
from std_msgs.msg import Float64
from ma_utils.utils import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Wrench
import numpy as np

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

ros_lock = threading.Lock()

class BridgeNode(Node):
    def __init__(self, socketio):
        super().__init__('bridge_node')
        self.socketio: SocketIO = socketio

        self.init_dicts()
        self.init_topics()
        self.init_subscribers()
        self.init_publishers()

        self.get_logger().info('Bridge node has been initialized')

    def init_dicts(self):
        self.actuator_state = {"actuator_1": {"propeller": 0, "angle": 0, "force": 0},
                               "actuator_2": {"propeller": 0, "angle": 0, "force": 0},
                               "actuator_3": {"propeller": 0, "angle": 0, "force": 0},
                               "actuator_4": {"propeller": 0, "angle": 0, "force": 0}}
        self.gear_ratio = 1

    def init_topics(self):
        self.declare_parameter('topics.navigation_pose', '_')
        self.declare_parameter('topics.actuator_1_motor_state', '_')
        self.declare_parameter('topics.actuator_2_motor_state', '_')
        self.declare_parameter('topics.actuator_3_motor_state', '_')
        self.declare_parameter('topics.actuator_4_motor_state', '_')
        self.declare_parameter('topics.actuator_1_angle', '_')
        self.declare_parameter('topics.actuator_2_angle', '_')
        self.declare_parameter('topics.actuator_3_angle', '_')
        self.declare_parameter('topics.actuator_4_angle', '_')
        self.declare_parameter('topics.waypoint', '_')
        self.declare_parameter('topics.desired_pose', '_')
        self.declare_parameter('topics.control_action_reference', '_')
        
    def init_subscribers(self):
        eta_topic = self.get_parameter('topics.navigation_pose').value
        actuator_1_topic = self.get_parameter('topics.actuator_1_motor_state').value
        actuator_2_topic = self.get_parameter('topics.actuator_2_motor_state').value
        actuator_3_topic = self.get_parameter('topics.actuator_3_motor_state').value                
        actuator_4_topic = self.get_parameter('topics.actuator_4_motor_state').value
        angle_1_topic = self.get_parameter('topics.actuator_1_angle').value
        angle_2_topic = self.get_parameter('topics.actuator_2_angle').value
        angle_3_topic = self.get_parameter('topics.actuator_3_angle').value
        angle_4_topic = self.get_parameter('topics.actuator_4_angle').value
        desired_pose_topic = self.get_parameter('topics.desired_pose').value
        control_action_topic = self.get_parameter('topics.control_action_reference').value

        self.eta_subscriber = self.create_subscription(
            PoseStamped, eta_topic, self.eta_callback, 10)
        
        self.actuator_1_subscriber = self.create_subscription(
            MotorState, actuator_1_topic, self.actuator_1_callback, 10)
        
        self.actuator_2_subscriber = self.create_subscription(
            MotorState, actuator_2_topic, self.actuator_2_callback, 10)
        
        self.actuator_3_subscriber = self.create_subscription(
            MotorState, actuator_3_topic, self.actuator_3_callback, 10)
        
        self.actuator_4_subscriber = self.create_subscription(
            MotorState, actuator_4_topic, self.actuator_4_callback, 10)
        
        self.angle_1_subscriber = self.create_subscription(
            Float64, angle_1_topic, self.angle_1_callback, 10)
        
        self.angle_2_subscriber = self.create_subscription(
            Float64, angle_2_topic, self.angle_2_callback, 10)
        
        self.angle_3_subscriber = self.create_subscription(
            Float64, angle_3_topic, self.angle_3_callback, 10)
        
        self.angle_4_subscriber = self.create_subscription(
            Float64, angle_4_topic, self.angle_4_callback, 10)
        
        self.desired_pose_subscriber = self.create_subscription(
            NorthEastHeading, desired_pose_topic, self.desired_pose_callback, 10)
        
        self.control_action_subscriber = self.create_subscription(
            Wrench, control_action_topic, self.control_action_callback, 10)
        
    def init_publishers(self):
        waypoint_topic = self.get_parameter('topics.waypoint').value
        self.waypoint_publisher = self.create_publisher(
            NorthEastHeading, waypoint_topic, 10)

    def eta_callback(self, msg: PoseStamped):
        heading = euler_from_quaternion(msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w)[2]
        
        out_msg = {"north": msg.pose.position.x, "east": msg.pose.position.y, "heading": heading}
        self.socketio.emit('eta', out_msg)

    def actuator_1_callback(self, msg: MotorState):
        propeller = msg.motor_speed / self.gear_ratio
        force = self.rpm_to_thrust(propeller)
        self.actuator_state["actuator_1"]["propeller"] = propeller
        self.actuator_state["actuator_1"]["force"] = force

        socketio.emit("actuator_state", self.actuator_state)

    def actuator_2_callback(self, msg: MotorState):
        propeller = msg.motor_speed / self.gear_ratio
        force = self.rpm_to_thrust(propeller)
        self.actuator_state["actuator_2"]["propeller"] = propeller
        self.actuator_state["actuator_2"]["force"] = force

        socketio.emit("actuator_state", self.actuator_state)

    def actuator_3_callback(self, msg: MotorState):
        propeller = msg.motor_speed / self.gear_ratio
        force = self.rpm_to_thrust(propeller)
        self.actuator_state["actuator_3"]["propeller"] = propeller
        self.actuator_state["actuator_3"]["force"] = force

        socketio.emit("actuator_state", self.actuator_state)

    def actuator_4_callback(self, msg: MotorState):
        propeller = msg.motor_speed / self.gear_ratio
        force = self.rpm_to_thrust(propeller)
        self.actuator_state["actuator_4"]["propeller"] = propeller
        self.actuator_state["actuator_4"]["force"] = force

        socketio.emit("actuator_state", self.actuator_state)

    def control_action_callback(self, msg: Wrench):
        out = {"X": msg.force.x, "Y": msg.force.y, "N": msg.torque.z}
        socketio.emit('control_action_reference', out)
        socketio.sleep(0.1)

    def angle_1_callback(self, msg: Float64):
        self.actuator_state["actuator_1"]["angle"] = np.radians(msg.data)

    def angle_2_callback(self, msg: Float64):
        self.actuator_state["actuator_2"]["angle"] = np.radians(msg.data)

    def angle_3_callback(self, msg: Float64):
        self.actuator_state["actuator_3"]["angle"] = np.radians(msg.data)

    def angle_4_callback(self, msg: Float64):
        self.actuator_state["actuator_4"]["angle"] = np.radians(msg.data)

    def desired_pose_callback(self, msg: NorthEastHeading):
        out = {"north": msg.north, "east": msg.east, "heading": msg.heading}
        socketio.emit('desired_pose', out)
        socketio.sleep(0.1)

    @staticmethod
    def rpm_to_thrust(rpm):
        #Fifth order curvefit for thrust 
        #p = p_propeller_rpm_to_thrust

        #rpm = np.clip(rpm, min_rpm, max_rpm)
        #thrust = p[0] + p[1] * rpm + p[2] * rpm**2 + p[3] * rpm**3 + p[4] * rpm**4 + p[5] * rpm**5  # new SAC1 motor
        #thrust = p[0] * rpm + p[1] * rpm**2 + p[2] * rpm**3 + p[3] * rpm**4 + p[4] * rpm**5  # old torquedo
        coeff_fit_pos = np.array([0.000467890526617240, 0.0343246796131229, 0])
        coeff_fit_neg = np.array([-0.000331393640323551, 0.0240539891434794, 0])
        rpmmax = 960

        if rpm < 0:
            T = np.polyval(coeff_fit_neg, rpm)
        else:
            T = np.polyval(coeff_fit_pos, rpm)

        if rpm > rpmmax:
            T = np.polyval(coeff_fit_pos, rpmmax)

        if rpm < -rpmmax:
            T = np.polyval(coeff_fit_neg, -rpmmax)

        return T


def ros_spin_thread():
    while rclpy.ok():
        with ros_lock:
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.01)

@atexit.register
def shutdown_ros():
    node.destroy_node()
    rclpy.shutdown()

@app.route("/")
@app.route("/bridge")
def bridge():
    return render_template('bridge.html')

@app.route('/')
@app.route("/docking")
def docking():
    return render_template('docking.html')

@app.route("/thruster")
def thruster():
    return render_template('thruster.html')

@app.route("/messages")
def messages():
    return render_template('messages.html')

@app.route("/topics")
def topics():
    return render_template('topics.html')

@socketio.on("topic_list")
def handle_topic_list(msg):
    with ros_lock:
        topic_list = node.get_topic_names_and_types()
    
    return topic_list

@socketio.on("set_waypoint")
def handle_waypoint(waypoint):
    msg = NorthEastHeading()
    msg.north = float(waypoint['north'])
    msg.east = float(waypoint['east'])
    msg.heading = float(waypoint['heading'])

    with ros_lock:
        node.waypoint_publisher.publish(msg)

dynamic_subscriber: Subscription = None
@socketio.on("subscribe")
def handle_subscribe(msg):
    global dynamic_subscriber
    with ros_lock:
        if dynamic_subscriber is not None:
            node.destroy_subscription(dynamic_subscriber)
            dynamic_subscriber = None

        msg_type = get_message(msg['type'])
        dynamic_subscriber = node.create_subscription(
            msg_type, msg['topic'], dynamic_topic_callback, 1)

def dynamic_topic_callback(msg):
    socketio.emit('dynamic_topic', yaml.safe_load(str(msg)))

    socketio.sleep(0.1)

if __name__ == '__main__':
    rclpy.init()
    node = BridgeNode(socketio)

    spin_thread = threading.Thread(target=ros_spin_thread)
    spin_thread.daemon = True
    spin_thread.start()

    socketio.run(app, host='localhost', port=5000)
