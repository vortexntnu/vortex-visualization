from math import pi
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    thruster_visualization_node = Node(
            package='thruster_visualization',
            executable='thruster_visualization_node',
            name='thruster_visualization_node',
            output='screen',
            parameters=[{
                'num_thrusters': 8,
                'thruster0_position':    [0.42, 0.16, 0.085],
                'thruster0_orientation': [0.0, 0.0, 7.0*pi/4.0],
                
                'thruster1_position':    [0.3, 0.16, 0.021],
                'thruster1_orientation': [0.0, 0.5*pi, 0.0],

                'thruster2_position':    [-0.305, 0.16, 0.021],
                'thruster2_orientation': [0.0, 0.5*pi, 0.0],

                'thruster3_position':    [-0.425, 0.16, 0.085],
                'thruster3_orientation': [0.0, 0.0, (1/4)*pi],
                
                'thruster4_position':    [-0.425, -0.16, 0.085],
                'thruster4_orientation': [0.0, 0.0, 7.0*pi/4.0],

                'thruster5_position':    [-0.305, -0.16, 0.021],
                'thruster5_orientation': [0.0, 0.5*pi, 0.0],

                'thruster6_position':    [0.3, -0.16, 0.021],
                'thruster6_orientation': [0.0, 0.5*pi, 0.0],

                'thruster7_position':    [0.42, -0.16, 0.085],
                'thruster7_orientation': [0.0, 0.0, (1/4)*pi],
            }]
        )
    return LaunchDescription([
        thruster_visualization_node
    ])