from math import pi
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    trushter_visualization_node = Node(
            package='thruster_visualization',
            executable='thruster_visualization_node',
            name='thruster_visualization_node',
            output='screen',
            parameters=[{
                #NOTE: These coordinates are in the base_link frame
                'thruster0_position': [0.7, 0.5, 0.4],
                'thruster0_orientation': 7.0 * pi / 4.0,

                'thruster1_position': [-0.7, 0.5, 0.4],
                'thruster1_orientation': 1.0 * pi / 4.0,

                'thruster2_position': [-0.7, -0.5, 0.4],
                'thruster2_orientation': 7.0 * pi / 4.0,

                'thruster3_position': [0.7, -0.5, 0.4],
                'thruster3_orientation': 1.0 * pi / 4.0,
            }]
        )
    return LaunchDescription([
        trushter_visualization_node
    ])