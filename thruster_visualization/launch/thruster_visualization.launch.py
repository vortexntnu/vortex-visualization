from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    trushter_visualization_node = Node(
            package='thruster_visualization',
            executable='thruster_visualization_node',
            name='thruster_visualization_node',
            output='screen',
        )
    return LaunchDescription([
        trushter_visualization_node
    ])