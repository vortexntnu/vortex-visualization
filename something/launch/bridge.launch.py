from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('bridge'), 'config', 'bridge.yaml')
    topics = os.path.join(get_package_share_directory('ma_config'), 'config', 'topics.yaml')

    return LaunchDescription([
        Node(
            package='bridge',
            executable='bridge.py',
            name='bridge_node',
            output='screen',
            parameters=[config, topics]
        ),
    ])