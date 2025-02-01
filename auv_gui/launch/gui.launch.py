from os import path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = path.join(
        get_package_share_directory('auv_gui'),
        'config',
        'gui_params.yaml'
    )

    gui_node = Node(
        package='auv_gui',
        executable='auv_gui_node.py',
        name='gui_node',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([gui_node])