from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = path.join(
        get_package_share_directory("auv_gui"), "config", "gui_params.yaml"
    )

    mock_data_arg = DeclareLaunchArgument(
        "mock_data",
        default_value="false",
        description="Use mock data for testing purposes",
        choices=["true", "false"],
    )

    gui_node = Node(
        package="auv_gui",
        executable="auv_gui_node.py",
        name="gui_node",
        output="screen",
        emulate_tty=True,
        parameters=[config, {"mock_data": LaunchConfiguration("mock_data")}],
    )

    return LaunchDescription([mock_data_arg, gui_node])
