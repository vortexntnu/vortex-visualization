import math

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "urdf_models"
    urdf_path = PathJoinSubstitution(
        ["urdf", "njord-buoys", "south_cardinal_mark.urdf"]
    )

    # Include the launch description for the urdf launch file
    urdf_launch_path = PathJoinSubstitution(
        [FindPackageShare("urdf_launch"), "launch", "description.launch.py"]
    )
    urdf_launch = IncludeLaunchDescription(
        launch_description_source=urdf_launch_path,
        launch_arguments={
            "urdf_package": package_name,
            "urdf_package_path": urdf_path,
        }.items(),
    )

    # base_link (NED) to base
    tf_base_link_ned_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            str(math.pi),
            "--yaw",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "base",
        ],
    )

    return LaunchDescription([urdf_launch, tf_base_link_ned_to_base])
