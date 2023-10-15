import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    package_name = 'urdf-models'
    urdf_default_path = PathJoinSubstitution(['urdf', 'freya', 'ASV_MainAssembly.urdf'])

    # Declare the launch arguments (for this launch file)
    urdf_package_path_launch_arg = DeclareLaunchArgument(
        name='model',
        default_value=urdf_default_path, 
        description='Path to robot urdf file relative to package'
    )

    # Include the launch description for the urdf launch file
    urdf_launch_path = PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py'])
    urdf_launch = IncludeLaunchDescription(
        launch_description_source=urdf_launch_path,
        launch_arguments={
            'urdf_package': package_name,
            'urdf_package_path': LaunchConfiguration('model')
        }.items()
    )

    # base_link (NED) to base_link (SEU) tf. TODO: Move to asv_setup tf.launch.py
    tf_base_link_ned_to_base_link_enu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x'             , '0',
                   '--y'             , '0',
                   '--z'             , '0',
                   '--roll'          , '0',
                   '--pitch'         , str(math.pi),
                   '--yaw'           , '0',
                   '--frame-id'      , 'base_link',
                   '--child-frame-id', 'base_link_SEU']
    )

    return LaunchDescription([
        urdf_package_path_launch_arg,
        urdf_launch,
        # tf_base_link_ned_to_base_link_enu
    ])