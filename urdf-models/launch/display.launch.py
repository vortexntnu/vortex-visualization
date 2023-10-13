from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


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

    return LaunchDescription([
        urdf_package_path_launch_arg,
        urdf_launch
    ])