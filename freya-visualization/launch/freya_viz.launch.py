import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():     
    
    freya_model_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('urdf_models'), 'launch', 'freya_urdf.launch.py')
        ),
    )

    thruster_viz_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('thruster_visualization'), 'launch', 'thruster_visualization.launch.py')
        ),
    )
        
    return LaunchDescription([
        freya_model_launch,
        thruster_viz_launch,
    ])