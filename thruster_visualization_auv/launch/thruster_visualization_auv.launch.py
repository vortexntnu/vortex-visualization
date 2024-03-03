import math
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    trushter_visualization_node = Node(
            package='thruster_visualization_auv',
            executable='thruster_visualization_auv_node',
            name='thruster_visualization_auv_node',
            output='screen',
            parameters=[{
                'thruster0_position':    [0.485, 0.15, 0.0],
                'thruster0_orientation': [0.0, 0.0, (5/4)*math.pi],
                
                'thruster1_position':    [0.375, 0.15, 0.0],
                'thruster1_orientation': [0.0, (1/2)*math.pi, 0.0],

                'thruster2_position':    [-0.31, 0.15, 0.0],
                'thruster2_orientation': [0.0, (1/2)*math.pi, 0.0],

                'thruster3_position':    [0.44, 0.15, 0.0],
                'thruster3_orientation': [0.0, 0.0, (7/4)*math.pi],
                
                'thruster4_position':    [0.44, -0.15, 0.0],
                'thruster4_orientation': [0.0, 0.0, (5/4)*math.pi],

                'thruster5_position':    [-0.31, -0.15, 0.0],
                'thruster5_orientation': [0.0, (1/2)*math.pi, 0.0],

                'thruster6_position':    [0.375, -0.15, 0.0],
                'thruster6_orientation': [0.0, (1/2)*math.pi, 0.0],

                'thruster7_position':    [0.485, -0.15, 0.0],
                'thruster7_orientation': [0.0, 0.0, (7/4)*math.pi],
            }]
        )
    # thruster_configs = [
    #         {
    #             'name': 'thruster0',
    #             'x': 0.485,
    #             'y': 0.15,
    #             'z': 0.0,
    #             'roll': 0.0,
    #             'pitch': 0.0,
    #             'yaw': (5/4)*math.pi,
    #             'frame_id': 'base_link',
    #             'child_frame_id': 'thruster0',
    #         },
    #         {
    #             'name': 'thruster1',
    #             'x': 0.375,
    #             'y': 0.15,
    #             'z': 0.0,
    #             'roll': 0.0,
    #             'pitch': (1/2)*math.pi,
    #             'yaw': 0.0,
    #             'frame_id': 'base_link',
    #             'child_frame_id': 'thruster1',
    #         },
    #                     {
    #             'name': 'thruster2',
    #             'x': -0.31,
    #             'y': 0.15,
    #             'z': 0.0,
    #             'roll': 0.0,
    #             'pitch': (1/2)*math.pi,
    #             'yaw': 0.0,
    #             'frame_id': 'base_link',
    #             'child_frame_id': 'thruster2',
    #         },
    #                     {
    #             'name': 'thruster3',
    #             'x': 0.44,
    #             'y': 0.15,
    #             'z': 0.0,
    #             'roll': 0.0,
    #             'pitch': 0.0,
    #             'yaw': (7/4)*math.pi,
    #             'frame_id': 'base_link',
    #             'child_frame_id': 'thruster3',
    #         },
    #                     {
    #             'name': 'thruster4',
    #             'x': 0.44,
    #             'y': -0.15,
    #             'z': 0.0,
    #             'roll': 0.0,
    #             'pitch': 0.0,
    #             'yaw': (5/4)*math.pi,
    #             'frame_id': 'base_link',
    #             'child_frame_id': 'thruster4',
    #         },
    #                     {
    #             'name': 'thruster5',
    #             'x': -0.31,
    #             'y': -0.15,
    #             'z': 0.0,
    #             'roll': 0.0,
    #             'pitch': (1/2)*math.pi,
    #             'yaw': 0.0,
    #             'frame_id': 'base_link',
    #             'child_frame_id': 'thruster5',
    #         },
    #                     {
    #             'name': 'thruster6',
    #             'x': 0.375,
    #             'y': -0.15,
    #             'z': 0.0,
    #             'roll': 0.0,
    #             'pitch': (1/2)*math.pi,
    #             'yaw': 0.0,
    #             'frame_id': 'base_link',
    #             'child_frame_id': 'thruster6',
    #         },
    #                     {
    #             'name': 'thruster7',
    #             'x': 0.485,
    #             'y': -0.15,
    #             'z': 0.0,
    #             'roll': 0.0,
    #             'pitch': 0.0,
    #             'yaw': (7/4)*math.pi,
    #             'frame_id': 'base_link',
    #             'child_frame_id': 'thruster7',
    #         },
            
    #     ]

    #     # Initialize a list to hold the nodes
    # nodes = []

    # # Iterate over each thruster configuration and create a node
    # for config in thruster_configs:
    #     node = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name=config['name'],
    #         arguments=[
    #             '--x', str(config['x']),
    #             '--y', str(config['y']),
    #             '--z', str(config['z']),
    #             '--roll', str(config['roll']),
    #             '--pitch', str(config['pitch']),
    #             '--yaw', str(config['yaw']),
    #             '--frame-id', config['frame_id'],
    #             '--child-frame-id', config['child_frame_id'],
    #         ]
    #     )
    #     nodes.append(node)

    return LaunchDescription([
        trushter_visualization_node
    ])