from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_params_file = os.path.join(
        get_package_share_directory('testbed_navigation'),
        'params',
        'nav2_params.yaml'
    )
    
    params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'controller_server.ros__parameters.base_frame_id': 'base_footprint',
            'planner_server.ros__parameters.base_frame_id': 'base_footprint'
        },
        convert_types=True
    )

    return LaunchDescription([
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params]
        )
    ])