from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    params = RewrittenYaml(
        source_file='path/to/nav2_params.yaml',
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