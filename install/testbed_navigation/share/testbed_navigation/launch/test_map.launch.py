from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Map server node
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': '/home/harshit/assignment_ws/install/testbed_bringup/share/testbed_bringup/maps/testbed_world.yaml',
            'frame_id': 'map'
        }]
    )
    
    # Lifecycle manager node
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    ld = LaunchDescription()
    
    # Add map server first
    ld.add_action(map_server_cmd)
    
    # Add lifecycle manager after a short delay
    ld.add_action(TimerAction(
        period=2.0,
        actions=[lifecycle_manager_cmd]
    ))
    
    return ld