from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Map server node with correct path
    map_yaml_path = PathJoinSubstitution([
        FindPackageShare('testbed_bringup'),
        'maps',
        'testbed_world.yaml'
    ])
    
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'frame_id': 'map',
            'use_sim_time': True
        }]
    )
    
    # Static transform publishers for testing
    static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )
    
    static_transform_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': True}]
    )
    
    # Lifecycle manager node
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'use_sim_time': True,
            'node_names': ['map_server']
        }]
    )

    ld = LaunchDescription()
    
    # Add static transforms first
    ld.add_action(static_transform_map_odom)
    ld.add_action(static_transform_odom_base)
    
    # Add map server 
    ld.add_action(map_server_cmd)
    
    # Add lifecycle manager after a short delay
    ld.add_action(TimerAction(
        period=2.0,
        actions=[lifecycle_manager_cmd]
    ))
    
    return ld