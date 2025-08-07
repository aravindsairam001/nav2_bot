from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    map_yaml_path = PathJoinSubstitution([
        FindPackageShare('testbed_bringup'),
        'maps',
        'testbed_world.yaml'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    # Modified map server configuration
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
            'frame_id': 'map',
            'topic_name': 'map',
            'publish_period_sec': 1.0,  # Add explicit publish period
        }]
    )
    
    # Modified static transform publisher
    static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    static_transform_map_base_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],  # Changed to base_footprint
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(map_server_cmd)
    # ld.add_action(static_transform_cmd)  # Disabled - let AMCL handle map->odom
    # ld.add_action(static_transform_map_base_cmd)  # Disabled - Gazebo handles odom->base_footprint
    ld.add_action(TimerAction(
        period=2.0,
        actions=[lifecycle_manager_cmd] 
    ))
    
    return ld