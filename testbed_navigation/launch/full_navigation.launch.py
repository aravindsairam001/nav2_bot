from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params_file = os.path.join(
        get_package_share_directory('testbed_navigation'),
        'params',
        'nav2_params.yaml'
    )

    # Map YAML file
    map_yaml_file = os.path.join(
        get_package_share_directory('testbed_navigation'),
        'maps',
        'map.yaml'
    )

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file, 'use_sim_time': use_sim_time}]
    )

    # Controller
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Planner
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Behavior Tree Navigator
    bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Waypoint follower (optional)
    wp_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Behavior server
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        parameters=[nav2_params_file]
    )

    # Lifecycle manager
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'controller_server', 'planner_server', 'bt_navigator', 'waypoint_follower', 'behavior_server']
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        map_server_node,
        planner_node,
        controller_node,
        bt_node,
        wp_node,
        behavior_node,
        lifecycle_node
    ])
