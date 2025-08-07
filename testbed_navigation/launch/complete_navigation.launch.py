#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Get package directories
    nav_dir = get_package_share_directory('testbed_navigation')
    desc_dir = get_package_share_directory('testbed_description')
    
    # Robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_dir, 'launch', 'robot_description.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Map server
    map_yaml_path = PathJoinSubstitution([
        FindPackageShare('testbed_bringup'),
        'maps',
        'testbed_world.yaml'
    ])
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': use_sim_time,
            'frame_id': 'map'
        }]
    )
    
    # AMCL for localization
    amcl_params = os.path.join(nav_dir, 'config', 'amcl_params.yaml')
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
            ('map', '/map')
        ]
    )
    
    # Nav2 parameters
    nav2_params_file = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    
    # Controller server
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Planner server
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Behavior tree navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Waypoint follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}]
    )
    
    # Lifecycle manager for map server
    lifecycle_manager_map = Node(
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
    
    # Lifecycle manager for localization
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl']
        }]
    )
    
    # Lifecycle manager for navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )
    
    # Add delays for sequential startup
    delayed_map_server = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="Starting map server..."),
            map_server_node,
            lifecycle_manager_map
        ]
    )
    
    delayed_localization = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="Starting localization (AMCL)..."),
            amcl_node,
            lifecycle_manager_localization
        ]
    )
    
    delayed_navigation = TimerAction(
        period=6.0,
        actions=[
            LogInfo(msg="Starting navigation stack..."),
            controller_node,
            planner_node,
            bt_navigator_node,
            waypoint_follower_node,
            lifecycle_manager_navigation
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        LogInfo(msg="Starting complete navigation system..."),
        robot_description_launch,
        delayed_map_server,
        delayed_localization,
        delayed_navigation,
    ])
