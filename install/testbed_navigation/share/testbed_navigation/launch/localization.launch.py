from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Get the path to the AMCL parameters
    nav_dir = get_package_share_directory('testbed_navigation')
    amcl_params_path = os.path.join(nav_dir, 'config', 'amcl_params.yaml')
    
    # Create the AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_path, {'use_sim_time': use_sim_time}],
        remappings=[('scan','/scan'),
                    ('map','/map')]
    )
    
    # Create the lifecycle manager for AMCL
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['amcl']}
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation/Gazebo clock'),
        amcl_node,
        lifecycle_manager
    ])