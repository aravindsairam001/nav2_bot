#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

  pkg_testbed_gazebo = get_package_share_directory('testbed_gazebo')
  pkg_testbed_description = get_package_share_directory('testbed_description')

  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_testbed_gazebo, 'launch', 'spawn_playground.launch.py'),
    )
  ) 
  
  state_pub = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_testbed_description, 'launch', 'robot_description.launch.py'),
    ),
    launch_arguments={'use_sim_time': 'true'}.items()
  )

  spawn = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_testbed_gazebo, 'launch', 'spawn_testbed.launch.py'),
    )
  )
  
  rviz_config_dir = os.path.join(
    pkg_testbed_description,
    'rviz',
    'full_bringup.rviz')
  
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz_node',
    parameters=[{
        'use_sim_time': True,
        'tf_buffer_size': 120,  # Increase TF buffer size
        'tf_timeout': 10.0      # Increase TF timeout
    }],
    arguments=['-d', LaunchConfiguration('rvizconfig')]
  )

  # Add sequential timing for better coordination
  delayed_spawn = TimerAction(
    period=3.0,  # Increased spawn delay to 3 seconds for better TF establishment
    actions=[
        LogInfo(msg="Spawning robot in Gazebo..."),
        spawn
    ]
  )

  # Add a delay for RViz to start after other components are ready
  delayed_rviz = TimerAction(
    period=7.0,  # Increased delay to 7 seconds to ensure sensor data and TF are synchronized
    actions=[
        LogInfo(msg="Starting RViz visualization..."),
        rviz_node
    ]
  )

  return LaunchDescription([
    DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_dir,
                                            description='Absolute path to rviz config file'),
    LogInfo(msg="Starting Testbed Full Bringup..."),
    LogInfo(msg="Starting Robot Description and Gazebo World..."),
    state_pub,
    gazebo,
    delayed_spawn,  # Use delayed spawn instead of immediate spawn
    delayed_rviz,
  ])