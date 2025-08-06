from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    map_file = PathJoinSubstitution([
        FindPackageShare("testbed_navigation"),
        "maps",
        "map.yaml"
    ])

    return LaunchDescription([
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_file}],
        ),
    ])