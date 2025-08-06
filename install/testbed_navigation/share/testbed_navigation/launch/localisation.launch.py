from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    amcl_params = PathJoinSubstitution([
        FindPackageShare("testbed_navigation"),
        "config",
        "amcl_params.yaml"
    ])

    return LaunchDescription([
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_params],
        ),
    ])