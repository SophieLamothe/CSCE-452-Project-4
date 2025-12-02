from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tile_localization",
            executable="map_publisher",
            name="map_publisher",
            parameters=[{"world_file": "cave.world"}],  # <-- list of dicts
            output="screen",
        ),
        Node(
            package="tile_localization",
            executable="localizer",
            name="localizer",
            output="screen",
        ),
    ])