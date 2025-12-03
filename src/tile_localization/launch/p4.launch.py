from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch argument for world file
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='cave.world',
        description='Name of the world file to load (e.g., cave.world, light.world)'
    )
    
    return LaunchDescription([
        world_file_arg,
        
        Node(
            package="tile_localization",
            executable="map_publisher",
            name="map_publisher",
            parameters=[{'world_file': LaunchConfiguration('world_file')}],
            output="screen",
        ),
        
        Node(
            package="tile_localization",
            executable="localizer",
            name="localizer",
            parameters=[{'world_file': LaunchConfiguration('world_file')}],
            output="screen",
        ),
    ])