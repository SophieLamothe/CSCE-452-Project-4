from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch argument for world file
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='cave.world',
    )
    
    # Argument for bag file name
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='localization_data',
    )
    
    return LaunchDescription([
        world_file_arg,
        bag_file_arg,
        
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
        
        # Record all topics
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a',
                 '-o', LaunchConfiguration('bag_file')],
            output='screen'
        ),
    ])