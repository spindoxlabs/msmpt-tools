
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


logger_parameters = {
    "debug": LaunchConfiguration("debug"),
    "output_folder": LaunchConfiguration("output_folder"),
    "tracks_topic": LaunchConfiguration("tracks_topic"),
}

def generate_launch_description():
    debug_arg = DeclareLaunchArgument("debug", default_value="true")
    output_folder_arg = DeclareLaunchArgument("output_folder", default_value="/data/")
    tracks_topic_arg = DeclareLaunchArgument("tracks_topic", default_value="/tracks")

    logger = Node(
            package='rexasi_logger',
            executable='logger',
            name='logger',
            parameters=[{**logger_parameters}],
    )

    return LaunchDescription(
        [
            debug_arg,
            output_folder_arg,
            tracks_topic_arg,
            logger
        ]
    )
