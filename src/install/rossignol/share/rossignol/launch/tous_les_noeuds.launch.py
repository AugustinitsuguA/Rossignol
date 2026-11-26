from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rossignol',
            executable='points_img',
            output='screen',
        ),
        Node(
            package='rossignol',
            executable='detect',
            output='screen',
        ),
        Node(
            package='rossignol_cpp',
            executable='filtre',
            output='screen',
        ),
        Node(
            package='rossignol',
            executable='commande_test',
            output='screen',
        ),
        Node(
            package='rossignol',
            executable='record',
            output='screen',
        ),
        Node(
            package='rossignol',
            executable='calibration',
            output='screen',
        ),
    ])
