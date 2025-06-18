#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_demos',
            executable='body_track_demo',
            name='body_track_demo',
            output='screen',
            parameters=[],
        ),
    ])