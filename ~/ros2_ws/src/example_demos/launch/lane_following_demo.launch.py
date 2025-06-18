#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_demos',
            executable='lane_following_demo',
            name='lane_following_demo',
            output='screen',
            parameters=[],
        ),
    ])