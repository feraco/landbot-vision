#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_demos',
            executable='fall_detection_demo',
            name='fall_detection_demo',
            output='screen',
            parameters=[],
        ),
    ])