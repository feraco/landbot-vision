#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_demos',
            executable='ui_control_panel',
            name='ui_control_panel',
            output='screen',
            parameters=[],
        ),
    ])