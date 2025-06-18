#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    body_control_arg = DeclareLaunchArgument('body_control', default_value='false')
    color_detect_arg = DeclareLaunchArgument('color_detect', default_value='false')
    hand_gesture_arg = DeclareLaunchArgument('hand_gesture', default_value='false')
    hand_track_arg = DeclareLaunchArgument('hand_track', default_value='false')
    body_track_arg = DeclareLaunchArgument('body_track', default_value='false')
    color_sorting_arg = DeclareLaunchArgument('color_sorting', default_value='false')
    fall_detection_arg = DeclareLaunchArgument('fall_detection', default_value='false')
    self_driving_arg = DeclareLaunchArgument('self_driving', default_value='false')
    lane_following_arg = DeclareLaunchArgument('lane_following', default_value='false')
    traffic_sign_arg = DeclareLaunchArgument('traffic_sign', default_value='false')

    return LaunchDescription([
        body_control_arg,
        color_detect_arg,
        hand_gesture_arg,
        hand_track_arg,
        body_track_arg,
        color_sorting_arg,
        fall_detection_arg,
        self_driving_arg,
        lane_following_arg,
        traffic_sign_arg,

        # Body Control Demo
        Node(
            package='example_demos',
            executable='body_control_demo',
            name='body_control_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('body_control')),
        ),

        # Color Detection Demo
        Node(
            package='example_demos',
            executable='color_detect_demo',
            name='color_detect_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('color_detect')),
        ),

        # Hand Gesture Demo
        Node(
            package='example_demos',
            executable='hand_gesture_demo',
            name='hand_gesture_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('hand_gesture')),
        ),

        # Hand Tracking Demo
        Node(
            package='example_demos',
            executable='hand_track_demo',
            name='hand_track_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('hand_track')),
        ),

        # Body Tracking Demo
        Node(
            package='example_demos',
            executable='body_track_demo',
            name='body_track_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('body_track')),
        ),

        # Color Sorting Demo
        Node(
            package='example_demos',
            executable='color_sorting_demo',
            name='color_sorting_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('color_sorting')),
        ),

        # Fall Detection Demo
        Node(
            package='example_demos',
            executable='fall_detection_demo',
            name='fall_detection_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('fall_detection')),
        ),

        # Self-Driving Demo
        Node(
            package='example_demos',
            executable='self_driving_demo',
            name='self_driving_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('self_driving')),
        ),

        # Lane Following Demo
        Node(
            package='example_demos',
            executable='lane_following_demo',
            name='lane_following_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('lane_following')),
        ),

        # Traffic Sign Recognition Demo
        Node(
            package='example_demos',
            executable='traffic_sign_demo',
            name='traffic_sign_demo',
            output='screen',
            condition=IfCondition(LaunchConfiguration('traffic_sign')),
        ),
    ])