#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_any6d", default_value="true"),
        DeclareLaunchArgument("start_all_objects", default_value="true"),
        Node(
            package="vision_nodes",
            executable="dino_any6d_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_any6d")),
        ),
        Node(
            package="vision_nodes",
            executable="dino_all_object_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_all_objects")),
        ),
    ])
