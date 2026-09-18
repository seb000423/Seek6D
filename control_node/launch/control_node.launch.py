"""Launch the control node after MoveIt and ros2_control are ready."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="control_node",
            executable="robot_control_any6d_moveit",
            name="robot_control_any6d",
            namespace="dsr01",
            output="screen",
        ),
    ])
