from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="udp_visualization_bridge",
                executable="udp_marker_bridge",
                name="udp_marker_bridge",
                output="screen",
                parameters=[
                    {"port": 9870},
                    {"bind_address": "0.0.0.0"},
                    {"marker_topic": "/udp_plot/marker"},
                    {"frame_id": "map"},
                ],
            ),
        ]
    )
