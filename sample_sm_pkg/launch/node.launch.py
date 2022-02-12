from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pseudo_node_pkg',
            executable='manipulation_node',
        ),
        Node(
            package='pseudo_node_pkg',
            executable='navigation_node',
        ),
        Node(
            package='pseudo_node_pkg',
            executable='vision_node',
        ),
        Node(
            package='pseudo_node_pkg',
            executable='voice_node',
        )
    ])
