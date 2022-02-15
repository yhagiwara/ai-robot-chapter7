from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 起動したいノードを記述
        Node(
            package='bringme_sm',
            executable='bringme_sm',
        ),

        Node(
            package='pseudo_node',
            executable='manipulation_node',
        ),

        Node(
            package='pseudo_node',
            executable='navigation_node',
        ),

        Node(
            package='pseudo_node',
            executable='vision_node',
        ),

        Node(
            package='pseudo_node',
            executable='voice_node',
        )
    ]
)
