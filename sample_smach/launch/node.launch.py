from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sample_smach',
            executable='manipulation',
        ),
        Node(
            package='sample_smach',
            executable='navigation',
        ),
        Node(
            package='sample_smach',
            executable='vision',
        ),
        Node(
            package='sample_smach',
            executable='voice',
        )
    ])
