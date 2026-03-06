from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='person_follower',
            executable='follower_node',
            name='follower_node',
            output='screen'
        )
    ])
