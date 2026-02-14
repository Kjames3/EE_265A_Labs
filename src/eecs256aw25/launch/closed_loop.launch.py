from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eecs256aw25',
            executable='closed_loop',
            name='closed_loop',
            output='screen',
            emulate_tty=True
        )
    ])
