from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racer',
            namespace='racer',
            executable='control_error_extractor',
            name='control_error_extractor'
        ),
        Node(
            package='racer',
            namespace='racer',
            executable='servo_driver',
            name='servo_driver'
        )
    ])