from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wpt',
            executable='ekf_px4',
            name='ekf_px4',
            output='screen'
        ),
        Node(
            package='wpt',
            executable='thrust_check.py',
            name='thrust_check',
            output='screen'
        )
    ])
