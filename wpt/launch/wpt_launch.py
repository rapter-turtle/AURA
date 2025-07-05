from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wpt',
            executable='aura_wpt',
            name='aura_wpt',
            output='screen'
        ),
        Node(
            package='wpt',
            executable='ekf',
            name='ekf',
            output='screen'
        )
    ])
