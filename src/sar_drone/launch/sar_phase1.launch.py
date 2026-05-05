from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sar_rover',
            executable='rover_node',
            name='rover_node',
            output='screen',
            prefix='xterm -e',
        ),
        Node(
            package='sar_drone',
            executable='drone_node',
            name='drone_node',
            output='screen',
            prefix='xterm -e',
        ),
    ])
