from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='battery_monitor',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen',
            parameters=[],
            remappings=[]
        )
    ])