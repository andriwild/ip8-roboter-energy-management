#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Battery Monitor Node
    battery_monitor_node = Node(
        package='battery_state_publisher',
        executable='battery_state_publisher',
        name='battery_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        battery_monitor_node
    ])