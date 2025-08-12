#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    bms_timeout_arg = DeclareLaunchArgument(
        'bms_timeout',
        default_value='5.0',
        description='Timeout for BMS messages in seconds'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publication rate in Hz'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='battery_frame',
        description='Frame ID for battery messages'
    )
    
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
                'bms_timeout': LaunchConfiguration('bms_timeout'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[]
    )
    
    return LaunchDescription([
        bms_timeout_arg,
        publish_rate_arg,
        frame_id_arg,
        use_sim_time_arg,
        battery_monitor_node
    ])