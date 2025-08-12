#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch Arguments
    dt_arg = DeclareLaunchArgument(
        'dt',
        default_value='1.0',
        description='Sample time in seconds for the SOC estimator'
    )
    
    initial_soc_arg = DeclareLaunchArgument(
        'initial_soc',
        default_value='1.0',
        description='Initial State of Charge (0.0 to 1.0)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # SOC Estimator Node
    soc_estimator_node = Node(
        package='battery_soc_estimator',
        executable='battery_soc_estimator',
        name='battery_soc_estimator',
        output='screen',
        parameters=[
            {
                'dt': LaunchConfiguration('dt'),
                'initial_soc': LaunchConfiguration('initial_soc'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[]
    )
    
    return LaunchDescription([
        dt_arg,
        initial_soc_arg,
        use_sim_time_arg,
        soc_estimator_node
    ])