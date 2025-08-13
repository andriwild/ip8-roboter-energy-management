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

    eol_capactiy_factor = DeclareLaunchArgument(
        'eol_capactiy_factor',
        default_value='0.8',
        description='End of life factor (eol_capacity = capacity * eol_factor)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    bms_node = Node(
        package='bms',
        executable='bms',
        name='bms',
        output='screen',
        parameters=[
            {
                'dt': LaunchConfiguration('dt'),
                'initial_soc': LaunchConfiguration('initial_soc'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'eol_capactiy_factor': LaunchConfiguration('eol_capactiy_factor')
            }
        ],
        remappings=[]
    )
    
    return LaunchDescription([
        dt_arg,
        initial_soc_arg,
        use_sim_time_arg,
        eol_capactiy_factor,
        bms_node
    ])