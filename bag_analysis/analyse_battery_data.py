#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from datetime import datetime
import seaborn as sns
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from rosidl_runtime_py.utilities import get_message

sns.set_style("whitegrid")

def extract_data(bag_path):
    print(f"Loading bag file: {bag_path}")
    
    battery_data = {'time': [], 'voltage': [], 'current': [], 'charge': [], 'capacity': [], 'percentage': []}
    pose_data = {'time': [], 'x': [], 'y': [], 'z': []}
    odom_data = {'time': [], 'x': [], 'y': [], 'z': [], 'vx': [], 'vy': [], 'vz': [], 'wx': [], 'wy': [], 'wz': []}
    
    storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    start_time = None
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if start_time is None:
            start_time = timestamp
        
        rel_time = (timestamp - start_time) / 1e9
        if rel_time < 7182:
            continue
        
        if topic == '/bms/state':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            battery_data['time'].append(rel_time)
            battery_data['voltage'].append(msg.voltage)
            battery_data['current'].append(msg.current)
            battery_data['charge'].append(msg.charge)
            battery_data['capacity'].append(msg.capacity)
            battery_data['percentage'].append(msg.percentage)
            
        elif topic == '/do150_0007/amcl_pose':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            pose_data['time'].append(rel_time)
            pose_data['x'].append(msg.pose.pose.position.x)
            pose_data['y'].append(msg.pose.pose.position.y)
            pose_data['z'].append(msg.pose.pose.position.z)
            
        elif topic == '/do150_0007/odom':
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            odom_data['time'].append(rel_time)
            odom_data['x'].append(msg.pose.pose.position.x)
            odom_data['y'].append(msg.pose.pose.position.y)
            odom_data['z'].append(msg.pose.pose.position.z)
            odom_data['vx'].append(msg.twist.twist.linear.x)
            odom_data['vy'].append(msg.twist.twist.linear.y)
            odom_data['vz'].append(msg.twist.twist.linear.z)
            odom_data['wx'].append(msg.twist.twist.angular.x)
            odom_data['wy'].append(msg.twist.twist.angular.y)
            odom_data['wz'].append(msg.twist.twist.angular.z)
    
    print(f"Data extraction complete")
    return battery_data, pose_data, odom_data


def create_combined_visualization(battery_data, pose_data, odom_data):
    num_plots = 4
    fig = plt.figure(figsize=(12, 20))
    gs = gridspec.GridSpec(num_plots, 1, figure=fig, hspace=0.3, height_ratios=[1, 1, 1, 1, 1, 1])
    
    ax1 = fig.add_subplot(gs[0, 0])
    if battery_data['time'] and battery_data['percentage']:
        ax1.plot(battery_data['time'], battery_data['percentage'], 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Battery Percentage (%)')
        #ax1.set_title('Battery State Over Time')
        ax1.grid(True, alpha=0.3)
    
    ax2 = fig.add_subplot(gs[1, 0])
    if battery_data['time'] and battery_data['voltage']:
        ax2.plot(battery_data['time'], battery_data['voltage'], 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Voltage (V)')
        #ax2.set_title('Battery Voltage')
        ax2.grid(True, alpha=0.3)
    
    ax3 = fig.add_subplot(gs[2, 0])
    if battery_data['time'] and battery_data['current']:
        ax3.plot(battery_data['time'], battery_data['current'], 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Current (A)')
        #ax3.set_title('Current Draw')
        ax3.grid(True, alpha=0.3)
    
    ax4 = fig.add_subplot(gs[3, 0])
    if battery_data['voltage'] and battery_data['current'] and battery_data['time']:
        power = np.array(battery_data['voltage']) * np.array(battery_data['current'])
        ax4.plot(battery_data['time'], power, 'purple', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Power (W)')
        #ax4.set_title('Power Consumption')
        ax4.grid(True, alpha=0.3)
    return fig


def main():
    parser = argparse.ArgumentParser(description='Visualize ROS bag data')
    parser.add_argument('bag_path', type=str, help='Path to the ROS bag file')
    args = parser.parse_args()
    
    try:
        battery_data, pose_data, odom_data = extract_data(args.bag_path)
        
        combined_fig = create_combined_visualization(battery_data, pose_data, odom_data)
        combined_fig.savefig('rosbag_analysis.png', dpi=150, bbox_inches='tight')
        print("Saved: rosbag_analysis.png")
        
        plt.show()
        
    except Exception as e:
        print(f"Error processing bag file: {e}")
        import traceback
        traceback.print_exc()
        raise

if __name__ == "__main__":
    main()
