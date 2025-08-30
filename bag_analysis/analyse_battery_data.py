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
    
    ##reader.close()
    
    print(f"Data extraction complete")
    print(f"Battery samples: {len(battery_data['time'])}")
    print(f"Pose samples: {len(pose_data['time'])}")
    print(f"Odometry samples: {len(odom_data['time'])}")
    
    return battery_data, pose_data, odom_data

def calculate_distance(x, y, z=None):
    if not x or not y:
        return np.array([])
    
    if z is None:
        z = np.zeros_like(x)
    
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)
    
    dx = np.diff(x)
    dy = np.diff(y)
    dz = np.diff(z)
    
    distances = np.sqrt(dx**2 + dy**2 + dz**2)
    cumulative_distance = np.concatenate([[0], np.cumsum(distances)])
    
    return cumulative_distance

def calculate_speed(vx, vy, vz):
    return np.sqrt(np.array(vx)**2 + np.array(vy)**2 + np.array(vz)**2)

def create_combined_visualization(battery_data, pose_data, odom_data):
    num_plots = 6
    fig = plt.figure(figsize=(12, 20))
    gs = gridspec.GridSpec(num_plots, 1, figure=fig, hspace=0.3, height_ratios=[1, 1, 1, 1, 1, 1.5])
    
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
    
    odom_distance = calculate_distance(odom_data['x'], odom_data['y'], odom_data['z']) if odom_data['x'] else np.array([])
    pose_distance = calculate_distance(pose_data['x'], pose_data['y'], pose_data['z']) if pose_data['x'] else np.array([])
    
    ax5 = fig.add_subplot(gs[4, 0])
    if odom_distance.any() and odom_data['time']:
        ax5.plot(odom_data['time'], odom_distance, 'b-', linewidth=2, label='Odometry')
    if pose_distance.any() and pose_data['time']:
        ax5.plot(pose_data['time'], pose_distance, 'r--', linewidth=2, label='AMCL')
    if odom_distance.any() or pose_distance.any():
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Cumulative Distance (m)')
        #ax5.set_title('Distance Traveled Over Time')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
    
    ax6 = fig.add_subplot(gs[5, 0])
    stats_text = []
    stats_text.append("=" * 50)
    stats_text.append("SUMMARY STATISTICS")
    stats_text.append("=" * 50)
    stats_text.append("")
    
    if battery_data['time'] and battery_data['percentage']:
        stats_text.append("BATTERY:")
        stats_text.append(f"  Initial Battery: {battery_data['percentage'][0]:.1f}%")
        stats_text.append(f"  Final Battery: {battery_data['percentage'][-1]:.1f}%")
        stats_text.append(f"  Battery Consumed: {battery_data['percentage'][0] - battery_data['percentage'][-1]:.1f}%")
        if battery_data['voltage']:
            stats_text.append(f"  Average Voltage: {np.mean(battery_data['voltage']):.2f} V")
            stats_text.append(f"  Min Voltage: {np.min(battery_data['voltage']):.2f} V")
            stats_text.append(f"  Max Voltage: {np.max(battery_data['voltage']):.2f} V")
        if battery_data['current']:
            stats_text.append(f"  Average Current: {np.mean(battery_data['current']):.2f} A")
            stats_text.append(f"  Max Current: {np.max(battery_data['current']):.2f} A")
        if battery_data['voltage'] and battery_data['current']:
            power = np.array(battery_data['voltage']) * np.array(battery_data['current'])
            stats_text.append(f"  Average Power: {np.mean(power):.2f} W")
            stats_text.append(f"  Max Power: {np.max(power):.2f} W")
        stats_text.append("")
    
    if odom_data['time'] and odom_data['vx']:
        stats_text.append("MOTION:")
        speed = calculate_speed(odom_data['vx'], odom_data['vy'], odom_data['vz'])
        stats_text.append(f"  Max Speed: {np.max(speed):.3f} m/s")
        stats_text.append(f"  Average Speed: {np.mean(speed):.3f} m/s")
        stats_text.append(f"  Median Speed: {np.median(speed):.3f} m/s")
        stats_text.append("")
    
    if odom_distance.any() or pose_distance.any():
        stats_text.append("DISTANCE:")
        if odom_distance.any():
            stats_text.append(f"  Total Distance (Odom): {odom_distance[-1]:.2f} m")
        if pose_distance.any():
            stats_text.append(f"  Total Distance (AMCL): {pose_distance[-1]:.2f} m")
        stats_text.append("")
    
    if odom_data['time'] or battery_data['time']:
        time_source = odom_data['time'] if odom_data['time'] else battery_data['time']
        if time_source:
            total_time = time_source[-1] - time_source[0]
            stats_text.append("TIME:")
            stats_text.append(f"  Total Time: {total_time:.2f} seconds")
            stats_text.append(f"  Total Time: {total_time/60:.2f} minutes")
            stats_text.append(f"  Total Time: {total_time/3600:.2f} hours")
            
            if odom_distance.any() and total_time > 0:
                avg_speed = odom_distance[-1] / total_time
                stats_text.append(f"  Overall Avg Speed: {avg_speed:.3f} m/s")
            
            if battery_data['percentage'] and odom_distance.any() and len(battery_data['percentage']) > 1:
                battery_consumed = battery_data['percentage'][0] - battery_data['percentage'][-1]
                if battery_consumed > 0:
                    efficiency = odom_distance[-1] / battery_consumed
                    stats_text.append(f"  Energy Efficiency: {efficiency:.2f} m per battery %")
    
    ax6.text(0.1, 0.95, '\n'.join(stats_text), transform=ax6.transAxes, 
             fontsize=11, verticalalignment='top', fontfamily='monospace')
    ax6.set_title('Summary Statistics')
    ax6.axis('off')
    
    plt.suptitle('ROS Bag Analysis Report', fontsize=18, fontweight='bold', y=0.995)
    plt.tight_layout()
    return fig

def main():
    parser = argparse.ArgumentParser(description='Visualize ROS bag data')
    parser.add_argument('bag_path', type=str, help='Path to the ROS bag file')
    args = parser.parse_args()
    
    try:
        battery_data, pose_data, odom_data = extract_data(args.bag_path)
        
        print("Creating visualization...")
        
        combined_fig = create_combined_visualization(battery_data, pose_data, odom_data)
        combined_fig.savefig('rosbag_analysis.png', dpi=150, bbox_inches='tight')
        print("Saved: rosbag_analysis.png")
        
        plt.show()
        print("Visualization complete!")
        
    except Exception as e:
        print(f"Error processing bag file: {e}")
        import traceback
        traceback.print_exc()
        raise

if __name__ == "__main__":
    main()