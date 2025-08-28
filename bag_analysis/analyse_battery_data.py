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
import gc

sns.set_style("darkgrid")

def downsample_data(data_dict, max_points=10000):
    if not data_dict['time']:
        return data_dict
    
    n_points = len(data_dict['time'])
    if n_points <= max_points:
        return data_dict
    
    print(f"Downsampling from {n_points} to {max_points} points")
    indices = np.linspace(0, n_points-1, max_points, dtype=int)
    
    downsampled = {}
    for key, values in data_dict.items():
        if values:
            downsampled[key] = [values[i] for i in indices]
    
    return downsampled

def extract_data(bag_path, downsample_threshold=10000):
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
    sample_counter = {'battery': 0, 'pose': 0, 'odom': 0}
    sampling_rate = {'battery': 1, 'pose': 1, 'odom': 1}
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if start_time is None:
            start_time = timestamp
        
        rel_time = (timestamp - start_time) / 1e9
        
        if topic == '/battery_state_enhanced':
            sample_counter['battery'] += 1
            if sample_counter['battery'] % sampling_rate['battery'] == 0:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                battery_data['time'].append(rel_time)
                battery_data['voltage'].append(msg.voltage)
                battery_data['current'].append(msg.current)
                battery_data['charge'].append(msg.charge)
                battery_data['capacity'].append(msg.capacity)
                battery_data['percentage'].append(msg.percentage)
                
                if len(battery_data['time']) > downsample_threshold * 2:
                    battery_data = downsample_data(battery_data, downsample_threshold)
                    sampling_rate['battery'] *= 2
            
        elif topic == '/do150_0007/amcl_pose':
            sample_counter['pose'] += 1
            if sample_counter['pose'] % sampling_rate['pose'] == 0:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                pose_data['time'].append(rel_time)
                pose_data['x'].append(msg.pose.pose.position.x)
                pose_data['y'].append(msg.pose.pose.position.y)
                pose_data['z'].append(msg.pose.pose.position.z)
                
                if len(pose_data['time']) > downsample_threshold * 2:
                    pose_data = downsample_data(pose_data, downsample_threshold)
                    sampling_rate['pose'] *= 2
            
        elif topic == '/do150_0007/odom':
            sample_counter['odom'] += 1
            if sample_counter['odom'] % sampling_rate['odom'] == 0:
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
                
                if len(odom_data['time']) > downsample_threshold * 2:
                    odom_data = downsample_data(odom_data, downsample_threshold)
                    sampling_rate['odom'] *= 2
    
    #reader.close()
    
    print(f"Data extraction complete")
    print(f"Battery samples: {len(battery_data['time'])} (total processed: {sample_counter['battery']})")
    print(f"Pose samples: {len(pose_data['time'])} (total processed: {sample_counter['pose']})")
    print(f"Odometry samples: {len(odom_data['time'])} (total processed: {sample_counter['odom']})")
    
    if len(battery_data['time']) > downsample_threshold:
        battery_data = downsample_data(battery_data, downsample_threshold)
    if len(pose_data['time']) > downsample_threshold:
        pose_data = downsample_data(pose_data, downsample_threshold)
    if len(odom_data['time']) > downsample_threshold:
        odom_data = downsample_data(odom_data, downsample_threshold)
    
    gc.collect()
    
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

def plot_energy_analysis(battery_data):
    fig = plt.figure(figsize=(15, 10))
    gs = gridspec.GridSpec(3, 2, figure=fig)
    
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(battery_data['time'], battery_data['percentage'], 'b-', linewidth=2)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Battery Percentage (%)')
    ax1.set_title('Battery State Over Time')
    ax1.grid(True, alpha=0.3)
    
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(battery_data['time'], battery_data['voltage'], 'g-', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Voltage (V)')
    ax2.set_title('Battery Voltage')
    ax2.grid(True, alpha=0.3)
    
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(battery_data['time'], battery_data['current'], 'r-', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Current (A)')
    ax3.set_title('Current Draw')
    ax3.grid(True, alpha=0.3)
    
    ax4 = fig.add_subplot(gs[2, 0])
    if battery_data['voltage'] and battery_data['current']:
        power = np.array(battery_data['voltage']) * np.array(battery_data['current'])
        ax4.plot(battery_data['time'], power, 'purple', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Power (W)')
        ax4.set_title('Power Consumption')
        ax4.grid(True, alpha=0.3)
    
    ax5 = fig.add_subplot(gs[2, 1])
    if battery_data['charge'] and battery_data['capacity']:
        charge_ratio = np.array(battery_data['charge']) / (np.array(battery_data['capacity']) + 1e-6)
        ax5.plot(battery_data['time'], charge_ratio, 'orange', linewidth=2)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Charge Ratio')
        ax5.set_title('Charge / Capacity Ratio')
        ax5.grid(True, alpha=0.3)
    
    plt.suptitle('Energy Analysis', fontsize=16, fontweight='bold')
    plt.tight_layout()
    return fig

def plot_motion_analysis(odom_data):
    fig = plt.figure(figsize=(15, 6))
    gs = gridspec.GridSpec(1, 2, figure=fig)
    
    speed = calculate_speed(odom_data['vx'], odom_data['vy'], odom_data['vz'])
    
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(odom_data['time'], speed, 'b-', linewidth=2)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Speed (m/s)')
    ax1.set_title('Total Speed Over Time')
    ax1.grid(True, alpha=0.3)
    
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.hist(speed, bins=50, color='skyblue', edgecolor='black', alpha=0.7)
    ax2.set_xlabel('Speed (m/s)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Speed Distribution')
    ax2.axvline(np.mean(speed), color='red', linestyle='--', label=f'Mean: {np.mean(speed):.2f} m/s')
    ax2.axvline(np.median(speed), color='green', linestyle='--', label=f'Median: {np.median(speed):.2f} m/s')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle('Motion Analysis', fontsize=16, fontweight='bold')
    plt.tight_layout()
    return fig

def plot_position_analysis(pose_data, odom_data):
    fig = plt.figure(figsize=(15, 8))
    gs = gridspec.GridSpec(1, 2, figure=fig)
    
    ax1 = fig.add_subplot(gs[0, 0])
    if pose_data['x'] and pose_data['y']:
        ax1.plot(pose_data['x'], pose_data['y'], 'b-', linewidth=2, label='AMCL Pose', alpha=0.8)
        ax1.scatter(pose_data['x'][0], pose_data['y'][0], c='green', s=100, marker='o', label='Start', zorder=5)
        ax1.scatter(pose_data['x'][-1], pose_data['y'][-1], c='red', s=100, marker='s', label='End', zorder=5)
    
    if odom_data['x'] and odom_data['y']:
        ax1.plot(odom_data['x'], odom_data['y'], 'r--', linewidth=1, label='Odometry', alpha=0.5)
    
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('2D Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    ax2 = fig.add_subplot(gs[0, 1], projection='3d')
    if odom_data['x'] and odom_data['y'] and odom_data['z']:
        downsample_3d = max(1, len(odom_data['x']) // 5000)
        x_3d = odom_data['x'][::downsample_3d]
        y_3d = odom_data['y'][::downsample_3d]
        z_3d = odom_data['z'][::downsample_3d]
        
        ax2.plot(x_3d, y_3d, z_3d, 'b-', linewidth=2)
        ax2.scatter(x_3d[0], y_3d[0], z_3d[0], c='green', s=100, marker='o')
        ax2.scatter(x_3d[-1], y_3d[-1], z_3d[-1], c='red', s=100, marker='s')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Z (m)')
        ax2.set_title('3D Trajectory (Odometry)')
    
    plt.suptitle('Position Analysis', fontsize=16, fontweight='bold')
    plt.tight_layout()
    return fig

def plot_distance_analysis(pose_data, odom_data):
    fig = plt.figure(figsize=(15, 8))
    gs = gridspec.GridSpec(2, 2, figure=fig)
    
    odom_distance = calculate_distance(odom_data['x'], odom_data['y'], odom_data['z']) if odom_data['x'] else np.array([])
    pose_distance = calculate_distance(pose_data['x'], pose_data['y'], pose_data['z']) if pose_data['x'] else np.array([])
    
    ax1 = fig.add_subplot(gs[0, :])
    if odom_distance.any():
        ax1.plot(odom_data['time'], odom_distance, 'b-', linewidth=2, label='Odometry')
    if pose_distance.any():
        ax1.plot(pose_data['time'], pose_distance, 'r--', linewidth=2, label='AMCL')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Cumulative Distance (m)')
    ax1.set_title('Distance Traveled Over Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2 = fig.add_subplot(gs[1, 0])
    if len(odom_distance) > 1 and len(odom_data['time']) > 1:
        distance_rate = np.diff(odom_distance) / (np.diff(odom_data['time']) + 1e-6)
        ax2.plot(odom_data['time'][1:], distance_rate, 'g-', linewidth=1)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Distance Rate (m/s)')
        ax2.set_title('Instantaneous Distance Rate')
        ax2.grid(True, alpha=0.3)
    
    ax3 = fig.add_subplot(gs[1, 1])
    stats_text = []
    if odom_distance.any():
        stats_text.append(f"Total Distance (Odom): {odom_distance[-1]:.2f} m")
    if pose_distance.any():
        stats_text.append(f"Total Distance (AMCL): {pose_distance[-1]:.2f} m")
    if odom_data['time']:
        total_time = odom_data['time'][-1] - odom_data['time'][0]
        stats_text.append(f"Total Time: {total_time:.2f} s")
        stats_text.append(f"Total Time: {total_time/3600:.2f} hours")
        if odom_distance.any() and total_time > 0:
            avg_speed = odom_distance[-1] / total_time
            stats_text.append(f"Average Speed: {avg_speed:.3f} m/s")
    
    ax3.text(0.1, 0.5, '\n'.join(stats_text), transform=ax3.transAxes, 
             fontsize=12, verticalalignment='center', fontfamily='monospace')
    ax3.set_title('Summary Statistics')
    ax3.axis('off')
    
    plt.suptitle('Distance Analysis', fontsize=16, fontweight='bold')
    plt.tight_layout()
    return fig

def main():
    parser = argparse.ArgumentParser(description='Visualize ROS bag data')
    parser.add_argument('bag_path', type=str, help='Path to the ROS bag file')
    parser.add_argument('--max-points', type=int, default=10000, 
                       help='Maximum points per plot (for memory management)')
    args = parser.parse_args()
    
    try:
        battery_data, pose_data, odom_data = extract_data(args.bag_path, args.max_points)
        
        print("Creating visualizations...")
        
        if battery_data['time']:
            energy_fig = plot_energy_analysis(battery_data)
            energy_fig.savefig('energy_analysis.png', dpi=150, bbox_inches='tight')
            print("Saved: energy_analysis.png")
            plt.close(energy_fig)
            gc.collect()
        
        if odom_data['time']:
            motion_fig = plot_motion_analysis(odom_data)
            motion_fig.savefig('motion_analysis.png', dpi=150, bbox_inches='tight')
            print("Saved: motion_analysis.png")
            plt.close(motion_fig)
            gc.collect()
        
        if pose_data['time'] or odom_data['time']:
            position_fig = plot_position_analysis(pose_data, odom_data)
            position_fig.savefig('position_analysis.png', dpi=150, bbox_inches='tight')
            print("Saved: position_analysis.png")
            plt.close(position_fig)
            gc.collect()
        
        if pose_data['time'] or odom_data['time']:
            distance_fig = plot_distance_analysis(pose_data, odom_data)
            distance_fig.savefig('distance_analysis.png', dpi=150, bbox_inches='tight')
            print("Saved: distance_analysis.png")
            plt.close(distance_fig)
            gc.collect()
        
        print("Visualization complete!")
        print(f"Memory efficient processing used for large datasets")
        
    except Exception as e:
        print(f"Error processing bag file: {e}")
        import traceback
        traceback.print_exc()
        raise

if __name__ == "__main__":
    main()