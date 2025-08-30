#!/usr/bin/env python3
# source install/setup.zsh
# py -m src.bms.test.bag_to_csv src/bms/test/rosbag_energy_20250829_065753_0.mcap -o src/bms/test/export

import argparse
import numpy as np
import pandas as pd
import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import BatteryState
from rosidl_runtime_py.utilities import get_message

def extract_battery_data(bag_path, output_dir='./'):
    print(f"Loading bag file: {bag_path}")
    print(f"Output directory: {output_dir}")
    
    battery_data = {
        'time': [],
        'voltage': [],
        'current': [],
        'charge': [],
        'capacity': [],
        'percentage': []
    }
    
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
    message_count = 0
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic == '/bms/state':
            if start_time is None:
                start_time = timestamp
            
            rel_time = (timestamp - start_time) / 1e9
            
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            battery_data['time'].append(rel_time)
            battery_data['voltage'].append(msg.voltage)
            battery_data['current'].append(msg.current)
            battery_data['charge'].append(msg.charge)
            battery_data['capacity'].append(msg.capacity)
            battery_data['percentage'].append(msg.percentage)
            
            message_count += 1
            
            if message_count % 1000 == 0:
                print(f"Processed {message_count} messages...")
    
    print(f"Total messages processed: {message_count}")
    
    return battery_data

def calculate_soc_from_percentage(percentage_data):
    percentage_array = np.array(percentage_data)
    soc = percentage_array / 100.0
    return soc

def save_csv_files(battery_data, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    
    voltage_df = pd.DataFrame(battery_data['voltage'])
    voltage_file = os.path.join(output_dir, 'export_u.csv')
    voltage_df.to_csv(voltage_file, index=False, header=False)
    print(f"Saved voltage data to: {voltage_file}")
    print(f"  Samples: {len(battery_data['voltage'])}")
    print(f"  Range: {np.min(battery_data['voltage']):.2f} - {np.max(battery_data['voltage']):.2f} V")
    
    current_df = pd.DataFrame(battery_data['current'])
    current_file = os.path.join(output_dir, 'export_i.csv')
    current_df.to_csv(current_file, index=False, header=False)
    print(f"Saved current data to: {current_file}")
    print(f"  Samples: {len(battery_data['current'])}")
    print(f"  Range: {np.min(battery_data['current']):.2f} - {np.max(battery_data['current']):.2f} A")
    
    soc_data = calculate_soc_from_percentage(battery_data['percentage'])
    soc_df = pd.DataFrame(soc_data)
    soc_file = os.path.join(output_dir, 'export_soc.csv')
    soc_df.to_csv(soc_file, index=False, header=False)
    print(f"Saved SOC data to: {soc_file}")
    print(f"  Samples: {len(soc_data)}")
    print(f"  Range: {np.min(soc_data):.3f} - {np.max(soc_data):.3f}")
    
    if 'charge' in battery_data and battery_data['charge']:
        if battery_data['capacity'] and battery_data['capacity'][0] > 0:
            soc_true = np.array(battery_data['charge']) / battery_data['capacity'][0]
        else:
            nominal_capacity = 44.0
            soc_true = np.array(battery_data['charge']) / nominal_capacity
            print(f"  Using nominal capacity: {nominal_capacity} Ah")
        
        soc_true_df = pd.DataFrame(soc_true)
        soc_true_file = os.path.join(output_dir, 'export_soc_true.csv')
        soc_true_df.to_csv(soc_true_file, index=False, header=False)
        print(f"Saved SOC true data to: {soc_true_file}")
        print(f"  Calculated from charge/capacity")
        print(f"  Range: {np.min(soc_true):.3f} - {np.max(soc_true):.3f}")
    
    return voltage_file, current_file, soc_file

def print_statistics(battery_data):
    print("\n" + "="*60)
    print("BATTERY DATA STATISTICS")
    print("="*60)
    
    if battery_data['time']:
        duration = battery_data['time'][-1] - battery_data['time'][0]
        print(f"\nTiming:")
        print(f"  Duration: {duration:.1f} seconds ({duration/60:.1f} minutes)")
        print(f"  Sampling rate: {len(battery_data['time'])/duration:.1f} Hz")
    
    if battery_data['voltage']:
        print(f"\nVoltage:")
        print(f"  Mean: {np.mean(battery_data['voltage']):.2f} V")
        print(f"  Std Dev: {np.std(battery_data['voltage']):.2f} V")
        print(f"  Min: {np.min(battery_data['voltage']):.2f} V")
        print(f"  Max: {np.max(battery_data['voltage']):.2f} V")
    
    if battery_data['current']:
        print(f"\nCurrent:")
        print(f"  Mean: {np.mean(battery_data['current']):.2f} A")
        print(f"  Std Dev: {np.std(battery_data['current']):.2f} A")
        print(f"  Min: {np.min(battery_data['current']):.2f} A")
        print(f"  Max: {np.max(battery_data['current']):.2f} A")
    
    if battery_data['percentage']:
        print(f"\nBattery Percentage:")
        print(f"  Initial: {battery_data['percentage'][0]:.1f}%")
        print(f"  Final: {battery_data['percentage'][-1]:.1f}%")
        print(f"  Consumed: {battery_data['percentage'][0] - battery_data['percentage'][-1]:.1f}%")
    
    if battery_data['capacity']:
        print(f"\nCapacity:")
        print(f"  Mean: {np.mean(battery_data['capacity']):.2f} Ah")
        if len(set(battery_data['capacity'])) == 1:
            print(f"  Constant value: {battery_data['capacity'][0]:.2f} Ah")
        else:
            print(f"  Min: {np.min(battery_data['capacity']):.2f} Ah")
            print(f"  Max: {np.max(battery_data['capacity']):.2f} Ah")
    
    if battery_data['charge']:
        print(f"\nCharge:")
        print(f"  Initial: {battery_data['charge'][0]:.2f} Ah")
        print(f"  Final: {battery_data['charge'][-1]:.2f} Ah")
        print(f"  Used: {battery_data['charge'][0] - battery_data['charge'][-1]:.2f} Ah")

def validate_data(battery_data):
    issues = []
    
    if not battery_data['voltage']:
        issues.append("No voltage data found")
    elif any(v <= 0 or v > 60 for v in battery_data['voltage']):
        issues.append("Voltage values out of reasonable range (0-60V)")
    
    if not battery_data['current']:
        issues.append("No current data found")
    elif any(abs(i) > 100 for i in battery_data['current']):
        issues.append("Current values exceed 100A")
    
    if not battery_data['percentage']:
        issues.append("No battery percentage data found")
    elif any(p < 0 or p > 100 for p in battery_data['percentage']):
        issues.append("Battery percentage out of range (0-100%)")
    
    lengths = [
        len(battery_data['voltage']),
        len(battery_data['current']),
        len(battery_data['percentage'])
    ]
    if len(set(lengths)) > 1:
        issues.append(f"Data length mismatch: voltage={lengths[0]}, current={lengths[1]}, percentage={lengths[2]}")
    
    if issues:
        print("\n" + "="*60)
        print("DATA VALIDATION WARNINGS:")
        print("="*60)
        for issue in issues:
            print(f"  - {issue}")
        print("")
    
    return len(issues) == 0

def main():
    parser = argparse.ArgumentParser(
        description='Convert ROS bag battery data to CSV files for BMS simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  %(prog)s my_rosbag.mcap
  %(prog)s my_rosbag.mcap -o ./output_folder
  %(prog)s my_rosbag.mcap -o ./bms_data --validate
  
Output files:
  export_u.csv      - Voltage measurements
  export_i.csv      - Current measurements  
  export_soc.csv    - State of Charge (from percentage)
  export_soc_true.csv - True SOC (from charge/capacity if available)
        """
    )
    
    parser.add_argument('bag_path', type=str, help='Path to the ROS bag file')
    parser.add_argument('-o', '--output', type=str, default='./', 
                       help='Output directory for CSV files (default: current directory)')
    parser.add_argument('--validate', action='store_true',
                       help='Perform data validation checks')
    parser.add_argument('--stats', action='store_true',
                       help='Show detailed statistics')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag_path):
        print(f"Error: Bag file not found: {args.bag_path}")
        return 1
    
    try:
        battery_data = extract_battery_data(args.bag_path, args.output)
        
        if not battery_data['voltage']:
            print("Error: No battery data found in bag file")
            print("Make sure the bag contains messages on topic: /bms/state")
            return 1
        
        if args.validate:
            is_valid = validate_data(battery_data)
            if not is_valid:
                response = input("Continue despite warnings? (y/n): ")
                if response.lower() != 'y':
                    print("Aborted by user")
                    return 1
        
        if args.stats:
            print_statistics(battery_data)
        
        print("\n" + "="*60)
        print("SAVING CSV FILES")
        print("="*60)
        
        voltage_file, current_file, soc_file = save_csv_files(battery_data, args.output)
        
        print("\n" + "="*60)
        print("CONVERSION COMPLETE")
        print("="*60)
        print(f"Files saved to: {os.path.abspath(args.output)}")
        print("\nYou can now use these files with your BMS simulation:")
        print(f"  voltage_file = '{voltage_file}'")
        print(f"  current_file = '{current_file}'")
        print(f"  soc_ref_file = '{soc_file}'")
        
        return 0
        
    except Exception as e:
        print(f"Error processing bag file: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    exit(main())