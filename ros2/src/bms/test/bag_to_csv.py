#!/usr/bin/env python3
# source install/setup.zsh
# py -m src.bms.test.bag_to_csv src/bms/test/rosbag_energy_20250829_065753_0.mcap -o src/bms/test/export

import numpy as np
import pandas as pd
import os
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag_file = "./src/bms/test/rosbag_long_term/rosbag_energy_20250829_065753_0.mcap"
output_dir = "./src/bms/test/lt_export"

def extract_battery_data(bag_path):
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
    
    return battery_data

def save_csv_files(battery_data, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    
    # Save voltage
    voltage_df = pd.DataFrame(battery_data['voltage'])
    voltage_file = os.path.join(output_dir, 'export_u.csv')
    voltage_df.to_csv(voltage_file, index=False, header=False)
    
    # Save current
    current_df = pd.DataFrame(battery_data['current'])
    current_file = os.path.join(output_dir, 'export_i.csv')
    current_df.to_csv(current_file, index=False, header=False)
    
    soc_data = np.array(battery_data['percentage']) / 100.0
    soc_df = pd.DataFrame(soc_data)
    soc_file = os.path.join(output_dir, 'export_soc.csv')
    soc_df.to_csv(soc_file, index=False, header=False)
    

def main():
    print(f"Processing: {bag_file}")
    print(f"Output to: {output_dir}")
    
    battery_data = extract_battery_data(bag_file)
    save_csv_files(battery_data, output_dir)
    
    print("Done.")

if __name__ == "__main__":
    main()
