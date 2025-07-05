#!/usr/bin/env python3
"""
ULog to Combined Sensor CSV Converter
Extracts specific sensor fields and combines them into synchronized CSV
"""

import pyulog
import pandas as pd
import numpy as np
import os
from scipy import interpolate
from scipy.spatial.transform import Rotation as R

def extract_sensor_data_to_csv(ulog_file, output_file=None, target_freq_hz=50, method='interpolation'):
    """
    Extract specific sensor fields from ULog and combine into single CSV
    
    Args:
        ulog_file (str): Path to .ulg file
        output_file (str): Output CSV filename (optional)
        target_freq_hz (int): Target sampling frequency in Hz (for interpolation method)
        method (str): 'interpolation' or 'merge_asof' for synchronization method
    
    Returns:
        pd.DataFrame: Combined sensor data
    """
    
    # Load ULog with required topics
    topics = ['sensor_combined', 'vehicle_attitude']
    try:
        ulog = pyulog.ULog(ulog_file, topics)
        print(f"Loaded: {ulog_file}")
        # print(f"Duration: {ulog.get_duration():.1f} seconds")
        duration_sec = (ulog.last_timestamp - ulog.start_timestamp) / 1e6
        print(f"Duration: {duration_sec:.1f} seconds")
        print(f"Synchronization method: {method}")
        print("Available topics:", [d.name for d in ulog.data_list])
    except Exception as e:
        print(f"Error loading ULog: {e}")
        return None
    
    # Choose synchronization method
    if method == 'merge_asof':
        combined_data = basic_merge_approach(ulog_file)
        # Process the merged data to match our column format
        result_df = process_merged_data(combined_data)
    else:  # interpolation method (default)
        # Extract sensor_combined data
        sensor_data = None
        attitude_data = None
        
        for data in ulog.data_list:
            if data.name == 'sensor_combined':
                print("Available sensor_combined fields:", list(data.data.keys()))
                sensor_df = pd.DataFrame({
                    'timestamp': data.data['timestamp'],
                    'Accelerometer_X': data.data['accelerometer_m_s2[0]'],
                    'Accelerometer_Y': data.data['accelerometer_m_s2[1]'], 
                    'Accelerometer_Z': data.data['accelerometer_m_s2[2]'],
                    'Gyroscope_X': data.data['gyro_rad[0]'],
                    'Gyroscope_Y': data.data['gyro_rad[1]'],
                    'Gyroscope_Z': data.data['gyro_rad[2]'],
                    # 'Magnetometer_X': data.data['magnetometer_ga[0]'],
                    # 'Magnetometer_Y': data.data['magnetometer_ga[1]'],
                    # 'Magnetometer_Z': data.data['magnetometer_ga[2]']
                })
                sensor_data = sensor_df
                
            elif data.name == 'vehicle_attitude':
                attitude_df = pd.DataFrame({
                    'timestamp': data.data['timestamp'],
                    # 'Roll': data.data['roll'],
                    # 'Pitch': data.data['pitch'],
                    # 'Yaw': data.data['yaw'],
                    'Quat_w': data.data['q[0]'],
                    'Quat_x': data.data['q[1]'],
                    'Quat_y': data.data['q[2]'],
                    'Quat_z': data.data['q[3]']
                })
                attitude_data = attitude_df
                quats = np.vstack((attitude_df['Quat_w'], attitude_df['Quat_x'], attitude_df['Quat_y'], attitude_df['Quat_z'])).T

                rpy = R.from_quat(quats).as_euler('xyz', degrees=False)

                attitude_df['Roll'] = rpy[:, 0]
                attitude_df['Pitch'] = rpy[:, 1]
                attitude_df['Yaw'] = rpy[:, 2]
        
        if sensor_data is None or attitude_data is None:
            print("Required topics not found in ULog file")
            return None
        
        print(f"Sensor data: {len(sensor_data)} points")
        print(f"Attitude data: {len(attitude_data)} points")
        
        # Synchronize timestamps using interpolation
        combined_data = synchronize_sensor_streams(sensor_data, attitude_data, target_freq_hz)
        
        # Convert timestamp to relative seconds and reorder columns
        combined_data['Timestamp'] = (combined_data['timestamp'] - combined_data['timestamp'].iloc[0]) / 1e6
        
        # Reorder columns to match requested format
        final_columns = [
            'Timestamp',
            'Accelerometer_X', 'Accelerometer_Y', 'Accelerometer_Z',
            'Gyroscope_X', 'Gyroscope_Y', 'Gyroscope_Z', 
            'Roll', 'Pitch', 'Yaw',
            'Quat_w', 'Quat_x', 'Quat_y', 'Quat_z'
        ]
        
        result_df = combined_data[final_columns]
    
    # Save to CSV
    if output_file is None:
        base_name = os.path.splitext(ulog_file)[0]
        method_suffix = "_merged" if method == 'merge_asof' else "_interpolated"
        output_file = f"{base_name}_combined_sensors{method_suffix}.csv"
    
    result_df.to_csv(output_file, index=False)
    print(f"Combined CSV saved: {output_file}")
    print(f"Shape: {result_df.shape}")
    
    return result_df

def basic_merge_approach(ulog_file):
    """Simple approach using pandas merge_asof for timestamp alignment"""
    
    ulog = pyulog.ULog(ulog_file, ['sensor_combined', 'vehicle_attitude'])
    
    # Extract data as DataFrames
    dataframes = {}
    for data in ulog.data_list:
        df = pd.DataFrame(data.data)
        df = df.sort_values('timestamp')  # Ensure sorted for merge_asof
        dataframes[data.name] = df
    
    # Merge using nearest timestamp matching
    combined = pd.merge_asof(
        dataframes['sensor_combined'],
        dataframes['vehicle_attitude'], 
        on='timestamp',
        direction='nearest',
        suffixes=('', '_att')
    )
    
    return combined

def process_merged_data(combined_data):
    """
    Process the merged data from basic_merge_approach to match our desired column format
    """
    
    # Convert timestamp to relative seconds
    combined_data['Timestamp'] = (combined_data['timestamp'] - combined_data['timestamp'].iloc[0]) / 1e6
    
    # Create DataFrame with our desired column names and order
    result_df = pd.DataFrame({
        'Timestamp': combined_data['Timestamp'],
        'Accelerometer_X': combined_data['accelerometer_m_s2[0]'],
        'Accelerometer_Y': combined_data['accelerometer_m_s2[1]'],
        'Accelerometer_Z': combined_data['accelerometer_m_s2[2]'],
        'Gyroscope_X': combined_data['gyro_rad_s[0]'],
        'Gyroscope_Y': combined_data['gyro_rad_s[1]'],
        'Gyroscope_Z': combined_data['gyro_rad_s[2]'],
        'Magnetometer_X': combined_data['magnetometer_ga[0]'],
        'Magnetometer_Y': combined_data['magnetometer_ga[1]'],
        'Magnetometer_Z': combined_data['magnetometer_ga[2]'],
        'Roll': combined_data['roll'],
        'Pitch': combined_data['pitch'],
        'Yaw': combined_data['yaw'],
        'Quat_w': combined_data['q[0]'],
        'Quat_x': combined_data['q[1]'],
        'Quat_y': combined_data['q[2]'],
        'Quat_z': combined_data['q[3]']
    })
    
    print(f"Merged data: {len(result_df)} points")
    
    return result_df

def synchronize_sensor_streams(sensor_data, attitude_data, target_freq_hz=50):
    """
    Synchronize sensor and attitude data to common timeline using interpolation
    """
    
    # Find common time window
    start_time = max(sensor_data['timestamp'].min(), attitude_data['timestamp'].min())
    end_time = min(sensor_data['timestamp'].max(), attitude_data['timestamp'].max())
    
    # Create target timeline at specified frequency
    dt_us = int(1e6 / target_freq_hz)  # microseconds between samples
    target_timestamps = np.arange(start_time, end_time, dt_us)
    
    print(f"Synchronizing to {target_freq_hz} Hz ({len(target_timestamps)} points)")
    
    # Interpolate sensor data to target timeline
    synchronized_data = {'timestamp': target_timestamps}
    
    # Interpolate sensor_combined fields
    for column in sensor_data.columns:
        if column != 'timestamp':
            interp_func = interpolate.interp1d(
                sensor_data['timestamp'], 
                sensor_data[column],
                kind='linear', 
                bounds_error=False,
                fill_value='extrapolate'
            )
            synchronized_data[column] = interp_func(target_timestamps)
    
    # Interpolate attitude fields  
    for column in attitude_data.columns:
        if column != 'timestamp':
            interp_func = interpolate.interp1d(
                attitude_data['timestamp'],
                attitude_data[column], 
                kind='linear',
                bounds_error=False,
                fill_value='extrapolate'
            )
            synchronized_data[column] = interp_func(target_timestamps)
    
    return pd.DataFrame(synchronized_data)

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Extract and combine ULog sensor data to CSV')
    parser.add_argument('input_file', help='Input .ulg file')
    parser.add_argument('-o', '--output', help='Output CSV file')
    parser.add_argument('-f', '--frequency', type=int, default=50, 
                       help='Target frequency in Hz (default: 50, only for interpolation method)')
    parser.add_argument('-m', '--method', choices=['interpolation', 'merge_asof'], 
                       default='interpolation',
                       help='Synchronization method: interpolation (default) or merge_asof')
    
    args = parser.parse_args()
    
    df = extract_sensor_data_to_csv(args.input_file, args.output, args.frequency, args.method)
    
    if df is not None:
        print("\nFirst few rows:")
        print(df.head())
        print(f"\nSample timestamps (first 5 rows):")
        print(df['Timestamp'].head())