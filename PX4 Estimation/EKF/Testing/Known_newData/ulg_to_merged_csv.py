#!/usr/bin/env python3

import os
import argparse
import numpy as np
import pandas as pd
from pyulog import ULog
from pyulog.px4 import PX4ULog

def quaternion_to_euler(w, x, y, z):
    """Convert quaternion to Euler angles (roll, pitch, yaw)"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def find_topic_instance(data_list, topic_name, instance=0):
    """Find a specific topic instance in ULog data"""
    for d in data_list:
        if d.name == topic_name and d.multi_id == instance:
            return d
    return None

def ulg_to_merged_csv(ulg_file, output_file=None, convert_to_degrees=False):
    """Convert a ULog file to a merged CSV with aligned sensor and attitude data"""
    print(f"Processing ULog file: {ulg_file}")
    
    # Generate output filename if not specified
    if output_file is None:
        base_name = os.path.splitext(os.path.basename(ulg_file))[0]
        output_file = f"{base_name}_merged_data.csv"
    
    # Load ULog file
    try:
        ulog = ULog(ulg_file)
        px4_ulog = PX4ULog(ulog)
        data_list = ulog.data_list
    except Exception as e:
        print(f"Error reading ULog file: {e}")
        return None
    
    print(f"Log contains {len(data_list)} data topics")
    
    # Find relevant topics
    accel_data = find_topic_instance(data_list, 'sensor_accel', 0)
    gyro_data = find_topic_instance(data_list, 'sensor_gyro', 0)
    mag_data = find_topic_instance(data_list, 'sensor_mag', 0)
    if mag_data is None:
        mag_data = find_topic_instance(data_list, 'vehicle_magnetometer', 0)
    attitude_data = find_topic_instance(data_list, 'vehicle_attitude', 0)
    
    # Check if we have the minimum required topics
    if accel_data is None:
        print("No accelerometer data found in log")
        accel_alt = find_topic_instance(data_list, 'sensor_combined', 0)
        if accel_alt:
            print("Using sensor_combined as alternative for accelerometer data")
            accel_data = accel_alt
        else:
            available_topics = [d.name for d in data_list]
            print(f"Available topics: {available_topics}")
            return None
    
    # Create base dataframe with accelerometer timestamps
    print("Creating merged dataframe...")
    data = {}
    
    # Start with accelerometer data
    accel_df = pd.DataFrame(accel_data.data)
    base_timestamps = accel_df['timestamp'].values / 1e6  # Convert to seconds
    rows = len(base_timestamps)
    
    data['Timestamp'] = base_timestamps
    
    # Get column names for accelerometer data
    accel_xyz_cols = [col for col in accel_df.columns if col in ['x', 'y', 'z']]
    if len(accel_xyz_cols) == 3:
        data['Accelerometer_X'] = accel_df[accel_xyz_cols[0]].values
        data['Accelerometer_Y'] = accel_df[accel_xyz_cols[1]].values
        data['Accelerometer_Z'] = accel_df[accel_xyz_cols[2]].values
    else:
        # Try alternate column names
        accel_xyz_cols = [col for col in accel_df.columns if 'acc' in col.lower() and any(ax in col.lower() for ax in ['x', 'y', 'z'])]
        if len(accel_xyz_cols) >= 3:
            data['Accelerometer_X'] = accel_df[accel_xyz_cols[0]].values
            data['Accelerometer_Y'] = accel_df[accel_xyz_cols[1]].values
            data['Accelerometer_Z'] = accel_df[accel_xyz_cols[2]].values
        else:
            print(f"Warning: Could not identify accelerometer columns clearly. Available columns: {accel_df.columns}")
            data['Accelerometer_X'] = np.nan
            data['Accelerometer_Y'] = np.nan
            data['Accelerometer_Z'] = np.nan
    
    # Add gyroscope data if available
    if gyro_data:
        print("Adding gyroscope data...")
        gyro_df = pd.DataFrame(gyro_data.data)
        gyro_time = gyro_df['timestamp'].values / 1e6
        
        gyro_xyz_cols = [col for col in gyro_df.columns if col in ['x', 'y', 'z']]
        if len(gyro_xyz_cols) == 3:
            # Interpolate to match base timestamps
            gyro_x = np.interp(base_timestamps, gyro_time, gyro_df[gyro_xyz_cols[0]].values)
            gyro_y = np.interp(base_timestamps, gyro_time, gyro_df[gyro_xyz_cols[1]].values)
            gyro_z = np.interp(base_timestamps, gyro_time, gyro_df[gyro_xyz_cols[2]].values)
            
            data['Gyroscope_X'] = gyro_x
            data['Gyroscope_Y'] = gyro_y
            data['Gyroscope_Z'] = gyro_z
        else:
            print(f"Warning: Could not identify gyroscope columns clearly. Available columns: {gyro_df.columns}")
            data['Gyroscope_X'] = np.nan
            data['Gyroscope_Y'] = np.nan
            data['Gyroscope_Z'] = np.nan
    else:
        print("No gyroscope data found")
        data['Gyroscope_X'] = np.nan
        data['Gyroscope_Y'] = np.nan
        data['Gyroscope_Z'] = np.nan
    
    # Add magnetometer data if available
    if mag_data:
        print("Adding magnetometer data...")
        mag_df = pd.DataFrame(mag_data.data)
        mag_time = mag_df['timestamp'].values / 1e6
        
        mag_xyz_cols = [col for col in mag_df.columns if col in ['x', 'y', 'z']]
        if len(mag_xyz_cols) == 3:
            # Interpolate to match base timestamps
            mag_x = np.interp(base_timestamps, mag_time, mag_df[mag_xyz_cols[0]].values)
            mag_y = np.interp(base_timestamps, mag_time, mag_df[mag_xyz_cols[1]].values)
            mag_z = np.interp(base_timestamps, mag_time, mag_df[mag_xyz_cols[2]].values)
            
            data['Magnetometer_X'] = mag_x
            data['Magnetometer_Y'] = mag_y
            data['Magnetometer_Z'] = mag_z
        elif any('magnetometer_ga' in col for col in mag_df.columns):
            # Handle PX4 specific magnetometer format
            mag_x = np.interp(base_timestamps, mag_time, mag_df['magnetometer_ga[0]'].values)
            mag_y = np.interp(base_timestamps, mag_time, mag_df['magnetometer_ga[1]'].values)
            mag_z = np.interp(base_timestamps, mag_time, mag_df['magnetometer_ga[2]'].values)
            
            data['Magnetometer_X'] = mag_x
            data['Magnetometer_Y'] = mag_y
            data['Magnetometer_Z'] = mag_z
        else:
            print(f"Warning: Could not identify magnetometer columns clearly. Available columns: {mag_df.columns}")
            data['Magnetometer_X'] = np.nan
            data['Magnetometer_Y'] = np.nan
            data['Magnetometer_Z'] = np.nan
    else:
        print("No magnetometer data found")
        data['Magnetometer_X'] = np.nan
        data['Magnetometer_Y'] = np.nan
        data['Magnetometer_Z'] = np.nan
    
    # Add attitude data if available
    if attitude_data:
        print("Adding attitude data...")
        att_df = pd.DataFrame(attitude_data.data)
        att_time = att_df['timestamp'].values / 1e6
        
        # Find quaternion columns
        quat_cols = []
        for col in att_df.columns:
            if 'q[' in col or col == 'q0' or col == 'q1' or col == 'q2' or col == 'q3':
                quat_cols.append(col)
        
        if len(quat_cols) >= 4:
            quat_cols.sort()  # Ensure they're in order
            
            # Interpolate quaternions
            quat_w = np.interp(base_timestamps, att_time, att_df[quat_cols[0]].values)
            quat_x = np.interp(base_timestamps, att_time, att_df[quat_cols[1]].values)
            quat_y = np.interp(base_timestamps, att_time, att_df[quat_cols[2]].values)
            quat_z = np.interp(base_timestamps, att_time, att_df[quat_cols[3]].values)
            
            data['Quat_w'] = quat_w
            data['Quat_x'] = quat_x
            data['Quat_y'] = quat_y
            data['Quat_z'] = quat_z
            
            # Calculate roll, pitch, yaw
            roll = []
            pitch = []
            yaw = []
            
            for i in range(rows):
                r, p, y = quaternion_to_euler(quat_w[i], quat_x[i], quat_y[i], quat_z[i])
                if convert_to_degrees:
                    r, p, y = np.degrees(r), np.degrees(p), np.degrees(y)
                roll.append(r)
                pitch.append(p)
                yaw.append(y)
            
            data['Roll'] = roll
            data['Pitch'] = pitch
            data['Yaw'] = yaw
        else:
            print(f"Warning: Could not identify quaternion columns clearly. Available columns: {att_df.columns}")
            # Fill with NaN
            data['Quat_w'] = np.nan
            data['Quat_x'] = np.nan
            data['Quat_y'] = np.nan
            data['Quat_z'] = np.nan
            data['Roll'] = np.nan
            data['Pitch'] = np.nan
            data['Yaw'] = np.nan
    else:
        print("No attitude data found")
        # Fill with NaN
        data['Quat_w'] = np.nan
        data['Quat_x'] = np.nan
        data['Quat_y'] = np.nan
        data['Quat_z'] = np.nan
        data['Roll'] = np.nan
        data['Pitch'] = np.nan
        data['Yaw'] = np.nan
    
    # Create the merged dataframe
    merged_df = pd.DataFrame(data)
    
    # Ensure all required columns exist
    required_columns = [
        'Timestamp', 
        'Accelerometer_X', 'Accelerometer_Y', 'Accelerometer_Z',
        'Gyroscope_X', 'Gyroscope_Y', 'Gyroscope_Z',
        'Magnetometer_X', 'Magnetometer_Y', 'Magnetometer_Z',
        'Roll', 'Pitch', 'Yaw',
        'Quat_w', 'Quat_x', 'Quat_y', 'Quat_z'
    ]
    
    for col in required_columns:
        if col not in merged_df.columns:
            merged_df[col] = np.nan
    
    # Reorder columns to match required format
    merged_df = merged_df[required_columns]
    
    # Save to CSV
    print(f"Saving merged data to {output_file}")
    merged_df.to_csv(output_file, index=False)
    print(f"Successfully saved merged data with {len(merged_df)} rows")
    
    return merged_df

def main():
    parser = argparse.ArgumentParser(description='Convert ULog file to merged CSV with sensor and attitude data')
    parser.add_argument('ulg_file', help='Path to the ULog file')
    parser.add_argument('-o', '--output', help='Output CSV file path')
    parser.add_argument('-d', '--degrees', action='store_true', help='Convert angles to degrees')
    
    args = parser.parse_args()
    
    ulg_to_merged_csv(args.ulg_file, args.output, args.degrees)

if __name__ == "__main__":
    main()
