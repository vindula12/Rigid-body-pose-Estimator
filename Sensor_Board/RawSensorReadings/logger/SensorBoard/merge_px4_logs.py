import pandas as pd
import os
import numpy as np
from datetime import datetime

# Path to the directory containing the CSV files
csv_dir = "~/Desktop/px4_logs/csv_output/"
csv_dir = os.path.expanduser(csv_dir)  # Expand the ~ to the full home directory path

# Path for the output CSV
output_file = os.path.join(os.path.dirname(csv_dir), "merged_data.csv")

# List of expected files (these should match the topics you logged)
expected_files = [
    "sensor_accel_0.csv",
    "sensor_gyro_0.csv", 
    "sensor_mag_0.csv",
    "vehicle_attitude_0.csv",
    "ekf2_innovations_0.csv"
]

# Check which files actually exist
available_files = []
missing_files = []
for file in expected_files:
    file_path = os.path.join(csv_dir, file)
    if os.path.exists(file_path):
        available_files.append(file)
    else:
        missing_files.append(file)

if missing_files:
    print(f"Warning: The following expected files are missing: {', '.join(missing_files)}")

if not available_files:
    print("Error: No expected CSV files found in the directory.")
    exit(1)

# Function to load and prepare a dataframe
def load_dataframe(filename):
    file_path = os.path.join(csv_dir, filename)
    print(f"Loading {filename}...")
    df = pd.read_csv(file_path)
    
    # Convert timestamp to seconds (PX4 logs use microseconds)
    if 'timestamp' in df.columns:
        df['timestamp'] = df['timestamp'] / 1e6
    
    return df

# Load all available dataframes
dataframes = {}
for file in available_files:
    base_name = file.split('.')[0]  # Remove .csv
    dataframes[base_name] = load_dataframe(file)

# Create a merged dataframe starting with timestamps
# We'll use the accelerometer's timestamps as the base
if 'sensor_accel_0' in dataframes:
    merged_df = pd.DataFrame()
    merged_df['Timestamp'] = dataframes['sensor_accel_0']['timestamp']
    
    # Add accelerometer data
    if 'sensor_accel_0' in dataframes:
        accel_df = dataframes['sensor_accel_0']
        merged_df['Accelerometer_X'] = accel_df['x']
        merged_df['Accelerometer_Y'] = accel_df['y']
        merged_df['Accelerometer_Z'] = accel_df['z']
    
    # Add gyroscope data
    if 'sensor_gyro_0' in dataframes:
        # Resample to match timestamps
        gyro_df = dataframes['sensor_gyro_0']
        # Create temporary dataframes for interpolation
        temp_gyro = pd.DataFrame()
        temp_gyro['timestamp'] = gyro_df['timestamp']
        temp_gyro['x'] = gyro_df['x']
        temp_gyro['y'] = gyro_df['y']
        temp_gyro['z'] = gyro_df['z']
        # Set timestamp as index for interpolation
        temp_gyro.set_index('timestamp', inplace=True)
        # Reindex to match merged_df timestamps with interpolation
        temp_gyro = temp_gyro.reindex(index=merged_df['Timestamp'], method='nearest')
        # Add to merged dataframe
        merged_df['Gyroscope_X'] = temp_gyro['x'].values
        merged_df['Gyroscope_Y'] = temp_gyro['y'].values
        merged_df['Gyroscope_Z'] = temp_gyro['z'].values
    
    # Add magnetometer data
    if 'sensor_mag_0' in dataframes:
        mag_df = dataframes['sensor_mag_0']
        # Create temporary dataframes for interpolation
        temp_mag = pd.DataFrame()
        temp_mag['timestamp'] = mag_df['timestamp']
        temp_mag['x'] = mag_df['x']
        temp_mag['y'] = mag_df['y']
        temp_mag['z'] = mag_df['z']
        # Set timestamp as index for interpolation
        temp_mag.set_index('timestamp', inplace=True)
        # Reindex to match merged_df timestamps with interpolation
        temp_mag = temp_mag.reindex(index=merged_df['Timestamp'], method='nearest')
        # Add to merged dataframe
        merged_df['Magnetometer_X'] = temp_mag['x'].values
        merged_df['Magnetometer_Y'] = temp_mag['y'].values
        merged_df['Magnetometer_Z'] = temp_mag['z'].values
    
    # Add attitude data
    if 'vehicle_attitude_0' in dataframes:
        att_df = dataframes['vehicle_attitude_0']
        # Create temporary dataframes for interpolation
        temp_att = pd.DataFrame()
        temp_att['timestamp'] = att_df['timestamp']
        temp_att['q[0]'] = att_df['q[0]']
        temp_att['q[1]'] = att_df['q[1]']
        temp_att['q[2]'] = att_df['q[2]']
        temp_att['q[3]'] = att_df['q[3]']
        # Set timestamp as index for interpolation
        temp_att.set_index('timestamp', inplace=True)
        # Reindex to match merged_df timestamps with interpolation
        temp_att = temp_att.reindex(index=merged_df['Timestamp'], method='nearest')
        
        # Add quaternion data
        merged_df['Quat_w'] = temp_att['q[0]'].values
        merged_df['Quat_x'] = temp_att['q[1]'].values
        merged_df['Quat_y'] = temp_att['q[2]'].values
        merged_df['Quat_z'] = temp_att['q[3]'].values
        
        # Calculate roll, pitch, yaw from quaternions
        # This function converts quaternions to Euler angles
        def quaternion_to_euler(w, x, y, z):
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
        
        # Apply the conversion
        roll_list = []
        pitch_list = []
        yaw_list = []
        
        for i in range(len(merged_df)):
            roll, pitch, yaw = quaternion_to_euler(
                merged_df['Quat_w'].iloc[i],
                merged_df['Quat_x'].iloc[i],
                merged_df['Quat_y'].iloc[i],
                merged_df['Quat_z'].iloc[i]
            )
            roll_list.append(roll)
            pitch_list.append(pitch)
            yaw_list.append(yaw)
            
        merged_df['Roll'] = roll_list
        merged_df['Pitch'] = pitch_list
        merged_df['Yaw'] = yaw_list
        
        # Convert to degrees if needed
        # merged_df['Roll'] = np.degrees(merged_df['Roll'])
        # merged_df['Pitch'] = np.degrees(merged_df['Pitch'])
        # merged_df['Yaw'] = np.degrees(merged_df['Yaw'])
    
    # Make sure we have all the columns from data_new.csv
    required_columns = [
        'Timestamp', 
        'Accelerometer_X', 'Accelerometer_Y', 'Accelerometer_Z',
        'Gyroscope_X', 'Gyroscope_Y', 'Gyroscope_Z',
        'Magnetometer_X', 'Magnetometer_Y', 'Magnetometer_Z',
        'Roll', 'Pitch', 'Yaw',
        'Quat_w', 'Quat_x', 'Quat_y', 'Quat_z'
    ]
    
    # Fill in any missing columns with NaN
    for col in required_columns:
        if col not in merged_df.columns:
            print(f"Warning: Column {col} is missing and will be filled with NaN values")
            merged_df[col] = np.nan
    
    # Reorder columns to match data_new.csv
    merged_df = merged_df[required_columns]
    
    # Save to CSV
    print(f"Saving merged data to {output_file}")
    merged_df.to_csv(output_file, index=False)
    print(f"Successfully saved merged data with {len(merged_df)} rows")
    
else:
    print("Error: No accelerometer data found. Cannot create base timestamps.")
    exit(1)
