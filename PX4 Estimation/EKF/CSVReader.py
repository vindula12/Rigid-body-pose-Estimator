"""
EKF2 CSV Processing Application (Python Version)
Modified from PX4 EKF2 implementation to work with CSV data
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import time

class SimpleEKF:
    """
    Simplified Extended Kalman Filter for attitude estimation
    """
    def __init__(self):
        # Initialize state (quaternion [w,x,y,z])
        self.x = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Initialize covariance
        self.P = np.eye(4) * 0.1
        
        # Process noise
        self.gyro_noise = 0.01
        self.accel_noise = 0.1
        self.mag_noise = 0.1
        
        # Measurement noise
        self.R_accel = np.eye(3) * 0.5
        self.R_mag = np.eye(3) * 0.5
        
        # Earth's magnetic field reference (will be calibrated)
        self.mag_earth = np.array([1.0, 0.0, 0.0])
        
        # Gravity reference
        self.gravity = np.array([0.0, 0.0, 9.81])
    
    def update(self, data, dt):
        """Process one timestep"""
        # 1. State prediction using gyroscope
        gyro = np.array([data['Gyroscope_X'], data['Gyroscope_Y'], data['Gyroscope_Z']])
        self.predict(gyro, dt)
        
        # 2. Correction using accelerometer and magnetometer
        accel = np.array([data['Accelerometer_X'], data['Accelerometer_Y'], data['Accelerometer_Z']])
        mag = np.array([data['Magnetometer_X'], data['Magnetometer_Y'], data['Magnetometer_Z']])
        
        self.update_accel(accel)
        self.update_mag(mag)
    
    def predict(self, gyro, dt):
        """State prediction step"""
        # Convert gyro measurements to quaternion rate
        q = self.x
        
        # Simplified quaternion integration from angular velocity
        wx, wy, wz = gyro
        
        # Quaternion derivative from angular velocity
        q_dot = 0.5 * np.array([
            -q[1]*wx - q[2]*wy - q[3]*wz,
            q[0]*wx + q[2]*wz - q[3]*wy,
            q[0]*wy - q[1]*wz + q[3]*wx,
            q[0]*wz + q[1]*wy - q[2]*wx
        ])
        
        # Integrate quaternion
        self.x = self.x + dt * q_dot
        
        # Normalize quaternion
        self.x = self.x / np.linalg.norm(self.x)
        
        # Update covariance (simplified)
        Q = np.eye(4) * self.gyro_noise * dt
        self.P = self.P + Q
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q
        
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]
        ])
        
        return R
    
    def update_accel(self, accel):
        """Update using accelerometer measurements"""
        # Current rotation matrix from NED to body
        R = self.quaternion_to_rotation_matrix(self.x)
        
        # Predicted acceleration in body frame (gravity rotated)
        accel_pred = R.T @ self.gravity
        
        # Normalized measured acceleration
        accel_norm = accel / np.linalg.norm(accel) * np.linalg.norm(self.gravity)
        
        # Innovation (difference between measurement and prediction)
        y = accel_norm - accel_pred
        
        # Simplified approach: use a simple gain for correction
        gain = 0.01
        
        # Compute correction
        correction = gain * y
        
        # Apply correction to state
        q_correction = np.array([0, correction[0], correction[1], correction[2]])
        self.x = self.x + q_correction
        self.x = self.x / np.linalg.norm(self.x)
    
    def update_mag(self, mag):
        """Update using magnetometer measurements"""
        # Current rotation matrix from NED to body
        R = self.quaternion_to_rotation_matrix(self.x)
        
        # Use mag readings to slowly update mag_earth reference
        if np.linalg.norm(self.mag_earth) < 0.1:
            # Initialize on first update
            self.mag_earth = R @ (mag / np.linalg.norm(mag))
        else:
            # Slowly update reference direction
            self.mag_earth = 0.999 * self.mag_earth + 0.001 * (R @ (mag / np.linalg.norm(mag)))
            self.mag_earth = self.mag_earth / np.linalg.norm(self.mag_earth)
        
        # Simplified correction approach for yaw
        gain = 0.005
        
        # Projected magnetic field in horizontal plane
        mag_h = mag.copy()
        mag_h[2] = 0
        if np.linalg.norm(mag_h) > 0:
            mag_h = mag_h / np.linalg.norm(mag_h)
        
        # Predicted direction based on current attitude
        mag_pred = R.T @ self.mag_earth
        mag_pred[2] = 0
        if np.linalg.norm(mag_pred) > 0:
            mag_pred = mag_pred / np.linalg.norm(mag_pred)
        
        # Compute correction (cross product for rotation error)
        correction = gain * np.cross(mag_h, mag_pred)
        
        # Apply only yaw correction
        q_correction = np.array([0, 0, 0, correction[2]])
        self.x = self.x + q_correction
        self.x = self.x / np.linalg.norm(self.x)
    
    def get_euler_angles(self):
        """Get the current attitude as Euler angles (roll, pitch, yaw) in degrees"""
        r = Rotation.from_quat([self.x[1], self.x[2], self.x[3], self.x[0]])  # scipy uses [x,y,z,w]
        return r.as_euler('xyz', degrees=True)
    
    def get_quaternion(self):
        """Get the current quaternion"""
        return self.x
    
    def print_state(self):
        """Print the current state"""
        euler = self.get_euler_angles()
        print(f"Roll: {euler[0]:.2f} deg, Pitch: {euler[1]:.2f} deg, Yaw: {euler[2]:.2f} deg")


def process_csv_with_ekf(input_file, output_file):
    """Process CSV data with EKF and save results to output file"""
    print(f"Reading sensor data from: {input_file}")
    
    # Read the CSV file
    try:
        data = pd.read_csv(input_file)
        print(f"Loaded {len(data)} sensor samples.")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Initialize EKF
    ekf = SimpleEKF()
    
    # Prepare output data structure
    output_data = {
        'Timestamp': [],
        'Roll': [],
        'Pitch': [],
        'Yaw': [],
        'Quat_w': [],
        'Quat_x': [],
        'Quat_y': [],
        'Quat_z': []
    }
    
    start_time = time.time()
    
    # Process each sample
    for i, row in data.iterrows():
        # Calculate dt (time difference between samples)
        dt = 0.01  # Default 10ms if we can't calculate
        
        if i > 0:
            dt = row['Timestamp'] - data.iloc[i-1]['Timestamp']
            # Convert to seconds if needed (if timestamps are in milliseconds)
            if dt > 1:
                dt /= 1000.0
        
        # Bound dt to reasonable values
        if dt <= 0 or dt > 1.0:
            dt = 0.01
        
        # Update filter with current measurement
        ekf.update(row, dt)
        
        # Get filtered states
        euler = ekf.get_euler_angles()
        quat = ekf.get_quaternion()
        
        # Store output
        output_data['Timestamp'].append(row['Timestamp'])
        output_data['Roll'].append(euler[0])
        output_data['Pitch'].append(euler[1])
        output_data['Yaw'].append(euler[2])
        output_data['Quat_w'].append(quat[0])
        output_data['Quat_x'].append(quat[1])
        output_data['Quat_y'].append(quat[2])
        output_data['Quat_z'].append(quat[3])
        
        # Print progress every 10%
        if i % (len(data) // 10) == 0:
            print(f"Processing: {i * 100 // len(data)}% complete")
            ekf.print_state()
    
    end_time = time.time()
    print(f"Processing completed in {(end_time - start_time)*1000:.1f} ms")
    
    # Create and save output DataFrame
    output_df = pd.DataFrame(output_data)
    output_df.to_csv(output_file, index=False)
    print(f"Filtered output written to: {output_file}")
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    plt.subplot(3, 1, 1)
    plt.plot(output_df['Timestamp'], output_df['Roll'], 'r')
    plt.title('Roll Angle')
    plt.ylabel('Degrees')
    
    plt.subplot(3, 1, 2)
    plt.plot(output_df['Timestamp'], output_df['Pitch'], 'g')
    plt.title('Pitch Angle')
    plt.ylabel('Degrees')
    
    plt.subplot(3, 1, 3)
    plt.plot(output_df['Timestamp'], output_df['Yaw'], 'b')
    plt.title('Yaw Angle')
    plt.xlabel('Timestamp')
    plt.ylabel('Degrees')
    
    plt.tight_layout()
    plt.savefig('ekf_attitude_output.png')
    print("Generated plot: ekf_attitude_output.png")


if __name__ == "__main__":
    import sys
    
    # Default filenames
    input_file = "data.csv"
    output_file = "filtered_output.csv"
    
    # Override with command-line arguments if provided
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_file = sys.argv[2]
    
    process_csv_with_ekf(input_file, output_file)