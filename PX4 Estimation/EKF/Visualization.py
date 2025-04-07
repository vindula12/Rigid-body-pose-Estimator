#!/usr/bin/env python3
"""
EKF2 Results Visualization Tool

This script reads the filtered output CSV and original sensor data CSV
and generates visualizations to help analyze the filter performance.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import argparse
import os

def load_data(filtered_file, original_file=None):
    """Load the filtered and original data files"""
    filtered_data = pd.read_csv(filtered_file)
    
    original_data = None
    if original_file:
        original_data = pd.read_csv(original_file)
    
    return filtered_data, original_data

def plot_euler_angles(filtered_data):
    """Plot the Euler angles (roll, pitch, yaw) over time"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    axes[0].plot(filtered_data['Timestamp'], filtered_data['Roll'], 'r-')
    axes[0].set_ylabel('Roll (degrees)')
    axes[0].set_title('Roll Angle')
    axes[0].grid(True)
    
    axes[1].plot(filtered_data['Timestamp'], filtered_data['Pitch'], 'g-')
    axes[1].set_ylabel('Pitch (degrees)')
    axes[1].set_title('Pitch Angle')
    axes[1].grid(True)
    
    axes[2].plot(filtered_data['Timestamp'], filtered_data['Yaw'], 'b-')
    axes[2].set_ylabel('Yaw (degrees)')
    axes[2].set_title('Yaw Angle')
    axes[2].set_xlabel('Timestamp')
    axes[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('euler_angles.png')
    print("Saved euler angles plot to: euler_angles.png")
    return fig

def plot_quaternions(filtered_data):
    """Plot the quaternion components over time"""
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    
    axes[0].plot(filtered_data['Timestamp'], filtered_data['Quat_w'], 'k-')
    axes[0].set_ylabel('w')
    axes[0].set_title('Quaternion - w component')
    axes[0].grid(True)
    
    axes[1].plot(filtered_data['Timestamp'], filtered_data['Quat_x'], 'r-')
    axes[1].set_ylabel('x')
    axes[1].set_title('Quaternion - x component')
    axes[1].grid(True)
    
    axes[2].plot(filtered_data['Timestamp'], filtered_data['Quat_y'], 'g-')
    axes[2].set_ylabel('y')
    axes[2].set_title('Quaternion - y component')
    axes[2].grid(True)
    
    axes[3].plot(filtered_data['Timestamp'], filtered_data['Quat_z'], 'b-')
    axes[3].set_ylabel('z')
    axes[3].set_title('Quaternion - z component')
    axes[3].set_xlabel('Timestamp')
    axes[3].grid(True)
    
    plt.tight_layout()
    plt.savefig('quaternions.png')
    print("Saved quaternion plot to: quaternions.png")
    return fig

def plot_3d_orientation(filtered_data, sample_rate=10):
    """Create a 3D animation showing orientation changes"""
    # Sample the data to reduce plot complexity
    sampled_data = filtered_data.iloc[::sample_rate, :]
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Create coordinate axes for each orientation
    axis_length = 1.0
    
    # Original XYZ coordinate axes
    ax.quiver(0, 0, 0, axis_length, 0, 0, color='r', label='X axis')
    ax.quiver(0, 0, 0, 0, axis_length, 0, color='g', label='Y axis')
    ax.quiver(0, 0, 0, 0, 0, axis_length, color='b', label='Z axis')
    
    # Plot coordinate frame for each orientation
    arrow_colors = [plt.cm.jet(i/len(sampled_data)) for i in range(len(sampled_data))]
    
    for i, row in enumerate(sampled_data.iterrows()):
        idx, data = row
        quat = [data['Quat_w'], data['Quat_x'], data['Quat_y'], data['Quat_z']]
        
        # Create rotation matrix from quaternion
        rot = R.from_quat([quat[1], quat[2], quat[3], quat[0]])  # scipy uses x,y,z,w format
        rot_matrix = rot.as_matrix()
        
        # Calculate rotated axes
        x_axis = rot_matrix @ np.array([axis_length, 0, 0])
        y_axis = rot_matrix @ np.array([0, axis_length, 0])
        z_axis = rot_matrix @ np.array([0, 0, axis_length])
        
        # Plot rotated coordinate frame
        alpha = 0.1 + 0.9 * (i / len(sampled_data))  # Fade in over time
        ax.quiver(0, 0, 0, x_axis[0], x_axis[1], x_axis[2], color=arrow_colors[i], alpha=alpha, arrow_length_ratio=0.1)
        ax.quiver(0, 0, 0, y_axis[0], y_axis[1], y_axis[2], color=arrow_colors[i], alpha=alpha, arrow_length_ratio=0.1)
        ax.quiver(0, 0, 0, z_axis[0], z_axis[1], z_axis[2], color=arrow_colors[i], alpha=alpha, arrow_length_ratio=0.1)
    
    # Set equal aspect ratio
    ax.set_box_aspect([1, 1, 1])
    ax.set_xlim([-axis_length, axis_length])
    ax.set_ylim([-axis_length, axis_length])
    ax.set_zlim([-axis_length, axis_length])
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('3D Orientation Visualization')
    
    plt.savefig('3d_orientation.png', dpi=300)
    print("Saved 3D orientation plot to: 3d_orientation.png")
    return fig

def compare_with_original(filtered_data, original_data):
    """Compare filtered output with original sensor data"""
    if original_data is None:
        print("Original data not provided, skipping comparison.")
        return None
    
    # Normalize accelerometer data to use as a simple reference for roll/pitch
    # (This is a rough comparison, not accurate but useful for visualization)
    acc_magnitude = np.sqrt(
        original_data['Accelerometer_X']**2 + 
        original_data['Accelerometer_Y']**2 + 
        original_data['Accelerometer_Z']**2
    )
    
    # Simple conversion from accelerometer to roll/pitch (ignoring dynamic acceleration)
    acc_roll = np.arctan2(original_data['Accelerometer_Y'], original_data['Accelerometer_Z']) * 180 / np.pi
    acc_pitch = np.arctan2(-original_data['Accelerometer_X'], 
                          np.sqrt(original_data['Accelerometer_Y']**2 + original_data['Accelerometer_Z']**2)) * 180 / np.pi
    
    # Simple conversion from magnetometer to yaw (ignoring tilt)
    mag_yaw = np.arctan2(original_data['Magnetometer_Y'], original_data['Magnetometer_X']) * 180 / np.pi
    
    # Create comparison plot
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    axes[0].plot(filtered_data['Timestamp'], filtered_data['Roll'], 'r-', label='EKF Roll')
    axes[0].plot(original_data['Timestamp'], acc_roll, 'r--', alpha=0.5, label='Accel-based Roll')
    axes[0].set_ylabel('Roll (degrees)')
    axes[0].set_title('Roll Angle Comparison')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(filtered_data['Timestamp'], filtered_data['Pitch'], 'g-', label='EKF Pitch')
    axes[1].plot(original_data['Timestamp'], acc_pitch, 'g--', alpha=0.5, label='Accel-based Pitch')
    axes[1].set_ylabel('Pitch (degrees)')
    axes[1].set_title('Pitch Angle Comparison')
    axes[1].legend()
    axes[1].grid(True)
    
    axes[2].plot(filtered_data['Timestamp'], filtered_data['Yaw'], 'b-', label='EKF Yaw')
    axes[2].plot(original_data['Timestamp'], mag_yaw, 'b--', alpha=0.5, label='Mag-based Yaw')
    axes[2].set_ylabel('Yaw (degrees)')
    axes[2].set_xlabel('Timestamp')
    axes[2].set_title('Yaw Angle Comparison')
    axes[2].legend()
    axes[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('euler_angles_comparison.png')
    print("Saved euler angles comparison plot to: euler_angles_comparison.png")
    
    return fig

def main():
    """Main function to process and visualize EKF results"""
    parser = argparse.ArgumentParser(description='Visualize EKF2 filtered output')
    parser.add_argument('filtered_file', help='Path to the filtered output CSV file')
    parser.add_argument('--original', help='Path to the original sensor data CSV file', default=None)
    parser.add_argument('--output-dir', help='Directory to save visualizations', default='visualizations')
    
    args = parser.parse_args()
    
    # Create output directory if it doesn't exist
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)
    
    # Change to output directory
    os.chdir(args.output_dir)
    
    # Load data
    filtered_data, original_data = load_data(args.filtered_file, args.original)
    
    print(f"Loaded filtered data with {len(filtered_data)} samples")
    if original_data is not None:
        print(f"Loaded original data with {len(original_data)} samples")
    
    # Generate plots
    print("Generating Euler angles plot...")
    plot_euler_angles(filtered_data)
    
    print("Generating quaternion components plot...")
    plot_quaternions(filtered_data)
    
    print("Generating 3D orientation visualization...")
    plot_3d_orientation(filtered_data)
    
    if original_data is not None:
        print("Generating comparison with original data...")
        compare_with_original(filtered_data, original_data)
    
    print(f"All visualizations saved to: {args.output_dir}/")

if __name__ == "__main__":
    main()

def compare_with_original(filtered_data, original_data):
    """Compare filtered output with original sensor data"""
    if original_data is None:
        print("Original data not provided, skipping comparison.")
        return None
    
    # Normalize accelerometer data to use as a simple reference for roll/pitch
    # (This is a very rough comparison, not accurate but useful for visualization)
    acc_magnitude = np.sqrt(
        original_data['Accelerometer_X']**2 + 
        original_data['Accelerometer_Y']**2 + 
        original_data['Accelerometer_Z']**2
    )
    
    # Simple conversion from accelerometer to roll/pitch (ignoring dynamic acceleration)
    acc_roll = np.arctan2(original_data['Accelerometer_Y'], original_data['Accelerometer_Z']) * 180 / np.pi
    acc_pitch = np.arctan2(-original_data['Accelerometer_X'], 
                          np.sqrt(original_data['Accelerometer_Y']**2 + original_data['Accelerometer_Z']**2)) * 180 / np.pi
    
    # Simple conversion from magnetometer to yaw (ignoring tilt)
    mag_yaw = np.arctan2(original_data['Magnetometer_Y'], original_data['Magnetometer_X']) * 180 / np.pi
    
    # Create comparison plot
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    axes[0].plot(filtered_data['Timestamp'], filtered_data['Roll'], 'r-', label='EKF Roll')
    axes[0].plot(original_data['Timestamp'], acc_roll, 'r--', alpha=0.5, label='Accel-based Roll')
    axes[0].set_ylabel('Roll (degrees)')
    axes[0].set_title('Roll Angle Comparison')
    axes[0].legend()
    axes[0].grid(True)
    
    axes[1].plot(filtered_data['Timestamp'], filtered_data['Pitch'], 'g-', label='EKF Pitch')
    axes[1].plot(original_data['Timestamp'], acc_pitch, 'g--', alpha=0.5, label='Accel-based Pitch')
    axes[1].set_ylabel('Pitch (degrees)')
    axes[1].set_title('Pitch Angle Comparison')
    axes[1].legend()
    axes[1].grid(True)
    
    axes[2].plot(filtered_data['Timestamp'], filtered_data['Yaw'], 'b-', label='EKF Yaw')
    axes[2].plot(original_data['Timestamp'], mag_yaw, 'b--', alpha=0.5, label='Mag-based Yaw')
    axes[2].set_ylabel('Yaw (degrees)')
    axes[2].set_xlabel('Timestamp')
    axes[2].set_title('Yaw Angle Comparison')
    axes[2].legend()
    axes[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('euler_angles_comparison.png')
    print("Saved euler angles comparison plot to: euler_angles_comparison.png")
    
    # Plot raw sensor data
    fig2, axes2 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # Accelerometer
    axes2[0].plot(original_data['Timestamp'], original_data['Accelerometer_X'], 'r-', label='X')
    axes2[0].plot(original_data['Timestamp'], original_data['Accelerometer_Y'], 'g-', label='Y')
    axes2[0].plot(original_data['Timestamp'], original_data['Accelerometer_Z'], 'b-', label='Z')
    axes2[0].set_ylabel('Acceleration (m/s²)')
    axes2[0].set_title('Accelerometer Data')
    axes2[0].legend()
    axes2[0].grid(True)
    
    # Gyroscope
    axes2[1].plot(original_data['Timestamp'], original_data['Gyroscope_X'], 'r-', label='X')
    axes2[1].plot(original_data['Timestamp'], original_data['Gyroscope_Y'], 'g-', label='Y')
    axes2[1].plot(original_data['Timestamp'], original_data['Gyroscope_Z'], 'b-', label='Z')
    axes2[1].set_ylabel('Angular Velocity (rad/s)')
    axes2[1].set_title('Gyroscope Data')
    axes2[1].legend()
    axes2[1].grid(True)
    
    # Magnetometer
    axes2[2].plot(original_data['Timestamp'], original_data['Magnetometer_X'], 'r-', label='X')
    axes2[2].plot(original_data['Timestamp'], original_data['Magnetometer_Y'], 'g-', label='Y')
    axes2[2].plot(original_data['Timestamp'], original_data['Magnetometer_Z'], 'b-', label='Z')
    axes2[2].set_ylabel('Magnetic Field')
    axes2[2].set_xlabel('Timestamp')
    axes2[2].set_title('Magnetometer Data')
    axes2[2].legend()
    axes2[2].grid(True)
    
    plt.tight_layout()
    plt.savefig('raw_sensor_data.png')
    print("Saved raw sensor data plot to: raw_sensor_data.png")
    
    return fig