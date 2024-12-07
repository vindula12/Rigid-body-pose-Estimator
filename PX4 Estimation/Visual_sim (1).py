import asyncio
import matplotlib                                                                                                                                             # type: ignore
import numpy as np                                                                                                                                             # type: ignore
from matplotlib import pyplot as plt                                                                                                                         # type: ignore
from mpl_toolkits.mplot3d import Axes3D                                                                                                                                     # type: ignore
from mavsdk import System                                                                                                                                            # type: ignore

matplotlib.use('TkAgg')  # Use TkAgg for compatibility on Ubuntu

# Initialize data for plotting
roll_data = []
pitch_data = []
yaw_data = []

# Initialize 3D plot
plt.ion()  # Enable interactive mode
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Draw the rotor and stand
def draw_rotor(ax):
    ax.clear()
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-1, 3])
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # Draw stand
    ax.plot([0, 0], [0, 0], [0, 1], 'k-', linewidth=2)

    # Draw rotor arms
    arm_length = 1.0
    arm_positions = np.array([[arm_length, 0, 1], [-arm_length, 0, 1],
                               [0, arm_length, 1], [0, -arm_length, 1]])

    for pos in arm_positions:
        ax.plot([0, pos[0]], [0, pos[1]], [1, pos[2]], 'b-', linewidth=1.5)

    return arm_positions

# Rotate the rotor using roll, pitch, and yaw angles
def rotate_rotor(ax, roll, pitch, yaw):
    # Draw stand and base
    arm_positions = draw_rotor(ax)
    
    # Compute rotation matrix
    roll_rad, pitch_rad, yaw_rad = np.radians(roll), np.radians(pitch), np.radians(yaw)
    R_roll = np.array([[1, 0, 0],
                        [0, np.cos(roll_rad), -np.sin(roll_rad)],
                        [0, np.sin(roll_rad), np.cos(roll_rad)]])
    R_pitch = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                         [0, 1, 0],
                         [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    R_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                       [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                       [0, 0, 1]])

    R = R_yaw @ R_pitch @ R_roll

    # Apply rotation to rotor arms
    for pos in arm_positions:
        rotated_pos = R @ pos
        ax.plot([0, rotated_pos[0]], [0, rotated_pos[1]], [1, rotated_pos[2] + 1], 'r-', linewidth=2)

    plt.pause(0.05)  # Allow time for the plot to update

async def get_imu_data():
    # Connect to the drone via UDP
    drone = System()
    print("Running")
    await drone.connect(system_address="udp://192.168.142.251:14556")  # Adjust the port number if necessary

    print("Waiting for the drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to drone")
            break

    # Infinite loop to fetch and display IMU data
    while True:
        # Fetch attitude (roll, pitch, yaw)
        async for imu_data in drone.telemetry.attitude_euler():
            roll = imu_data.roll_deg
            pitch = imu_data.pitch_deg
            yaw = imu_data.yaw_deg
            roll_data.append(roll)
            pitch_data.append(pitch)
            yaw_data.append(yaw)
            print(f"Roll: {roll:.2f} degrees, Pitch: {pitch:.2f} degrees, Yaw: {yaw:.2f} degrees")
            break  # Fetch one data point at a time

        # Update 3D plot
        rotate_rotor(ax, roll, pitch, yaw)

        await asyncio.sleep(0.05)  # Adjust the frequency of updates (e.g., 0.05s delay)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(get_imu_data())
import asyncio
import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mavsdk import System

matplotlib.use('TkAgg')  # Use TkAgg for compatibility on Ubuntu

# Initialize data for plotting
roll_data = []
pitch_data = []
yaw_data = []

# Initialize 3D plot
plt.ion()  # Enable interactive mode
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Draw the stand and twin rotors
def draw_twin_rotor(ax):
    ax.clear()
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-1, 3])
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('3D Twin Rotor Visualization')

    # Draw stand
    ax.plot([0, 0], [0, 0], [0, 1], 'k-', linewidth=3)

    # Rotor positions
    rotor1_position = np.array([1.5, 0, 1])
    rotor2_position = np.array([-1.5, 0, 1])

    # Draw rotor arms
    ax.plot([0, rotor1_position[0]], [0, rotor1_position[1]], [1, rotor1_position[2]], 'b-', linewidth=2)
    ax.plot([0, rotor2_position[0]], [0, rotor2_position[1]], [1, rotor2_position[2]], 'b-', linewidth=2)

    # Draw rotors as circles
    circle_points = 20
    angle = np.linspace(0, 2 * np.pi, circle_points)
    rotor_radius = 0.5

    # Rotor 1 circle
    x_circle1 = rotor1_position[0] + rotor_radius * np.cos(angle)
    y_circle1 = rotor1_position[1] + rotor_radius * np.sin(angle)
    z_circle1 = np.full_like(x_circle1, rotor1_position[2])
    ax.plot(x_circle1, y_circle1, z_circle1, 'r-')

    # Rotor 2 circle
    x_circle2 = rotor2_position[0] + rotor_radius * np.cos(angle)
    y_circle2 = rotor2_position[1] + rotor_radius * np.sin(angle)
    z_circle2 = np.full_like(x_circle2, rotor2_position[2])
    ax.plot(x_circle2, y_circle2, z_circle2, 'g-')

    return rotor1_position, rotor2_position

# Rotate the twin rotor using roll, pitch, and yaw angles
def rotate_twin_rotor(ax, roll, pitch, yaw):
    rotor1_position, rotor2_position = draw_twin_rotor(ax)

    # Compute rotation matrix
    roll_rad, pitch_rad, yaw_rad = np.radians(roll), np.radians(pitch), np.radians(yaw)
    R_roll = np.array([[1, 0, 0],
                        [0, np.cos(roll_rad), -np.sin(roll_rad)],
                        [0, np.sin(roll_rad), np.cos(roll_rad)]])
    R_pitch = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                         [0, 1, 0],
                         [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    R_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                       [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                       [0, 0, 1]])

    R = R_yaw @ R_pitch @ R_roll

    # Rotate and update rotor positions
    for rotor_position in [rotor1_position, rotor2_position]:
        rotated_position = R @ rotor_position
        ax.plot([0, rotated_position[0]], [0, rotated_position[1]], [1, rotated_position[2]], 'b-', linewidth=2)

    plt.pause(0.05)  # Allow time for the plot to update

async def get_imu_data():
    # Connect to the drone via UDP
    drone = System()
    print("Running")
    await drone.connect(system_address="udp://192.168.43.54:14556")  # Adjust the port number if necessary

    print("Waiting for the drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to drone")
            break

    # Infinite loop to fetch and display IMU data
    while True:
        # Fetch attitude (roll, pitch, yaw)
        async for imu_data in drone.telemetry.attitude_euler():
            roll = imu_data.roll_deg
            pitch = imu_data.pitch_deg 
            yaw = imu_data.yaw_deg
            roll_data.append(roll)
            pitch_data.append(pitch)
            yaw_data.append(yaw)
            print(f"Roll: {roll:.2f} degrees, Pitch: {pitch:.2f} degrees, Yaw: {yaw:.2f} degrees")
            break  # Fetch one data point at a time

        # Update 3D twin rotor
        rotate_twin_rotor(ax, roll, pitch, yaw)

        await asyncio.sleep(0.05)  # Adjust the frequency of updates (e.g., 0.05s delay)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(get_imu_data())
