import asyncio
from mavsdk import System

async def main():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://192.168.124.169:14556")  # Replace with your connection string
# import asyncio
# from mavsdk import System
# #IP ethernet: 10.42.0.96
# async def get_imu_data():
#     # Connect to the drone via UDP
#     drone = System()
#     print("Running")
#     await drone.connect(system_address="udp://192.168.124.169:14556")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Subscribe to attitude updates
    print("Fetching quaternion readings...")
    async for attitude in drone.telemetry.attitude_quaternion():
        q0, q1, q2, q3 = attitude.w, attitude.x, attitude.y, attitude.z
        print(f"Quaternion: q0={q0}, q1={q1}, q2={q2}, q3={q3}")
        await asyncio.sleep(0.5)  # Fetch at a regular interval

# Run the async function
if __name__ == "__main__":
    asyncio.run(main())
